//////////////////////////////////////////////////
// GyroM5v2 - M5StickC project:
//   Steering assisting unit for RC drift Car
// New Features from GyroM5v1:
//   Parameter setting by WiFi access
//   Variable output PWM frequency
//   PID computation by QuickPID library
//   Direction free in HW installation
// URL:
//   https://github.com/hshin-git/GyroM5
//////////////////////////////////////////////////
#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <QuickPID.h>


//////////////////////////////////////////////////
// Global constants
//////////////////////////////////////////////////
// Title name for LCD/HTML
const char TITLE[] = "GyroM5";

// WiFi parameters
const char WIFI_SSID[] = "GyroM5";
const char WIFI_PASS[] = "";
const IPAddress WIFI_IP(192,168,4,1);
const IPAddress WIFI_SUBNET(255,255,255,0);
WiFiServer WIFI_SERVER(80);

// PWM parameters
const int PWM_HERZ = 50;
const int PWM_BITS = 16;
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
//
const int PWM_DMAX = (1<<PWM_BITS);
const int PWM_CYCL = 1000000/PWM_HERZ;
const int PWM_WAIT = PWM_CYCL + 1000;
//
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;
const int RANGE_MAX = 120;

// GPIO parameters
const int CH1_IN = 26;
const int CH3_IN = 36;
const int CH1_OUT = 0;

// delay after button (msec)
const int DELAY = 300;



//////////////////////////////////////////////////
// QuickPID
//////////////////////////////////////////////////
float Setpoint, Input, Output;
float Kp=1.0, Ki=0.0, Kd=1.0;
QuickPID myPID(&Input, &Output, &Setpoint, Kp,Ki,Kd, QuickPID::DIRECT);


//////////////////////////////////////////////////
// LCD helpers
//////////////////////////////////////////////////
void lcd_clear(int bg_color=TFT_BLACK, int fg_color=TFT_WHITE) {
  M5.Lcd.fillScreen(bg_color);
  M5.Lcd.setTextColor(fg_color);
  M5.Lcd.setCursor(0,0);
}
bool lcd_header(char *text, bool forced=false) {
  static int lastTime = 0;
  if (forced || (lastTime + 500 < millis())) {
    lcd_clear();
    M5.Lcd.print(TITLE);
    M5.Lcd.print("@");
    M5.Lcd.println(text);
    lastTime = millis();
    return true;
  }
  return false;
}


//////////////////////////////////////////////////
// 5Vin watcher
//////////////////////////////////////////////////
//int lastVinTime = 0;
void axp_halt(){
  Wire1.beginTransmission(0x34);
  Wire1.write(0x32);
  Wire1.endTransmission();
  Wire1.requestFrom(0x34, 1);
  uint8_t buf = Wire1.read();
  Wire1.beginTransmission(0x34);
  Wire1.write(0x32);
  Wire1.write(buf | 0x80); // halt bit
  Wire1.endTransmission();
}
void vin_watch() {
  static int lastTime = 0;
  float vin = M5.Axp.GetVinData()*1.7 /1000;
  float usb = M5.Axp.GetVusbinData()*1.7 /1000;
  //Serial.printf("vin,usb = %f,%f\n",vin,usb);
  if ( vin < 3.0 && usb < 3.0 ) {
    if ( lastTime + 5000 < millis() ) {
      axp_halt();
    }
  } else {
    lastTime = millis();
  }
}


//////////////////////////////////////////////////
// PWM output frequency
//////////////////////////////////////////////////
int PWM_HERZ_ = 50;
int PWM_CYCL_ = 1000000/PWM_HERZ_;
void ch1_setHerz(int herz) {
  if (herz<50 || herz>500) return;
  PWM_HERZ_ = herz;
  PWM_CYCL_ = 1000000/PWM_HERZ_;
  ledcSetup(PWM_CH1,PWM_HERZ_,PWM_BITS);
  ledcAttachPin(CH1_OUT,PWM_CH1);
  ledcWrite(PWM_CH1,0);
}
void ch1_output(int usec) {
  int duty = map(usec, 0,PWM_CYCL_, 0,PWM_DMAX);
  ledcWrite(PWM_CH1,(usec>0? duty: 0));
}


//////////////////////////////////////////////////
// Parameter config by WiFi
//////////////////////////////////////////////////
Preferences GYROM5;
const char NVM_NAME[] = "GYROM5";
const char NVM_KEY[] = "CONF";

// GyroM5 parameters
const char *KEYS[] = {"KG","KP","KI","KD", "CH1","CH3","PWM", "MIN","MAX", "END",};
const int INIT[] = {50,50,20,5, 0,0,50, 1000,2000, 12345,};
int CONF[] = {50,50,20,5, 0,0,50, 1000,2000, 12345,};
enum INDEX {_KG=0,_KP,_KI,_KD, _CH1,_CH3,_PWM, _MIN,_MAX, _END,};
const int SIZE = sizeof(CONF)/sizeof(int);
const int TAIL = 3; // number of items after "PWM"


// HTML template
const char HTML_TEMPLATE[] = R"(
<!DOCTYPE HTML>
<html>
<head>
<meta name='viewport' content='width=device-width,initial-scale=1' />
<title>%s</title>
</head>
<body>
<form method='get' name='setting'>
<table>
<tr><th>name</th><th>range</th><th>value</th><th>description</th></tr>
<tr><td>KG</td><td><input type='range' name='KG' min='0' max='100' step='1' value='0' oninput='onInput(this)' /></td><td><span id='KG'>0</span></td><td>IMU gain G (0-100)</td></tr>
<tr><td>KP</td><td><input type='range' name='KP' min='0' max='100' step='1' value='0' oninput='onInput(this)' /></td><td><span id='KP'>0</span></td><td>PID gain P (0-100)</td></tr>
<tr><td>KI</td><td><input type='range' name='KI' min='0' max='100' step='1' value='0' oninput='onInput(this)' /></td><td><span id='KI'>0</span></td><td>PID gain I (0-100)</td></tr>
<tr><td>KD</td><td><input type='range' name='KD' min='0' max='100' step='1' value='0' oninput='onInput(this)' /></td><td><span id='KD'>0</span></td><td>PID gain D (0-100)</td></tr>
<tr><td>CH1</td><td><input type='range' name='CH1' min='0' max='1' step='1' value='0' oninput='onInput(this)' /></td><td><span id='CH1'>0</span></td><td>0:NOR, 1:REV</td></tr>
<tr><td>CH3</td><td><input type='range' name='CH3' min='0' max='5' step='1' value='0' oninput='onInput(this)' /></td><td><span id='CH3'>0</span></td><td>0:TB, 1:KG, 2:KP, 3:KI, 4:KD, 5:NO</td></tr>
<tr><td>PWM</td><td><input type='range' name='PWM' min='50' max='400' step='50' value='50' oninput='onInput(this)' /></td><td><span id='PWM'>50</span><td>PWM frequency (Hz)</td></tr>
</table>
<input type='submit' value='submit' />
<input type='button' value='reload' onclick='window.location=window.location.href.split("?")[0];' />
</form>
</body>
<script>
function onInput(obj) {
 document.getElementById(obj.name).textContent = obj.value;
}
function onLoad() {
 const CONFIG = {KG:%d,KP:%d,KI:%d,KD:%d,CH1:%d,CH3:%d,PWM:%d,};
 const INPUTS = document.getElementsByTagName('input');
 for (let key in CONFIG){ document.getElementsByName(key)[0].value = document.getElementById(key).textContent  = CONFIG[key]; } 
 if (%d) { for (var i=0;i<INPUTS.length-1; i++) { INPUTS[i].disabled = true; } }
}
window.onload = onLoad();
</script>
</html>
)";

// HTML buffer
char HTML_BUFFER[sizeof(HTML_TEMPLATE)+sizeof(TITLE)+4*SIZE];

// Web server for config
bool configAccepted = false;
void configLoop() {
  WiFiClient client = WIFI_SERVER.available();

  if (client) {
    //Serial.println("New Client.");
    String currentLine = "";

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        //Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html; charset=utf-8;");
            client.println();
            sprintf(HTML_BUFFER,HTML_TEMPLATE, TITLE,CONF[_KG],CONF[_KP],CONF[_KI],CONF[_KD],CONF[_CH1],CONF[_CH3],CONF[_PWM],0);
            client.println(HTML_BUFFER);
            break;
          } else if (currentLine.indexOf("GET /?") == 0) {
            int p1 = 0;
            int p2 = 0;
            int val = 0;
            for (int n=0; n<SIZE-TAIL; n++) {
              String key = KEYS[n];
              key = key + "=";
              p1 = currentLine.indexOf(key, p2) + key.length();
              p2 = currentLine.indexOf(n<SIZE-1? '&': ' ', p1);
              val = currentLine.substring(p1, p2).toInt();
              CONF[n] = val;
            }
            GYROM5.putBytes(NVM_KEY,&CONF,sizeof(CONF));
            ch1_setHerz(CONF[_PWM]);
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html; charset=utf-8;");
            client.println();
            sprintf(HTML_BUFFER,HTML_TEMPLATE, TITLE,CONF[_KG],CONF[_KP],CONF[_KI],CONF[_KD],CONF[_CH1],CONF[_CH3],CONF[_PWM],1);
            client.println(HTML_BUFFER);
            configAccepted = true;
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    //Serial.println("Client Disconnected.");
  }
}

void config_by_wifi() {
  //WIFI_SERVER.begin();
  //
  if (lcd_header("CONF",true)) {
    M5.Lcd.println("[B] CANCEL");
    M5.Lcd.println("SSID:"); M5.Lcd.printf(" %s\n",WIFI_SSID);
    M5.Lcd.println("PASS:"); M5.Lcd.printf(" %s\n",WIFI_PASS);
    M5.Lcd.print("IP:\n "); M5.Lcd.println(WIFI_IP);
  }
  //
  configAccepted = false;
  while (!configAccepted) {
    configLoop();
    vin_watch();
    M5.update();
    if (M5.BtnB.isPressed()) {
      delay(DELAY);
      return;
    }
  }
  //
  //WIFI_SERVER.end();
}


// conf ch1 end points
void config_ch1ends() {
  int ch1,val;
  for (int n=0; n<2; n++) {
    delay(DELAY);
    while (true) {
      ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
      val = map(ch1, 0,PWM_CYCL_, 0,PWM_DMAX);
      //ledcWrite(PWM_CH1,(ch1>0? val: 0));
      ch1_output(ch1);
      if (lcd_header("ENDS")) {
        M5.Lcd.println((n? "LEFT": "RIGHT"));
        M5.Lcd.printf("[A] SAVE\n");
        M5.Lcd.printf("[B] CANCEL\n");
        M5.Lcd.printf(" CH1:%6d\n",ch1);
        M5.Lcd.printf(" VAL:%6d\n",val);
      }
      M5.update();
      if (M5.BtnA.isPressed()) {
        if (ch1>0) CONF[(n? _MAX: _MIN)] = ch1;
        break;
      }
      if (M5.BtnB.isPressed()) {
        ch1_output(0);
        delay(DELAY);
        return;
      }
    } 
  }
  // save
  if (ch1>0) {
    if (CONF[_MAX] < CONF[_MIN]) {
      int temp = CONF[_MAX];
      CONF[_MAX] = CONF[_MIN];
      CONF[_MIN] = temp;
    }
    GYROM5.putBytes(NVM_KEY,&CONF,sizeof(CONF));
  }
  ch1_output(0);
  delay(DELAY);
}


//////////////////////////////////////////////////
// Low/High Pass Filters
//////////////////////////////////////////////////
float CH1ER_LPF[4];
float CH1ER_HPF[4];
float YRATE_LPF[4];
// LPF
void lpf_init(float buf[],float alpha) {
  buf[0] = 0.;
  buf[1] = alpha;
}
float lpf_update(float buf[],float x) {
  float yp = buf[0];
  float alpha = buf[1];
  float y = alpha*x + (1.-alpha)*yp;
  buf[0] = y;
  return y;
}
// HPF
void hpf_init(float buf[],float alpha) {
  buf[0] = 0.;
  buf[1] = 0.;
  buf[2] = alpha;
}
float hpf_update(float buf[],float x) {
  float yp = buf[0];
  float xp = buf[1];
  float alpha = buf[2];
  float y = alpha*(x - xp) + alpha*yp;
  buf[0] = y;
  buf[1] = x;
  return y;
}


//////////////////////////////////////////////////
// Initial calibrators
//////////////////////////////////////////////////
float CH1US_MEAN;
float OMEGA_MEAN[3];
float ACCEL_MEAN[3];

// Frequency counter
int getFrequency(bool getOnly = false) {
  static int lastTime = 0;
  static int count = 0;
  static int freq = 50;
  if (!getOnly) count = count + 1;
  if (lastTime + 1000 < millis()) {
    freq = count;
    count = 0;
    lastTime = millis();
  }
  return freq;
}

void call_calibration(void) {
  // wait
  lcd_header("WAIT",true);
  for (int n=0; n<10; n++) {
    while (pulseIn(CH1_IN,HIGH,PWM_WAIT)==0) vin_watch();
  }
  // zero
  for (int n=0; n<250; n++) {
    float omega[3],accel[3];
    int ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
    int hrz = getFrequency();
    M5.MPU6886.getGyroData(&omega[0],&omega[1],&omega[2]);
    M5.MPU6886.getAccelData(&accel[0],&accel[1],&accel[2]);
    CH1US_MEAN = (n==0? ch1: (CH1US_MEAN + ch1)/2.);
    for (int i=0; i<3; i++) {
      OMEGA_MEAN[i] = (n==0? omega[i]: (OMEGA_MEAN[i] + omega[i])/2.);
      ACCEL_MEAN[i] = (n==0? accel[i]: (ACCEL_MEAN[i] + accel[i])/2.);
    }
    //
    if (lcd_header("INIT",n==0)) {
      M5.Lcd.println("CH1(usec)");
      M5.Lcd.printf( "   %8.2f\n",CH1US_MEAN);
      M5.Lcd.printf( "   %6dHz\n",hrz);
      M5.Lcd.println("OMEGA(rad/s)");
      M5.Lcd.printf( " X:%8.2f\n",OMEGA_MEAN[0]);
      M5.Lcd.printf( " Y:%8.2f\n",OMEGA_MEAN[1]);
      M5.Lcd.printf( " Z:%8.2f\n",OMEGA_MEAN[2]);
      M5.Lcd.println("ACCEL(G)");
      M5.Lcd.printf( " X:%8.2f\n",ACCEL_MEAN[0]);
      M5.Lcd.printf( " Y:%8.2f\n",ACCEL_MEAN[1]);
      M5.Lcd.printf( " Z:%8.2f\n",ACCEL_MEAN[2]);
    }
  }
}

// yaw rate
float getYawRate(float *omega) {
  float w0 = (omega[0]-OMEGA_MEAN[0])*ACCEL_MEAN[0];
  float w1 = (omega[1]-OMEGA_MEAN[1])*ACCEL_MEAN[1];
  float w2 = (omega[2]-OMEGA_MEAN[2])*ACCEL_MEAN[2];
  return (w0 + w1 + w2) * M5.MPU6886.gRes;
}
// vertiacal accel
float getVertical(float *accel) {
  float gv = accel[0]*ACCEL_MEAN[0] + accel[1]*ACCEL_MEAN[1] + accel[2]*ACCEL_MEAN[2];
  return gv;
}
// horizontal accel
float getHorizontal(float *accel) {
  float gv = accel[0]*ACCEL_MEAN[0] + accel[1]*ACCEL_MEAN[1] + accel[2]*ACCEL_MEAN[2];
  float a0 = accel[0] - gv*ACCEL_MEAN[0];
  float a1 = accel[1] - gv*ACCEL_MEAN[1];
  float a2 = accel[2] - gv*ACCEL_MEAN[2];
  return sqrt(a0*a0 + a1*a1 + a2*a2);
}


//////////////////////////////////////////////////
// put your setup code here, to run once:
//////////////////////////////////////////////////
void setup() {
  // (1) Initialize M5StickC object
  M5.begin();
  M5.MPU6886.Init();
  M5.Axp.ScreenBreath(9);
  //while (!setCpuFrequencyMhz(80));  

  // (2) Initialize Preferences
  GYROM5.begin(NVM_NAME);
  GYROM5.getBytes(NVM_KEY, &CONF, sizeof(CONF));
  if (CONF[_END] != INIT[_END]) {
    GYROM5.putBytes(NVM_KEY, &INIT, sizeof(CONF));
    GYROM5.getBytes(NVM_KEY, &CONF, sizeof(CONF));
  }

  // (3) Initialize GPIO settings
  pinMode(CH1_IN,INPUT);
  pinMode(CH3_IN,INPUT);
  pinMode(CH1_OUT,OUTPUT);
  ch1_setHerz(CONF[_PWM]);
  //ledcSetup(PWM_CH1,PWM_HERZ,PWM_BITS);
  //ledcAttachPin(CH1_OUT,PWM_CH1);
  //ledcWrite(PWM_CH1,0);

  // (4) WiFi AP setting
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(DELAY);
  WiFi.softAPConfig(WIFI_IP, WIFI_IP, WIFI_SUBNET);
  IPAddress myIP = WiFi.softAPIP();
  WIFI_SERVER.begin();

  // (5) Initialize Filters
  lpf_init(CH1ER_LPF,2.0/ 50);
  hpf_init(CH1ER_HPF,50./100);
  lpf_init(YRATE_LPF,0.5);

  // (6) Initialize Zero points
  call_calibration();

  // (7) Initialize QuickPID
  Input = 0.0;
  Setpoint = 0.0;
  myPID.SetMode(QuickPID::TIMER);
  myPID.SetSampleTimeUs(1000000/getFrequency(true));
  myPID.SetTunings(CONF[_KP]/100.0,CONF[_KI]/500.0,CONF[_KD]/500.0);
  myPID.SetOutputLimits(CONF[_MIN]-CH1US_MEAN,CONF[_MAX]-CH1US_MEAN);

  // (8) Initialize Others
  //Serial.begin(115200);
}


//////////////////////////////////////////////////
// put your main code here, to run repeatedly:
//////////////////////////////////////////////////
// PWM widths in usec
int CH1_USEC = 0;
int CH2_USEC = 0;
int CH3_USEC = 0;

// IMU values
float OMEGA[3];
float ACCEL[3];

// Main loop counter
//int LOOP = 0;

// Main loop
void loop() {
  int ch1_duty,ch1_usec;
  int ch3_gain,KG,KP,KI,KD;
  float yrate,error,contr;
  
  // (1) Input PWM values
  CH1_USEC = pulseIn(CH1_IN,HIGH,PWM_WAIT);
  //if (LOOP%25 == 0) CH3_USEC = pulseIn(CH3_IN,HIGH,PWM_WAIT);

  // (2) Input IMU values
  M5.MPU6886.getGyroData(&OMEGA[0],&OMEGA[1],&OMEGA[2]);
  M5.MPU6886.getAccelData(&ACCEL[0],&ACCEL[1],&ACCEL[2]);

  // (3) Prepare PID gains
  KG = CONF[_KG];
  KP = CONF[_KP];
  KI = CONF[_KI];
  KD = CONF[_KD];
  ch3_gain = map(CH3_USEC, PULSE_MIN,PULSE_MAX, -RANGE_MAX,RANGE_MAX);
  switch (CONF[_CH3]) {
    case 1: KG = ch3_gain; break;
    case 2: KP = ch3_gain; break;
    case 3: KI = ch3_gain; break;
    case 4: KD = ch3_gain; break;
    case 5: KP = 50; KG = KI = KD = 0; break;
    default: break;
  }
  if (CONF[_CH1]) KG = -KG;
  
  // (4) Compute PID control
  yrate = getYawRate(OMEGA);
  //yrate = (OMEGA[2] - OMEGA_MEAN[2]) * M5.MPU6886.gRes;
  //error = (CH1_USEC - CH1US_MEAN) - (KG/1.0)*yrate;
  //contr =  (KP/50.0)*(error + (KI/50.0)*lpf_update(CH1ER_LPF,error) + (KD/50.0)*hpf_update(CH1ER_HPF,error));

  // QuickPID
  Setpoint = (CH1_USEC - CH1US_MEAN);
  //Input = (KG/1.0) * lpf_update(YRATE_LPF,yrate);
  Input = (KG/1.0) * yrate;
  myPID.Compute();
  ch1_usec = constrain(CH1US_MEAN + Output, CONF[_MIN],CONF[_MAX]);
  ch1_output((CH1_USEC>0? ch1_usec: 0));

  // convert usec to duty and constrain
  //ch1_usec = constrain(CH1US_MEAN + contr,CONF[_MIN],CONF[_MAX]);
  //ch1_duty = map(ch1_usec, 0,PWM_CYCL, 0,PWM_DMAX);
  //ledcWrite(PWM_CH1,(CH1_USEC>0? ch1_duty: 0));
  
  // (5) Update LCD
  if (lcd_header("HOME")) {
    M5.Lcd.println("INPUT (usec)");
    M5.Lcd.printf( " CH1:%6d\n", CH1_USEC);
    //M5.Lcd.printf( " CH2:%6d\n", CH2_USEC);
    M5.Lcd.printf( " CH3:%6d\n", CH3_USEC);
    M5.Lcd.println("YRATE (rad/s)");
    M5.Lcd.printf( " R:%8.2f\n", yrate);
    M5.Lcd.println("ACCEL (G)");
    M5.Lcd.printf( " H:%8.2f\n", getHorizontal(ACCEL));
    M5.Lcd.printf( " V:%8.2f\n", getVertical(ACCEL));
    //M5.Lcd.println("OMEGA (rad/s)");
    //M5.Lcd.printf( " X:%8.2f\n", OMEGA[0] *M5.MPU6886.gRes);
    //M5.Lcd.printf( " Y:%8.2f\n", OMEGA[1] *M5.MPU6886.gRes);
    //M5.Lcd.printf( " Z:%8.2f\n", OMEGA[2] *M5.MPU6886.gRes);
    //M5.Lcd.println("ACCEL (G)");
    //M5.Lcd.printf(" X:%8.2f\n", ACCEL[0]);
    //M5.Lcd.printf(" Y:%8.2f\n", ACCEL[1]);
    //M5.Lcd.printf(" Z:%8.2f\n", ACCEL[2]);
    M5.Lcd.println("PID (0-100)");
    M5.Lcd.printf( " G/P:%3d/%3d\n", KG,KP);
    M5.Lcd.printf( " I/D:%3d/%3d\n", KI,KD);
    M5.Lcd.printf( " P:%8.2f\n", myPID.GetPterm());
    M5.Lcd.printf( " I:%8.2f\n", myPID.GetIterm());
    M5.Lcd.printf( " D:%8.2f\n", myPID.GetDterm());
    M5.Lcd.println("PWM (Hz)");
    M5.Lcd.printf( " I:%6d\n", getFrequency(true));
    M5.Lcd.printf( " O:%6d\n", PWM_HERZ_);
    // QuickPID
    CH3_USEC = pulseIn(CH3_IN,HIGH,PWM_WAIT);
    myPID.SetTunings(KP/100.0,KI/500.0,KD/500.0);
    myPID.SetOutputLimits(CONF[_MIN]-CH1US_MEAN,CONF[_MAX]-CH1US_MEAN);
  }

  // (6) watch vin and buttons
  vin_watch();
  M5.update();
  if (M5.BtnA.isPressed()) config_by_wifi();
  else
  if (M5.BtnB.isPressed()) config_ch1ends();
  getFrequency();
  //LOOP = (LOOP + 1) % 50;
}

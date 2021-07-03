//////////////////////////////////////////////////
// GyroM5v2 - M5StickC project:
//   Steering assisting unit for RC drift car
// New Features from GyroM5v1:
//   Parameter setting by WiFi AP
//   Variable PWM frequency output
//   Continuous time PID controller
//   Automatic detection of vertical axis
// URL:
//   https://github.com/hshin-git/GyroM5
//////////////////////////////////////////////////
#include <M5StickC.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <QuickPID.h>
//#include <Ticker.h>


//////////////////////////////////////////////////
// Global constants
//////////////////////////////////////////////////
// WiFi parameters
const char WIFI_SSID[] = "GyroM5";
const char WIFI_PASS[] = "";
const IPAddress WIFI_IP(192,168,5,1);
const IPAddress WIFI_SUBNET(255,255,255,0);
WiFiServer WIFI_SERVER(80);

// PWM parameters
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
//
const int PWM_BITS = 16;
const int PWM_DMAX = (1<<PWM_BITS);
const int PWM_WAIT = (1000000/50) + 1000;
//
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;
const int PULSE_AMP = (PULSE_MAX-PULSE_MIN)/2;

// CH3
const int GAIN_MIN = 0;
const int GAIN_MAX = 100;
const int GAIN_AMP = 120;

// GPIO parameters
const int CH1_IN = 26;
const int CH3_IN = 36;
const int CH1_OUT = 0;

// delay after button (msec)
const int DELAY = 300;



//////////////////////////////////////////////////
// PWM output frequency
//////////////////////////////////////////////////
int PWM_HERZ = 50;
int PWM_CYCL = 1000000/PWM_HERZ;
//
void ch1_setHerz(int herz) {
  if (herz<50 || herz>500) return;
  PWM_HERZ = herz;
  PWM_CYCL = 1000000/PWM_HERZ;
  ledcSetup(PWM_CH1,PWM_HERZ,PWM_BITS);
  ledcAttachPin(CH1_OUT,PWM_CH1);
  ledcWrite(PWM_CH1,0);
}
void ch1_output(int usec) {
  int duty = map(usec, 0,PWM_CYCL, 0,PWM_DMAX);
  ledcWrite(PWM_CH1,(usec>0? duty: 0));
}


//////////////////////////////////////////////////
// LCD helpers
//////////////////////////////////////////////////
TFT_eSprite canvas = TFT_eSprite(&M5.Lcd);
const int BG_COLOR = TFT_BLACK;
const int FG_COLOR = TFT_WHITE;
//
void canvas_init(void) {
  M5.Axp.ScreenBreath(8);
  M5.Lcd.setRotation(0);
  canvas.createSprite(M5.Lcd.width(),M5.Lcd.height());
}
bool canvas_header(char *text, int msec) {
  static long lastTime = 0;
  if (lastTime + msec < millis()) {
    canvas.fillScreen(BG_COLOR);
    canvas.setCursor(0,0);
    canvas.setTextColor(BG_COLOR,FG_COLOR);
    canvas.printf(" %-12s\n",text);
    canvas.setTextColor(FG_COLOR,BG_COLOR);
    lastTime = millis();
    return true;
  }
  return false;
}
bool canvas_footer(char *text) {
  canvas.setTextColor(BG_COLOR,FG_COLOR);
  canvas.printf(" %-12s\n",text);
  canvas.setTextColor(FG_COLOR,BG_COLOR);
  canvas.pushSprite(0,0);
}


//////////////////////////////////////////////////
// Ring buffer to store and draw sampled values
//////////////////////////////////////////////////
// ring buffer
const int RING_MAX = 4;
struct {
  int IDS = 0;
  int *ARRAY[RING_MAX];
  int HEAD[RING_MAX];
  char *TEXT[RING_MAX];
  int COLOR[RING_MAX];
} RING;

// data storage
const int DATA_MSEC = 100; // sampling pediod
const int DATA_SIZE = 4096; // sampling storage size
int DATA_Setpoint[DATA_SIZE];
int DATA_Output[DATA_SIZE];
int DATA_Input[DATA_SIZE];

//
int data_init(int *buf, char *txt, int col=TFT_WHITE) {
  int id = RING.IDS;
  if (id < RING_MAX) {
    RING.ARRAY[id] = buf;
    RING.HEAD[id] = 0;
    RING.TEXT[id] = txt;
    RING.COLOR[id] = col;
    for (int i=0; i<DATA_SIZE; i++) buf[i] = 0;
    RING.IDS = id+1;
    return id;
  }
  return -1;
}
void data_put(int id, int val) {
  int N = DATA_SIZE;
  int *A = RING.ARRAY[id];
  int p = RING.HEAD[id];
  A[p] = val;
  RING.HEAD[id] = (p+1)%N;
}
void data_draw(int past, int lastLine=9, int top=80, int left=0, int width=80, int height=80) {
  int N = DATA_SIZE;
  int PAST = past<=0? N: min(N,past);
  
  for (int id=0; id<RING.IDS; id++) {
    int *A = RING.ARRAY[id];
    int p = RING.HEAD[id];
    char *txt = RING.TEXT[id];
    int color = RING.COLOR[id];
    // plot
    p = (p-PAST+N)%N;
    for (int n=0; n<PAST-1; n++) {
      int v0 = A[p];
      int v1 = A[(p+1)%N];
      int x0 = map(n, 0,PAST, 0,width);
      int x1 = map(n+1, 0,PAST, 0,width);
      int y0 = map(v0, -PULSE_AMP,PULSE_AMP, top+height,top);
      int y1 = map(v1, -PULSE_AMP,PULSE_AMP, top+height,top);
      canvas.drawLine(x0,y0,x1,y1,color);
      p = (p+1)%N;
    }
    // legend
    if (txt) {
      canvas.setCursor(8*(3*id+1),top);
      canvas.setTextColor(color);
      canvas.printf("%3s",txt);
      canvas.setTextColor(FG_COLOR);
      // reset cursor in ad-hoc manner
      canvas.setCursor(0,8*lastLine);
    }
  }
}
void data_grid(int v, int top=80, int left=0, int width=80, int height=80) {
  int y = map(v, -PULSE_AMP,PULSE_AMP, top+height,top);
  canvas.drawLine(left,y, left+width,y,FG_COLOR);
}
void data_to_csv(WiFiClient *cl) {
  int N = DATA_SIZE;
  int p = RING.HEAD[0];
  // head
  cl->print("SEC,");
  for (int id=0; id<RING.IDS; id++) {
    cl->print(RING.TEXT[id]);
    cl->print(id<RING.IDS-1? ",": "\n");
  }
  // data
  for (int n=0; n<N; n++) {
    cl->printf("%.2f,", n*DATA_MSEC/1000.0);
    for (int id=0; id<RING.IDS; id++) {
      int *A = RING.ARRAY[id];
      cl->print(A[p]);
      cl->print(id<RING.IDS-1? ",": "\n");
    }
    p = (p+1)%N;  
  }
}
bool data_sample(int msec) {
  static long lastTime = 0;
  if (lastTime + msec < millis()) {
    lastTime = millis();
    return true;
  }
  return false;
}
float data_MAE(int id1, int id2, int past=0) {
  int N = DATA_SIZE;  
  int *A1 = RING.ARRAY[id1];
  int *A2 = RING.ARRAY[id2];
  int p = RING.HEAD[id1];
  int PAST = past<=0? N: min(N,past);
  float MAE = 0.0;
  p = (p-PAST+N)%N;
  for (int n=0; n<PAST; n++) {
    MAE += abs(A1[p]-A2[p]);
    p = (p+1)%N;
  }
  return MAE/N;
}
float data_RMSE(int id1, int id2, int past=0) {
  int N = DATA_SIZE;  
  int *A1 = RING.ARRAY[id1];
  int *A2 = RING.ARRAY[id2];
  int p = RING.HEAD[id1];
  int PAST = past<=0? N: min(N,past);
  float MSE = 0.0;
  p = (p-PAST+N)%N;
  for (int n=0; n<PAST; n++) {
    MSE += (A1[p]-A2[p])*(A1[p]-A2[p]);
    p = (p+1)%N;   
  }
  return sqrt(MSE/N);
}


//////////////////////////////////////////////////
// 5Vin watcher
//////////////////////////////////////////////////
void _axp_halt(){
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
  static long lastTime = 0;
  float vin = M5.Axp.GetVinData()*1.7 /1000;
  float usb = M5.Axp.GetVusbinData()*1.7 /1000;
  //Serial.printf("vin,usb = %f,%f\n",vin,usb);
  if ( vin < 3.0 && usb < 3.0 ) {
    if ( lastTime + 5000 < millis() ) {
      _axp_halt();
    }
  } else {
    lastTime = millis();
  }
}


//////////////////////////////////////////////////
// Parameter config by WiFi
//////////////////////////////////////////////////
Preferences STORAGE;
const char CONFIG_NAME[] = "GYROM5";
const char CONFIG_KEY[] = "CONF";

// GyroM5 parameters
const char *KEYS[] = {"KG","KP","KI","KD", "CH1","CH3","PWM", "MIN","MAX", "END",};
const int _INIT_[] = {50,50,20,5, 0,0,50, 1000,2000, 12345,};
int CONFIG[] = {50,50,20,5, 0,0,50, 1000,2000, 12345,};
enum _INDEX {_KG=0,_KP,_KI,_KD, _CH1,_CH3,_PWM, _MIN,_MAX, _END,};
const int SIZE = sizeof(CONFIG)/sizeof(int);
const int TAIL = 3; // number of items after "PWM"

// storage read/write
void config_init() {
  STORAGE.begin(CONFIG_NAME);
  STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  if (CONFIG[_END] != _INIT_[_END]) { // the first time
    STORAGE.putBytes(CONFIG_KEY, &_INIT_, sizeof(CONFIG));
    STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  }
}
void config_puts() {
  STORAGE.putBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
}
void config_gets() {
  STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
}
void config_show() {
  if (canvas_header("CONF",0)) {
    for (int n=0; n<SIZE; n++) canvas.printf(" %s:%6d\n",KEYS[n],CONFIG[n]);
    canvas_footer("CONF");
  }
}


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
<input type='submit' value='upload setting' />
<input type='button' value='download data' onclick='window.location=window.location.href.split("?")[0]+"csv";' />
<input type='button' value='reload setting' onclick='window.location=window.location.href.split("?")[0];' />
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
char HTML_BUFFER[sizeof(HTML_TEMPLATE)+sizeof(WIFI_SSID)+8*SIZE];

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
            sprintf(HTML_BUFFER,HTML_TEMPLATE, WIFI_SSID,CONFIG[_KG],CONFIG[_KP],CONFIG[_KI],CONFIG[_KD],CONFIG[_CH1],CONFIG[_CH3],CONFIG[_PWM],0);
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
              p2 = currentLine.indexOf(n<(SIZE-TAIL)-1? '&': ' ', p1);
              val = currentLine.substring(p1, p2).toInt();
              CONFIG[n] = val;
            }
            config_puts();
            ch1_setHerz(CONFIG[_PWM]);
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html; charset=utf-8;");
            client.println();
            sprintf(HTML_BUFFER,HTML_TEMPLATE, WIFI_SSID,CONFIG[_KG],CONFIG[_KP],CONFIG[_KI],CONFIG[_KD],CONFIG[_CH1],CONFIG[_CH3],CONFIG[_PWM],1);
            client.println(HTML_BUFFER);
            configAccepted = true;
            break;
          } else if (currentLine.indexOf("GET /csv") == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/csv; charset=utf-8;");
            client.println("Content-Disposition:attachment; filename=data.csv");
            client.println();
            data_to_csv(&client);
            client.println();
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
  if (canvas_header("WIFI",0)) {
    char url[32];
    canvas.println("[A] CANCEL");
    canvas.println("SSID:"); canvas.printf(" %s\n",WIFI_SSID);
    canvas.println("PASS:"); canvas.printf(" %s\n",WIFI_PASS);
    canvas.println("IP:"); canvas.print(" "); canvas.println(WIFI_IP);
    canvas_footer("WIFI");
    sprintf(url,"http://%d.%d.%d.%d/",((WIFI_IP>>0)&0xff),((WIFI_IP>>8)&0xff),((WIFI_IP>>16)&0xff),((WIFI_IP>>24)&0xff));
    M5.Lcd.qrcode(url,0,80,80,2);
  }
  delay(DELAY);
  //
  configAccepted = false;
  while (!configAccepted) {
    configLoop();
    vin_watch();
    M5.update();
    if (M5.BtnA.isPressed()) {
      delay(DELAY);
      return;
    }
  }
  //
  //WIFI_SERVER.end();
}


// config for ch1 end points
void config_ch1ends() {
  int ch1,val;
  for (int n=0; n<2; n++) {
    delay(DELAY);
    while (true) {
      ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
      val = map(ch1, 0,PWM_CYCL, 0,PWM_DMAX);
      //ledcWrite(PWM_CH1,(ch1>0? val: 0));
      ch1_output(ch1);
      if (canvas_header("ENDS",500)) {
        canvas.println((n? "LEFT": "RIGHT"));
        canvas.printf("[A] SAVE\n");
        canvas.printf("[B] CANCEL\n");
        canvas.printf(" CH1:%6d\n",ch1);
        canvas.printf(" VAL:%6d\n",val);
        canvas.printf(" |%s| \n",(n? "<<<<    ": "    >>>>"));
        canvas_footer("ENDS");
      }
      M5.update();
      if (M5.BtnA.isPressed()) {
        if (ch1>0) CONFIG[(n? _MAX: _MIN)] = ch1;
        break;
      }
      else
      if (M5.BtnB.isPressed()) {
        ch1_output(0);
        delay(DELAY);
        return;
      }
    } 
  }
  // save
  if (ch1>0) {
    if (CONFIG[_MAX] < CONFIG[_MIN]) {
      int temp = CONFIG[_MAX];
      CONFIG[_MAX] = CONFIG[_MIN];
      CONFIG[_MIN] = temp;
    }
    config_puts();
  }
  ch1_output(0);
  delay(DELAY);
}


//////////////////////////////////////////////////
// Low/High Pass Filters
//////////////////////////////////////////////////
//float LPF_ME[4];
//float LPF_MAE[4];
//float LPF_MSE[4];
//// LPF
//void lpf_init(float buf[],float alpha) {
//  buf[0] = 0.;
//  buf[1] = alpha;
//}
//float lpf_update(float buf[],float x) {
//  float yp = buf[0];
//  float alpha = buf[1];
//  float y = alpha*x + (1.-alpha)*yp;
//  buf[0] = y;
//  return y;
//}
//// HPF
//void hpf_init(float buf[],float alpha) {
//  buf[0] = 0.;
//  buf[1] = 0.;
//  buf[2] = alpha;
//}
//float hpf_update(float buf[],float x) {
//  float yp = buf[0];
//  float xp = buf[1];
//  float alpha = buf[2];
//  float y = alpha*(x - xp) + alpha*yp;
//  buf[0] = y;
//  buf[1] = x;
//  return y;
//}


//////////////////////////////////////////////////
// Initial calibrators
//////////////////////////////////////////////////
float CH1US_MEAN;
float OMEGA_MEAN[3];
float ACCEL_MEAN[3];

// Frequency counter
int getFrequency(bool getOnly = false) {
  static long lastTime = 0;
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
  long startTime;
  // wait
  for (int n=0; n<10; n++) {
    if (canvas_header("WAIT", n? 500: 0)) {
      canvas.printf(" N:%8d\n",n);
      canvas_footer("WAIT");
    }
    while (pulseIn(CH1_IN,HIGH,PWM_WAIT)==0) vin_watch();
  }
  // zero
  startTime = millis();
  for (int n=0; true; n++) {
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
    if (canvas_header("INIT", n? 500: 0)) {
      canvas.println("PWM (Hz)");
      canvas.printf( " F:%8d\n",hrz);
      canvas.println("CH1 (us)");
      canvas.printf( " D:%8.2f\n",CH1US_MEAN);
      canvas.println("OMEGA (rad/s)");
      canvas.printf( " X:%8.2f\n",OMEGA_MEAN[0]);
      canvas.printf( " Y:%8.2f\n",OMEGA_MEAN[1]);
      canvas.printf( " Z:%8.2f\n",OMEGA_MEAN[2]);
      canvas.println("ACCEL (G)");
      canvas.printf( " X:%8.2f\n",ACCEL_MEAN[0]);
      canvas.printf( " Y:%8.2f\n",ACCEL_MEAN[1]);
      canvas.printf( " Z:%8.2f\n",ACCEL_MEAN[2]);
      canvas_footer("INIT");
    }
    if (startTime + 5000 < millis()) break;
  }
}

// yaw rate := (w,z)
float getYawRate(float *omega) {
  float wz0 = (omega[0]-OMEGA_MEAN[0])*ACCEL_MEAN[0];
  float wz1 = (omega[1]-OMEGA_MEAN[1])*ACCEL_MEAN[1];
  float wz2 = (omega[2]-OMEGA_MEAN[2])*ACCEL_MEAN[2];
  return (wz0 + wz1 + wz2) * M5.MPU6886.gRes;
}
// vertical accel := (a,z)
float getVerticalG(float *accel) {
  float az = accel[0]*ACCEL_MEAN[0] + accel[1]*ACCEL_MEAN[1] + accel[2]*ACCEL_MEAN[2];
  return az;
}
// horizontal accel := |a - (a,z)z|
float getHorizontalG(float *accel) {
  float az = accel[0]*ACCEL_MEAN[0] + accel[1]*ACCEL_MEAN[1] + accel[2]*ACCEL_MEAN[2];
  float h0 = accel[0] - az*ACCEL_MEAN[0];
  float h1 = accel[1] - az*ACCEL_MEAN[1];
  float h2 = accel[2] - az*ACCEL_MEAN[2];
  return sqrt(h0*h0 + h1*h1 + h2*h2);
}
// vibrational accel := |a - z|
float getVibrationalG(float *accel) {
  float v0 = accel[0] - ACCEL_MEAN[0];
  float v1 = accel[1] - ACCEL_MEAN[1];
  float v2 = accel[2] - ACCEL_MEAN[2];
  return sqrt(v0*v0 + v1*v1 + v2*v2);
}


//////////////////////////////////////////////////
// QuickPID with timer interruption
//////////////////////////////////////////////////
float Setpoint=0.0;
float Input=0.0;
float Output=0.0;
QuickPID myPID(&Input, &Output, &Setpoint, 1.0,0.0,0.0, QuickPID::DIRECT);

// PWM input values in usec
int CH1_USEC = 0;
int CH2_USEC = 0;
int CH3_USEC = 0;

// IMU input values
float IMU_OMEGA[3];
float IMU_ACCEL[3];

// PID loop
void IRAM_ATTR loopPID() {
  int ch1_usec;
  float yrate;
  float Kg = (CONFIG[_KG]/1.0);
 
  // Input IMU
  M5.MPU6886.getGyroData(&IMU_OMEGA[0],&IMU_OMEGA[1],&IMU_OMEGA[2]);
  M5.MPU6886.getAccelData(&IMU_ACCEL[0],&IMU_ACCEL[1],&IMU_ACCEL[2]);

  //
  Kg = CONFIG[_CH1]? -Kg: Kg;
  yrate = getYawRate(IMU_OMEGA);
  
  // Compute PID
  Setpoint = CH1_USEC>0? CH1_USEC - CH1US_MEAN: 0.0;
  Input = Kg * yrate;
  myPID.Compute();
  ch1_usec = constrain(CH1US_MEAN + Output, CONFIG[_MIN],CONFIG[_MAX]);
  
  // Output PWM
  ch1_output((CH1_USEC>0? ch1_usec: 0));
}

// PID timer
//Ticker tickerPID;
//hw_timer_t *timerPID = NULL;

// PID setup
void setupPID(bool init=false) {
  float Kp = (CONFIG[_KP]/50.);
  float Ki = (CONFIG[_KI]/500.);
  float Kd = (CONFIG[_KD]/5000.);
  int Min = int(CONFIG[_MIN] - CH1US_MEAN);
  int Max = int(CONFIG[_MAX] - CH1US_MEAN);

  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetOutputLimits(Min,Max);
  
  if (init) {
    //int CycleInUs = 1000000/CONFIG[_PWM];
    int CycleInUs = 1000000/getFrequency(true);
    myPID.SetSampleTimeUs(CycleInUs);
    myPID.SetMode(QuickPID::AUTOMATIC);
    //myPID.SetMode(QuickPID::TIMER);

    // PID timer is not working
    //tickerPID.attach(CycleInUs/1000000.0,loopPID);
    //timerPID = timerBegin(0, getApbFrequency()/1000000, true);
    //timerAttachInterrupt(timerPID, loopPID, true);
    //timerAlarmWrite(timerPID, CycleInUs, true);
    //timerAlarmEnable(timerPID);
  }
}


//////////////////////////////////////////////////
// put your setup code here, to run once:
//////////////////////////////////////////////////
void setup() {
  // (1) Initialize M5StickC object
  M5.begin();
  M5.MPU6886.Init();
  //while (!setCpuFrequencyMhz(80));  
  canvas_init();

  // (2) Initialize configuration
  config_init();
  config_show(); delay(2000);

  // (3) Initialize GPIO settings
  pinMode(CH1_IN,INPUT);
  pinMode(CH3_IN,INPUT);
  pinMode(CH1_OUT,OUTPUT);
  ch1_setHerz(CONFIG[_PWM]);

  // (4) WiFi AP setting
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(DELAY);
  WiFi.softAPConfig(WIFI_IP, WIFI_IP, WIFI_SUBNET);
  IPAddress myIP = WiFi.softAPIP();
  WIFI_SERVER.begin();

  // (5) Initialize ring buffer
  data_init(DATA_Setpoint,"CH1",TFT_CYAN);
  data_init(DATA_Output,"SRV",TFT_MAGENTA);
  data_init(DATA_Input,"YAW",TFT_YELLOW);

  // (6) Initialize zeros/means
  call_calibration();

  // (7) Initialize PID
  setupPID(true);

  // (8) Initialize others
  //Serial.begin(115200);
}


//////////////////////////////////////////////////
// put your main code here, to run repeatedly:
//////////////////////////////////////////////////
void loop() {
  // Fetch new Setpoint/Input and compute Output by PID
  CH1_USEC = pulseIn(CH1_IN,HIGH,PWM_WAIT);
  loopPID();
  getFrequency();
  
  // Sample PID variables in every 20msec
  if (data_sample(DATA_MSEC)){
    data_put(0,Setpoint);
    data_put(1,Output);
    data_put(2,Input);
  }

  // Monitor variables in every 500msec
  if (canvas_header("HOME",500)) {
    int ch3_gain;
    // RCV monitor
    canvas.println("RCV (us)");
    canvas.printf( " CH1:%6d\n", CH1_USEC);
    //canvas.printf( " CH2:%6d\n", CH2_USEC);
    canvas.printf( " CH3:%6d\n", CH3_USEC);
    // PWM monitor
    canvas.println("PWM (Hz)");
    canvas.printf( " IN :%6d\n", (CH1_USEC>0? getFrequency(true): 0));
    canvas.printf( " OUT:%6d\n", PWM_HERZ);
    // IMU monitor
    //canvas.println("OMEGA (rad/s)");
    //canvas.printf( " X:%8.2f\n", IMU_OMEGA[0]*M5.MPU6886.gRes);
    //canvas.printf( " Y:%8.2f\n", IMU_OMEGA[1]*M5.MPU6886.gRes);
    //canvas.printf( " Z:%8.2f\n", IMU_OMEGA[2]*M5.MPU6886.gRes);
    //canvas.println("ACCEL (G)");
    //canvas.printf( " X:%8.2f\n", IMU_ACCEL[0]);
    //canvas.printf( " Y:%8.2f\n", IMU_ACCEL[1]);
    //canvas.printf( " Z:%8.2f\n", IMU_ACCEL[2]);
    // PID monitor
    //canvas.println("PID (0-100)");
    //canvas.printf( " G/P:%3d/%3d\n", CONFIG[_KG],CONFIG[_KP]);
    //canvas.printf( " I/D:%3d/%3d\n", CONFIG[_KI],CONFIG[_KD]);
    // ERR monitor
    canvas.println("PID (us)");
    canvas.printf( " MAE:%6.1f\n", data_MAE(0,2,int(10.0*1000/DATA_MSEC)));
    //canvas.printf( "RMSE:%6.1f\n", data_RMSE(0,2,int(10.0*1000/DATA_MSEC)));
    // RGB graph
    data_grid(0);
    data_grid(CONFIG[_MIN]-CH1US_MEAN);
    data_grid(CONFIG[_MAX]-CH1US_MEAN);
    data_draw(int(10.0*1000/DATA_MSEC));
    // LCD draw
    canvas_footer("HOME");
    
    // CH3 => CONFIG
    CH3_USEC = pulseIn(CH3_IN,HIGH,PWM_WAIT);
    ch3_gain = map(CH3_USEC, PULSE_MIN,PULSE_MAX, -GAIN_AMP,GAIN_AMP);
    ch3_gain = constrain(ch3_gain, GAIN_MIN,GAIN_MAX);
    switch (CONFIG[_CH3]) {
      case 1: CONFIG[_KG] = ch3_gain; break;
      case 2: CONFIG[_KP] = ch3_gain; break;
      case 3: CONFIG[_KI] = ch3_gain; break;
      case 4: CONFIG[_KD] = ch3_gain; break;
      case 5: CONFIG[_KG] = 50; CONFIG[_KG] = CONFIG[_KI] = CONFIG[_KD] = 0; break;
      default: break;
    }
    // QuickPID <= CONFIG
    setupPID();    
  }

  // Watch vin and buttons
  vin_watch();
  M5.update();
  if (M5.BtnA.isPressed()) config_by_wifi();
  else
  if (M5.BtnB.isPressed()) config_ch1ends();
}

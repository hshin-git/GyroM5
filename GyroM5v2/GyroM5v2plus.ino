//////////////////////////////////////////////////
// GyroM5v2 - M5StickC project:
//   Steering assisting unit for RC drift car
// New Features from GyroM5v1:
//   Parameter setting by WiFi AP and WWW server
//   Variable PWM/PID frequency
//   Continuous time PID controller
//   Automatic detection of vertical direction
// URL:
//   https://github.com/hshin-git/GyroM5
//////////////////////////////////////////////////
#include <M5StickCPlus.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <Ticker.h>
#include <QuickPID.h>


//////////////////////////////////////////////////
// Global constants
//////////////////////////////////////////////////
// WiFi parameters
const char *WIFI_SSID = "GyroM5v2plus";
const char *WIFI_PASS = NULL; // (8 char or more for WPA2, NULL for open use)
const IPAddress WIFI_IP(192,168,5,1);
const IPAddress WIFI_SUBNET(255,255,255,0);
WiFiServer WIFI_SERVER(80);

// GPIO parameters
const int CH1_IN = 26;
const int CH3_IN = 36;
const int CH1_OUT = 0;  // G0 must be HIGH while booting, so shoud be output pin

// PWM channel
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;

// PWM resolution
const int PWM_BITS = 16;      // 16bit is valid up to 1220.70Hz
const int PWM_DUTY = (1<<PWM_BITS);
const int PWM_WAIT = 21*1000; // one 50Hz cycle plus 1msec

// PWM pulse specs in msec
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;
const int PULSE_AMP = (PULSE_MAX-PULSE_MIN)/2;

// CH3 remote gain
const int GAIN_MIN = 0;
const int GAIN_MAX = 100;
const int GAIN_AMP = 120;

// LCD parameters
const int LCD_BACK = 8;   // brightness 7-15
const int LCD_MSEC = 500; // reflesh cycle in msec
const int BG_COLOR = TFT_BLACK;
const int FG_COLOR = TFT_WHITE;

// GUI button delay (msec)
const int GUI_MSEC = 300;

// RTC
RTC_DateTypeDef RTC_DATE;
RTC_TimeTypeDef RTC_TIME;




//////////////////////////////////////////////////
// PWM output frequency
//////////////////////////////////////////////////
// Variable PWM frequency
int PWM_FREQ = 50;
int PWM_USEC = 1000000/PWM_FREQ;
//
void ch1_setFreq(int freq) {
  static bool firstTime = true;
  if (freq<50 || freq>400) return;
  PWM_FREQ = freq;
  PWM_USEC = 1000000/PWM_FREQ;
  //
  if (!firstTime) ledcDetachPin(CH1_OUT);
  ledcSetup(PWM_CH1,PWM_FREQ,PWM_BITS);
  ledcWrite(PWM_CH1,0);
  ledcAttachPin(CH1_OUT,PWM_CH1);
  firstTime = false;
}
void ch1_setUsec(int usec) {
  int duty = map(usec, 0,PWM_USEC, 0,PWM_DUTY);
  ledcWrite(PWM_CH1,(usec>0? duty: 0));
}



//////////////////////////////////////////////////
// PWM reading without blocking
//////////////////////////////////////////////////
// PWM watch dog timer
Ticker PWMIN_WDT;
//
const int PWMIN_MAX = 4;
int PWMIN_IDS = 0;
typedef struct {
  int pin;
  int tout;
  // for pulse
  int *dst;
  int prev;
  unsigned long last;
  // for freq
  int *dstFreq;
  unsigned long lastFreq;
} _PWMIN;
_PWMIN PWMIN[PWMIN_MAX];
// PWM interrupt handler
void _pwmin_isr(void *arg) {
  unsigned long tnow = micros();
  int id = (int)arg;
  _PWMIN *pwm = &PWMIN[id];
  int vnow = digitalRead(pwm->pin);
  if (pwm->prev==0 && vnow==1) {
    // at up edge
    pwm->prev = 1;
    pwm->last = tnow;
    // for freq
    *(pwm->dstFreq) = (tnow > pwm->lastFreq? 1000000/(tnow - pwm->lastFreq): 0);
    pwm->lastFreq = tnow;
  }
  else
  if (pwm->prev==1 && vnow==0) {
    // at down edge
    *(pwm->dst) = tnow - pwm->last;
    pwm->prev = 0;
    pwm->last = tnow;
  }
}
// PWM timer handler
void _pwmin_tsr(void) {
  unsigned long tnow = micros();
  for (int id=0; id<PWMIN_IDS; id++) {
    _PWMIN *pwm = &PWMIN[id];
    if (pwm->last + pwm->tout < tnow) {
      *(pwm->dst) = 0;
      *(pwm->dstFreq) = 0;
    }
  }
}
//
bool pwmin_init(int pin, int *usec, int *freq, int toutUs=21*1000) {
  if (PWMIN_IDS < PWMIN_MAX) {
    int id = PWMIN_IDS;
    _PWMIN *pwm = &PWMIN[id];
    //
    pwm->pin = pin;
    pwm->tout = toutUs;
    // for pulse
    pwm->dst = usec;
    pwm->prev = 0;
    pwm->last = micros();
    // for freq
    pwm->dstFreq = freq;
    pwm->lastFreq = micros();
    //
    pinMode(pin,INPUT);
    attachInterruptArg(pin,_pwmin_isr,(void*)id,CHANGE);
    if (id==0) PWMIN_WDT.attach_ms(pwm->tout/1000,_pwmin_tsr);
    //
    PWMIN_IDS = id + 1;
    return true;
  }
  return false;
}
//
bool gpio25_dis_init(){
  gpio_pulldown_dis(GPIO_NUM_25);
  gpio_pullup_dis(GPIO_NUM_25);
}
//
void pwmin_disable(void) {
  if (PWMIN_IDS <= 0) return;
  PWMIN_WDT.detach();
  for (int id=0; id<PWMIN_IDS; id++) {
    _PWMIN *pwm = &PWMIN[id];
    detachInterrupt(pwm->pin);
  }
  delay(GUI_MSEC);
}
void pwmin_enable(void) {
  if (PWMIN_IDS <= 0) return;
  for (int id=0; id<PWMIN_IDS; id++) {
    _PWMIN *pwm = &PWMIN[id];
    attachInterruptArg(pwm->pin,_pwmin_isr,(void*)id,CHANGE);
    if (id==0) PWMIN_WDT.attach_ms(pwm->tout/1000,_pwmin_tsr);
  }
}


//////////////////////////////////////////////////
// LCD helpers
//////////////////////////////////////////////////
// Double bufferd LCD
TFT_eSprite canvas = TFT_eSprite(&M5.Lcd);
//
void canvas_init(void) {
  M5.Axp.ScreenBreath(LCD_BACK);
  M5.Lcd.setRotation(0);
  canvas.createSprite(M5.Lcd.width(),M5.Lcd.height());
}
bool canvas_header(char *text, int msec) {
  static unsigned long lastTime = 0;
  if (lastTime + msec < millis()) {
    M5.Rtc.GetTime(&RTC_TIME);
    canvas.fillScreen(BG_COLOR);
    canvas.setCursor(0,0);
    canvas.setTextColor(BG_COLOR,FG_COLOR);
    //canvas.printf(" %-12s\n",text);
    canvas.printf(" %-6s %02d:%02d\n",text,RTC_TIME.Hours,RTC_TIME.Minutes);
    canvas.setTextColor(FG_COLOR,BG_COLOR);
    lastTime = millis();
    return true;
  }
  return false;
}
void canvas_footer(char *text) {
  M5.Rtc.GetData(&RTC_DATE);
  canvas.setTextColor(BG_COLOR,FG_COLOR);
  //canvas.printf(" %-12s\n",text);
  canvas.printf(" %-6s %02d/%02d\n",text,RTC_DATE.Month,RTC_DATE.Date);
  canvas.setTextColor(FG_COLOR,BG_COLOR);
  canvas.pushSprite(0,0);
}



//////////////////////////////////////////////////
// Ring buffer to store and draw sampled values
//////////////////////////////////////////////////
// Ring buffer
const int RING_MAX = 4;
int RING_IDS = 0;
typedef struct {
  int *buff;
  int head;
  char *text;
  int color;
} _RING;
_RING RING[RING_MAX];

// data storage
const int DATA_MSEC = 100; // sampling cycle in msec
const int DATA_SIZE = 8*60*(1000/DATA_MSEC); // sampling storage size
//
int DATA_Setpoint[DATA_SIZE];
int DATA_Output[DATA_SIZE];
int DATA_Input[DATA_SIZE];

// data operations
int data_init(int *buf, char *txt, int col=TFT_WHITE) {
  int id = RING_IDS;
  if (id < RING_MAX) {
    RING[id].buff = buf;
    RING[id].head = 0;
    RING[id].text = txt;
    RING[id].color = col;
    for (int i=0; i<DATA_SIZE; i++) buf[i] = 0;
    RING_IDS = id+1;
    return id;
  }
  return -1;
}
void data_put(int id, int val) {
  int N = DATA_SIZE;
  int *A = RING[id].buff;
  int p = RING[id].head;
  A[p] = val;
  RING[id].head = (p+1)%N;
}
void data_draw(int lastData, int lastLine=8, int top=120, int left=0, int width=120, int height=120) {
  int N = DATA_SIZE;
  int LAST = (lastData>0? min(N,lastData): N);
    
  for (int id=0; id<RING_IDS; id++) {
    int *A = RING[id].buff;
    int p = RING[id].head;
    char *txt = RING[id].text;
    int color = RING[id].color;
    // plot
    p = (p-LAST+N)%N;
    for (int n=0; n<LAST-1; n++) {
      int v0 = A[p];
      int v1 = A[(p+1)%N];
      int x0 = map(n, 0,LAST, 0,width);
      int x1 = map(n+1, 0,LAST, 0,width);
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
void data_grid(int v, int top=120, int left=0, int width=120, int height=120) {
  int y = map(v, -PULSE_AMP,PULSE_AMP, top+height,top);
  canvas.drawLine(left,y, left+width,y,FG_COLOR);
}
void data_dump(WiFiClient *cl) {
  int N = DATA_SIZE;
  int p = RING[0].head;
  // head
  cl->print("SEC,");
  for (int id=0; id<RING_IDS; id++) {
    cl->print(RING[id].text);
    cl->print(id<RING_IDS-1? ",": "\n");
  }
  // data
  for (int n=0; n<N; n++) {
    cl->printf("%.3f,", n*DATA_MSEC/1000.0);
    for (int id=0; id<RING_IDS; id++) {
      int *A = RING[id].buff;
      cl->print(A[p]);
      cl->print(id<RING_IDS-1? ",": "\n");
    }
    p = (p+1)%N;  
  }
}
bool data_sample(int msec) {
  static unsigned long lastTime = 0;
  if (lastTime + msec < millis()) {
    lastTime = millis();
    return true;
  }
  return false;
}
float data_MAE(int id1, int id2, int lastData=0) {
  int N = DATA_SIZE;
  int *A1 = RING[id1].buff;
  int *A2 = RING[id2].buff;
  int p = RING[id1].head;
  int LAST = (lastData>0? min(N,lastData): N);
  float MAE = 0.0;
  p = (p-LAST+N)%N;
  for (int n=0; n<LAST; n++) {
    MAE += abs(A1[p]-A2[p]);
    p = (p+1)%N;
  }
  return MAE/LAST;
}
float data_RMSE(int id1, int id2, int lastData=0) {
  int N = DATA_SIZE;
  int *A1 = RING[id1].buff;
  int *A2 = RING[id2].buff;
  int p = RING[id1].head;
  int LAST = (lastData>0? min(N,lastData): N);
  float MSE = 0.0;
  p = (p-LAST+N)%N;
  for (int n=0; n<LAST; n++) {
    MSE += (A1[p]-A2[p])*(A1[p]-A2[p]);
    p = (p+1)%N;   
  }
  return sqrt(MSE/LAST);
}



//////////////////////////////////////////////////
// Watch 5Vin for interlocking with RC units
//////////////////////////////////////////////////

void vin_watch() {
  static unsigned long lastTime = 0;
  float vin = M5.Axp.GetVinData()*1.7 /1000;
  float usb = M5.Axp.GetVusbinData()*1.7 /1000;
  //Serial.printf("vin,usb = %f,%f\n",vin,usb);
  if ( vin < 3.0 && usb < 3.0 ) {
    if ( lastTime + 5*1000 < millis() ) {
      M5.Axp.PowerOff();
    }
  } else {
    lastTime = millis();
  }
}



//////////////////////////////////////////////////
// GyroM5 storage for setting
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
  pwmin_disable();
  //
  STORAGE.begin(CONFIG_NAME);
  STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  if (CONFIG[_END] != _INIT_[_END]) { // the first time
    STORAGE.putBytes(CONFIG_KEY, &_INIT_, sizeof(CONFIG));
    STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  }
  //
  pwmin_enable();
}
void config_puts() {
  // timer/interrupt must be disabled during writing Preferences, otherwise ESP32 crashes...
  pwmin_disable();
  //
  STORAGE.putBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  //
  pwmin_enable();
}
void config_gets() {
  pwmin_disable();
  //
  STORAGE.getBytes(CONFIG_KEY, &CONFIG, sizeof(CONFIG));
  //
  pwmin_enable();
}
void config_dump(WiFiClient *cl) {
  // head
  for (int n=0; n<SIZE; n++) {
    cl->print(KEYS[n]);
    cl->print(n<SIZE-1? ",": "\n");
  }
  // data
  for (int n=0; n<SIZE; n++) {
    cl->print(CONFIG[n]);
    cl->print(n<SIZE-1? ",": "\n");
  }
}


//////////////////////////////////////////////////
// Parameter config by WiFi
//////////////////////////////////////////////////
void wifi_init(void) {
  pwmin_disable();
  //
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(GUI_MSEC);
  WiFi.softAPConfig(WIFI_IP, WIFI_IP, WIFI_SUBNET);
  WiFi.begin();
  //IPAddress myIP = WiFi.softAPIP();
  WIFI_SERVER.begin();
  //
  pwmin_enable();
}
//
void wifi_quit(void) {
  pwmin_disable();
  //
  WIFI_SERVER.end();
  //
  pwmin_enable();
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
<input type='hidden' name='JST' value='20001020103030' />
<input type='submit' value='upload setting' onclick='onSubmit()' />
<input type='button' value='download data' onclick='window.location=window.location.href.split("?")[0]+"csv";' />
<input type='button' value='reload setting' onclick='window.location=window.location.href.split("?")[0];' />
</form>
</body>
<script>
function onInput(obj) { document.getElementById(obj.name).textContent = obj.value; }
function onLoad() {
 const CONFIG = {KG:%d,KP:%d,KI:%d,KD:%d,CH1:%d,CH3:%d,PWM:%d,};
 const INPUTS = document.getElementsByTagName('input');
 for (let key in CONFIG){ document.getElementsByName(key)[0].value = document.getElementById(key).textContent  = CONFIG[key]; } 
 if (%d) { for (var i=0;i<INPUTS.length-1; i++) { INPUTS[i].disabled = true; } }
}
function D2(n) { return ('0'+n).slice(-2); }
function onSubmit() {
 const now = new Date();
 const str = now.getFullYear() + D2(now.getMonth()+1) + D2(now.getDate()) + D2(now.getHours()) + D2(now.getMinutes()) + D2(now.getSeconds());
 document.getElementsByName('JST')[0].value = str;
}
window.onload = onLoad();
</script>
</html>
)";

// HTML buffer
char HTML_BUFFER[sizeof(HTML_TEMPLATE)+sizeof(WIFI_SSID)+8*SIZE];

// foward prototype
void gpid_init(bool);

// Web server for config
bool configAccepted = false;
void serverLoop() {
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
            // response for request "/"
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html; charset=utf-8;");
            client.println("");
            sprintf(HTML_BUFFER,HTML_TEMPLATE, WIFI_SSID,CONFIG[_KG],CONFIG[_KP],CONFIG[_KI],CONFIG[_KD],CONFIG[_CH1],CONFIG[_CH3],CONFIG[_PWM],0);
            client.println(HTML_BUFFER);
            break;
          } 
          else
          if (currentLine.indexOf("GET /?") == 0) {
            // response for request "/?KG=..."
            int p1 = 0;
            int p2 = 0;
            int val = 0;
            // set CONFIG
            for (int n=0; n<(SIZE-TAIL); n++) {
              char key[16];
              sprintf(key,"%s=",KEYS[n]);
              p1 = currentLine.indexOf(key, p2) + strlen(key);
              p2 = currentLine.indexOf('&', p1);
              val = currentLine.substring(p1, p2).toInt();
              CONFIG[n] = val;
            }
            // set RTC
            p1 = currentLine.indexOf("JST=",p2) + strlen("JST=");
            RTC_DATE.Year = currentLine.substring(p1+0,p1+4).toInt();
            RTC_DATE.Month = currentLine.substring(p1+4,p1+6).toInt();
            RTC_DATE.Date = currentLine.substring(p1+6,p1+8).toInt();
            M5.Rtc.SetData(&RTC_DATE);
            RTC_TIME.Hours = currentLine.substring(p1+8,p1+10).toInt();
            RTC_TIME.Minutes = currentLine.substring(p1+10,p1+12).toInt();
            RTC_TIME.Seconds = currentLine.substring(p1+12,p1+14).toInt();
            M5.Rtc.SetTime(&RTC_TIME);
            // save CONFIG
            config_puts();
            ch1_setFreq(CONFIG[_PWM]);
            gpid_init(true);
            // response
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html; charset=utf-8;");
            client.println("");
            sprintf(HTML_BUFFER,HTML_TEMPLATE, WIFI_SSID,CONFIG[_KG],CONFIG[_KP],CONFIG[_KI],CONFIG[_KD],CONFIG[_CH1],CONFIG[_CH3],CONFIG[_PWM],1);
            client.println(HTML_BUFFER);
            configAccepted = true;
            break;
          } 
          else
          if (currentLine.indexOf("GET /csv") == 0) {
            // response for request "/csv"
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/csv; charset=utf-8;");
            //client.println("Content-Disposition:attachment; filename=data.csv");
            client.printf("Content-Disposition:attachment; filename=data-%04d%02d%02d-%02d%02d.csv\n", RTC_DATE.Year,RTC_DATE.Month,RTC_DATE.Date, RTC_TIME.Hours,RTC_TIME.Minutes);
            client.println("");
            config_dump(&client);
            data_dump(&client);
            client.println("");
            break;
          } 
          else 
          {
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

void setup_by_wifi() {
  //
  //WIFI_SERVER.begin();
  //
  if (canvas_header("WIFI",0)) {
    char url[32];
    canvas.println("[A] HOME");
    canvas.println("SSID:"); canvas.printf(" %s\n",WIFI_SSID);
    canvas.println("PASS:"); canvas.printf(" %s\n",(WIFI_PASS==NULL? "": WIFI_PASS));
    canvas.println("IP:"); canvas.print(" "); canvas.println(WIFI_IP);
    canvas_footer("WIFI");
    sprintf(url,"http://%d.%d.%d.%d/",((WIFI_IP>>0)&0xff),((WIFI_IP>>8)&0xff),((WIFI_IP>>16)&0xff),((WIFI_IP>>24)&0xff));
    M5.Lcd.qrcode(url,0,120,120,2);
  }
  delay(GUI_MSEC);
  //
  configAccepted = false;
  while (!configAccepted) {
    serverLoop();
    vin_watch();
    M5.update();
    if (M5.BtnA.isPressed()) {
      delay(GUI_MSEC);
      return;
    }
    //delay(2);
  }
  //
  //WIFI_SERVER.end();
  //
}


// config for ch1 end points
void setup_ch1ends() {
  int ch1,val;
  for (int n=0; n<2; n++) {
    delay(GUI_MSEC);
    while (true) {
      ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
      val = map(ch1, 0,PWM_USEC, 0,PWM_DUTY);
      //ledcWrite(PWM_CH1,(ch1>0? val: 0));
      ch1_setUsec(ch1);
      if (canvas_header("ENDS",LCD_MSEC)) {
        canvas.println((n? "LEFT": "RIGHT"));
        canvas.printf("[A] SAVE\n");
        canvas.printf("[B] HOME\n");
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
        //ch1_setUsec(0);
        delay(GUI_MSEC);
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
  //ch1_setUsec(0);
  delay(GUI_MSEC);
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
int countHz(bool getOnly = false) {
  static unsigned long lastTime = 0;
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

void mean_init(void) {
  unsigned long startTime;
  int count;
  // wait
  while (pulseIn(CH1_IN,HIGH,PWM_WAIT)==0) {
    vin_watch();
    if (canvas_header("WAIT", LCD_MSEC)) {
      for (int n=0; n<SIZE-1; n++) canvas.printf(" %s:%6d\n",KEYS[n],CONFIG[n]);
      canvas_footer("WAIT");
    }
  }
  // zero
  CH1US_MEAN = 0.0;
  for (int i=0; i<3; i++) OMEGA_MEAN[i] = ACCEL_MEAN[i] = 0.0;
  // mean
  startTime = millis();
  count = 0; 
  for (int n=0; true; n++) {
    float omega[3],accel[3];
    int ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
    int fHz = countHz();
    M5.IMU.getGyroData(&omega[0],&omega[1],&omega[2]);
    M5.IMU.getAccelData(&accel[0],&accel[1],&accel[2]);
    //
    CH1US_MEAN += ch1;
    for (int i=0; i<3; i++) {
      OMEGA_MEAN[i] += omega[i];
      ACCEL_MEAN[i] += accel[i];
    }
    count = count + 1;
    //
    if (canvas_header("INIT", LCD_MSEC)) {
      canvas.println("CH1");
      canvas.printf( " HZ:%7d\n",fHz);
      canvas.printf( " US:%7.2f\n",CH1US_MEAN/count);
      canvas.println("OMEGA (o/s)");
      canvas.printf( " X:%8.2f\n",OMEGA_MEAN[0]/count);
      canvas.printf( " Y:%8.2f\n",OMEGA_MEAN[1]/count);
      canvas.printf( " Z:%8.2f\n",OMEGA_MEAN[2]/count);
      canvas.println("ACCEL (G)");
      canvas.printf( " X:%8.2f\n",ACCEL_MEAN[0]/count);
      canvas.printf( " Y:%8.2f\n",ACCEL_MEAN[1]/count);
      canvas.printf( " Z:%8.2f\n",ACCEL_MEAN[2]/count);
      canvas_footer("INIT");
    }
    if (startTime + 5*1000 < millis()) break;
  }
  //
  CH1US_MEAN = CH1US_MEAN/count;
  for (int i=0; i<3; i++) {
    OMEGA_MEAN[i] = OMEGA_MEAN[i]/count;
    ACCEL_MEAN[i] = ACCEL_MEAN[i]/count;
  }
}

// yaw rate := (w,z)
float getYawRate(float *omega) {
  float wz0 = (omega[0]-OMEGA_MEAN[0])*ACCEL_MEAN[0];
  float wz1 = (omega[1]-OMEGA_MEAN[1])*ACCEL_MEAN[1];
  float wz2 = (omega[2]-OMEGA_MEAN[2])*ACCEL_MEAN[2];
  return (wz0 + wz1 + wz2);
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
float Setpoint = 0.0;
float Input = 0.0;
float Output = 0.0;
QuickPID GyroPID(&Input, &Output, &Setpoint, 1.0,0.0,0.0, QuickPID::DIRECT);

// PWM input values in usec
int CH1_USEC = 0;
int CH2_USEC = 0;
int CH3_USEC = 0;
//
int CH1_FREQ = 0;
int CH2_FREQ = 0;
int CH3_FREQ = 0;

// IMU input values
float IMU_OMEGA[3];
float IMU_ACCEL[3];

// PID timer
//Ticker tickerPID;
//hw_timer_t *timerPID = NULL;

// PID setup
void gpid_init(bool resetPID=false) {
  float Kp = (CONFIG[_KP]/50.);
  float Ki = (CONFIG[_KI]/250.);
  float Kd = (CONFIG[_KD]/5000.);
  int Min = int(CONFIG[_MIN] - CH1US_MEAN);
  int Max = int(CONFIG[_MAX] - CH1US_MEAN);

  GyroPID.SetTunings(Kp,Ki,Kd);
  GyroPID.SetOutputLimits(Min,Max);
  
  if (resetPID) {
    int CycleInUs = 1000000/CONFIG[_PWM];
    //int CycleInUs = 1000000/countHz(true);
    GyroPID.SetSampleTimeUs(CycleInUs);
    GyroPID.SetMode(QuickPID::TIMER);
    //GyroPID.SetMode(QuickPID::AUTOMATIC);

    // PID timer is not working
    //tickerPID.attach(CycleInUs/1000000.0,gpid_update);
    //timerPID = timerBegin(0, getApbFrequency()/1000000, true);
    //timerAttachInterrupt(timerPID, gpid_update, true);
    //timerAlarmWrite(timerPID, CycleInUs, true);
    //timerAlarmEnable(timerPID);
  }
}

// PID loop
void gpid_update() {
  int ch1_usec;
  float yrate;
  float Kg = (CONFIG[_KG]/20.0);
 
  // Input IMU
  M5.IMU.getGyroData(&IMU_OMEGA[0],&IMU_OMEGA[1],&IMU_OMEGA[2]);
  M5.IMU.getAccelData(&IMU_ACCEL[0],&IMU_ACCEL[1],&IMU_ACCEL[2]);

  //
  Kg = CONFIG[_CH1]? -Kg: Kg;
  yrate = getYawRate(IMU_OMEGA);
  
  // Compute PID
  Setpoint = CH1_USEC>0? CH1_USEC - CH1US_MEAN: 0.0;
  Input = Kg * yrate;
  GyroPID.Compute();
  ch1_usec = constrain(CH1US_MEAN + Output, CONFIG[_MIN],CONFIG[_MAX]);
  
  // Output PWM
  ch1_setUsec((CH1_USEC>0? ch1_usec: 0));
}
//
bool gpid_timing(int usec) {
  static unsigned long lastTime = 0;
  if (lastTime + usec < micros()) {
    lastTime = micros();
    return true;
  }
  return false;
}



//////////////////////////////////////////////////
// put your setup code here, to run once:
//////////////////////////////////////////////////
void setup() {
  // (1) setup M5StickC object
  M5.begin();
  M5.IMU.Init();
  //while (!setCpuFrequencyMhz(80));  

  // (2) setup configuration
  canvas_init();
  config_init();

  // (3) setup WiFi AP
  wifi_init();

  // (4) setup GPIO
  pinMode(CH1_IN,INPUT);
  pinMode(CH3_IN,INPUT);
  pinMode(CH1_OUT,OUTPUT);
  pwmin_init(CH1_IN,&CH1_USEC,&CH1_FREQ,PWM_WAIT);
  pwmin_init(CH3_IN,&CH3_USEC,&CH3_FREQ,PWM_WAIT);
  ch1_setFreq(CONFIG[_PWM]);
  gpio25_dis_init();
  
  // (5) Initialize Ring buffer
  data_init(DATA_Setpoint,"CH1",TFT_CYAN);
  data_init(DATA_Output,"SRV",TFT_MAGENTA);
  data_init(DATA_Input,"YAW",TFT_YELLOW);

  // (6) setup Zeros/Means
  mean_init();

  // (7) setup PID
  gpid_init(true);

  // (8) setup others
  //Serial.begin(115200);
}



//////////////////////////////////////////////////
// put your main code here, to run repeatedly:
//////////////////////////////////////////////////
void loop() {
  // Fetch Setpoint/Input and compute Output by PID
  //CH1_USEC = pulseIn(CH1_IN,HIGH,PWM_WAIT);
  if (gpid_timing(PWM_USEC)) {
    gpid_update();
    countHz();
  }
  
  // Sample PID variables in every 100msec
  if (data_sample(DATA_MSEC)){
    data_put(0,Setpoint);
    data_put(1,Output);
    data_put(2,Input);
  }

  // Monitor variables in every 500msec
  if (canvas_header("HOME",LCD_MSEC)) {
    int lastData = 8*1000/DATA_MSEC;
    int lastLine = 1;
    int ch3_gain;    
    // RCV monitor
    //canvas.println("RCV (us)"); lastLine++;
    //canvas.printf( " CH1:%6d\n", CH1_USEC); lastLine++;
    //canvas.printf( " CH2:%6d\n", CH2_USEC); lastLine++;
    //canvas.printf( " CH3:%6d\n", CH3_USEC); lastLine++;
    // PWM monitor
    canvas.println("PWM (Hz)"); lastLine++;
    canvas.printf( " IN :%6d\n", CH1_FREQ); lastLine++;
    canvas.printf( " OUT:%6d\n", PWM_FREQ); lastLine++;
    canvas.printf( " PID:%6d\n", countHz(true)); lastLine++;
    // IMU monitor
    //canvas.println("OMEGA (rad/s)"); lastLine++;
    //canvas.printf( " X:%8.2f\n", IMU_OMEGA[0]); lastLine++;
    //canvas.printf( " Y:%8.2f\n", IMU_OMEGA[1]); lastLine++;
    //canvas.printf( " Z:%8.2f\n", IMU_OMEGA[2]); lastLine++;
    //canvas.println("ACCEL (G)"); lastLine++;
    //canvas.printf( " X:%8.2f\n", IMU_ACCEL[0]); lastLine++;
    //canvas.printf( " Y:%8.2f\n", IMU_ACCEL[1]); lastLine++;
    //canvas.printf( " Z:%8.2f\n", IMU_ACCEL[2]); lastLine++;
    // PID monitor
    canvas.println("PID (0-100)"); lastLine++;
    canvas.printf( " G/P:%3d/%3d\n", CONFIG[_KG],CONFIG[_KP]); lastLine++;
    canvas.printf( " I/D:%3d/%3d\n", CONFIG[_KI],CONFIG[_KD]); lastLine++;
    //canvas.printf( " MAE:%6.1f\n", data_MAE(0,2,lastData)); lastLine++;
    //canvas.printf( "RMSE:%6.1f\n", data_RMSE(0,2,lastData)); lastLine++;
    // RGB graph
    data_grid(0);
    data_grid(CONFIG[_MIN]-CH1US_MEAN);
    data_grid(CONFIG[_MAX]-CH1US_MEAN);
    data_draw(lastData,lastLine);
    // LCD draw
    canvas_footer("HOME");
    
    // CH3 >> CONFIG
    //CH3_USEC = pulseIn(CH3_IN,HIGH,PWM_WAIT);
    if (CH3_USEC > 0) {
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
    }
    // CONFIG >> QuickPID
    gpid_init();    
  }

  // Watch vin and buttons
  vin_watch();
  M5.update();
  if (M5.BtnA.isPressed()) setup_by_wifi();
  else
  if (M5.BtnB.isPressed()) setup_ch1ends();
}

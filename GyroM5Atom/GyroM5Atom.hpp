////////////////////////////////////////////////////////////////////////////////
// GyroM5Atomシステムのライブラリ
// GyroM5Atom system library
// https://github.com/hshin-git/GyroM5Atom
////////////////////////////////////////////////////////////////////////////////

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#define DEBUG Serial


////////////////////////////////////////////////////////////////////////////////
// class CONFIG{}: 設定パラメータの管理クラス
//  init(): パラメータの初期値
//  load(): パラメータの復元
//  save(): パラメータの保存
//  setup(): パラメータの初期化
//  getJSON(): パラメータのJSON文字列
//  setCONF(): パラメータの更新
////////////////////////////////////////////////////////////////////////////////
Preferences CONFIG_PREF;

class CONFIG {
  //
  static const char JSON[];
  #define CONFIG_NAME   "GyroM5Atom"
  #define CONFIG_KEY    "GyroM5Atom"
  #define CONFIG_MAGIC  12345
  //
public:
  // setting parameters
  int KG;
  int KP;
  int KI;
  int KD;
  int REV;
  int MIN;
  int MAX;
  int MEAN;
  int FREQ;
  int AXIS;
  int MAGIC;
  //
  void init() {
    KG = 50;
    KP = 50;
    KI = 10;
    KD = 5;
    REV = 1;
    MIN = 1000;
    MAX = 2000;
    MEAN = 1500;
    FREQ = 50;
    AXIS = 1;
    MAGIC = CONFIG_MAGIC;
  }
  void load() {
    if (CONFIG_PREF.begin(CONFIG_NAME)) {
      CONFIG_PREF.getBytes(CONFIG_KEY,(uint8_t*)this,sizeof(*this));
      CONFIG_PREF.end();
    }
  }
  void save() {
    if (CONFIG_PREF.begin(CONFIG_NAME)) {
      CONFIG_PREF.putBytes(CONFIG_KEY,(uint8_t*)this,sizeof(*this));
      CONFIG_PREF.end();
    }
  }
  void setup() {
    load();
    if (MAGIC != CONFIG_MAGIC) {
      init(); save();
    }
  }
  char *getJSON() {
    static char json[1024];
    sprintf(json, JSON, KG,KP,KI,KD,REV,MIN,MAX,MEAN,FREQ,AXIS);
    DEBUG.println(json);
    return json;
  }
  void setCONF(const char *key, int val) {
    DEBUG.print(key); DEBUG.print("="); DEBUG.println(val);
         if (strcmp(key,"KG")==0) KG = val;
    else if (strcmp(key,"KP")==0) KP = val;
    else if (strcmp(key,"KI")==0) KI = val;
    else if (strcmp(key,"KD")==0) KD = val;
    else if (strcmp(key,"REV")==0) REV = val;
    else if (strcmp(key,"MIN")==0) MIN = val;
    else if (strcmp(key,"MAX")==0) MAX = val;
    else if (strcmp(key,"MEAN")==0) MEAN = val;
    else if (strcmp(key,"FREQ")==0) FREQ = val;
    else if (strcmp(key,"AXIS")==0) AXIS = val;
  }
  //
};
const char CONFIG::JSON[] = R"(
{
'KG':[0,100,1,%d,'%%',0],
'KP':[0,100,1,%d,'%%',0],
'KI':[0,100,1,%d,'%%',0],
'KD':[0,100,1,%d,'%%',0],
'REV':[0,1,1,%d,'bool',0],
'MIN':[1000,2000,1,%d,'usec',1],
'MAX':[1000,2000,1,%d,'usec',1],
'MEAN':[1000,2000,1,%d,'usec',1],
'FREQ':[50,400,50,%d,'Hz',0],
'AXIS':[1,6,1,%d,'1-6',0],
'CH1_FREQ':[0,400,1,50,'Hz',2],
'CH1_USEC':[1000,2000,1,1500,'usec',2],
'IMU_PITCH':[-90,90,1,0,'deg',2],
'IMU_ROLL':[-90,90,1,0,'deg',2],
'IMU_RATE':[-360,360,1,0,'deg/sec',2],
'PID_FREQ':[0,500,1,50,'Hz',2],
'PID_USEC':[1000,2000,1,1500,'usc',2],
}
)";



////////////////////////////////////////////////////////////////////////////////
// class SERVER{}: WiFi/WWWサーバの管理クラス
//  setup(): サーバの初期化
//  start(): サーバの起動
//  loop(): サーバの処理
//  stop(): サーバの停止
//  isWake(): サーバの起動有無
//  lookFloat(): Ajax監視対象の登録
////////////////////////////////////////////////////////////////////////////////
class SERVER {
  //
  static const char _SSID_[];
  static WebServer server;
  static bool serverInit;
  static bool serverWake;
  //
  static const char HTML_INIT[];
  static const char HTML_SAVE[];
  static char CHAR_BUFF[];
  //
  #define LOOK_MAX  8
  static int LOOK_INDEX;
  static char* LOOK_KEY[];
  static float* LOOK_PTR[];
  //
  static void handleRoot() { 
    sprintf(CHAR_BUFF, HTML_INIT, CONF.getJSON());
    server.send(200, "text/html", CHAR_BUFF); 
    //DEBUG.println(CHAR_BUFF);
  }
  static void handleSave() {
    for (int n = 0; n < server.args(); n++) CONF.setCONF(server.argName(n).c_str(),server.arg(n).toInt());
    CONF.save();
    sprintf(CHAR_BUFF, HTML_SAVE, CONF.getJSON());
    server.send(200, "text/html", CHAR_BUFF);
    delay(500); stop();
    //DEBUG.println(CHAR_BUFF);
  }
  static void handleStop() {
    for (int n = 0; n < server.args(); n++) CONF.setCONF(server.argName(n).c_str(),server.arg(n).toInt());
    CONF.save();
    sprintf(CHAR_BUFF, HTML_SAVE, CONF.getJSON());
    server.send(200, "text/html", CHAR_BUFF);
    delay(500); stop();
    //DEBUG.println(CHAR_BUFF);
  }
  static void handleJson() {
    char* p = CHAR_BUFF;
    p = p + sprintf(p, "{");
    for (int n = 0; n < LOOK_INDEX; n++) {
      p = p + sprintf(p,"\"%s\":%.f,", LOOK_KEY[n],*LOOK_PTR[n]);
    }
    sprintf(--p, "}");
    server.send(200, "application/json", CHAR_BUFF);
    //DEBUG.println(CHAR_BUFF);
  }
  static void handleNotFound() {
    server.send(404, "text/plain", "Not Found.");
  }
  //
public:
  //
  static CONFIG CONF;
  //
  static void setup(void) {
    CONF.setup();
  }
  //
  static void start(void) {
  
    DEBUG.println("Setup WIFI AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(_SSID_);
    WiFi.begin();
    delay(500);
    
    DEBUG.print("Started! IP address: ");
    DEBUG.println(WiFi.softAPIP());
  
    if (MDNS.begin(_SSID_)) {
      MDNS.addService("http", "tcp", 80);
      DEBUG.println("MDNS responder started");
      DEBUG.print("You can now connect to http://");
      DEBUG.print(_SSID_);
      DEBUG.println(".local");
    }
  
    if (!serverInit) {
      server.on("/", HTTP_GET, handleRoot);
      server.on("/json", HTTP_GET, handleJson);
      server.on("/save", HTTP_GET, handleSave);
      server.on("/stop", HTTP_GET, handleStop);
      server.onNotFound(handleNotFound);
      server.begin();
      DEBUG.println("HTTP server started");
      serverInit = true;
    }
  
    serverWake = true;
  }
  static void loop(void) {
    if (serverWake) server.handleClient();
  }
  static void stop(void) {
    if (serverWake) {
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      serverWake = false;
      DEBUG.println("Stopped WIFI");
    }
  }
  static bool isWake(void) {
    return serverWake;
  }
  static IPAddress getIP(void) {
    return WiFi.softAPIP();
  }
  //
  static void lookFloat(const char *key, float *ptr) {
    if (LOOK_INDEX < LOOK_MAX) {
      LOOK_KEY[LOOK_INDEX] = (char*)key;
      LOOK_PTR[LOOK_INDEX] = ptr;
      LOOK_INDEX++;
    }
  }
  //
};
//
const char SERVER::_SSID_[] = "m5atom";
WebServer SERVER::server(80);
bool SERVER::serverInit = false;
bool SERVER::serverWake = false;
//
int SERVER::LOOK_INDEX = 0;
char* SERVER::LOOK_KEY[LOOK_MAX];
float* SERVER::LOOK_PTR[LOOK_MAX];
//
CONFIG SERVER::CONF;
char SERVER::CHAR_BUFF[5000];
//
const char SERVER::HTML_INIT[] = R"(
<!DOCTYPE HTML>
<html>
<head>
<meta name='viewport' content='width=device-width,initial-scale=1' />
<title>GyroM5Atom</title>
</head>
<body>
<h1>GyroM5Atom</h1>
<ul>
<li>'select' for selecting a parameter
<li>'slider' for adjusting the selected parameter
<li>'min/max <- ch1' for setting endpoints
<li>'m5atom <- config' for saving parameters
</ul>
<form action='save' method='get' name='setting'>
<table>
<thead><tr><th>select</th><th>name</th><th>value</th><th>slider</th><th>unit</th></tr></thead>
<tbody></tbody>
</table>
<br>
<input type='hidden' name='JST' value='20220630103030' />
<input type='button' value='min/max <- ch1' onclick='onMinMax()' />
<input type='submit' value='m5atom <- conf' onclick='onSubmit()' />
</form>
</body>
<script>
const CONFIG = %s;
//
function onLoad() {
 //for (let key in CONFIG){ document.getElementsByName(key)[0].value = document.getElementById(key).textContent  = CONFIG[key]; } 
 let tbody = document.getElementsByTagName('tbody')[0];
 for (let key in CONFIG) {
  let spec = CONFIG[key];
  let row = document.createElement('tr');
  // radio
  let c0 = document.createElement('td');
  let i0 = document.createElement('input'); i0.type = 'radio'; i0.name = 'select'; i0.value = key; i0.checked = false;
  if (!spec[5]) i0.onchange = function() { onChange(i0); }; else i0.disabled = true;
  c0.appendChild(i0);
  // name
  let c1 = document.createElement('td');
  c1.appendChild(document.createTextNode(key));
  // value
  let c2 = document.createElement('td');
  let s2 = document.createElement('span'); s2.id = key; s2.textContent = spec[3];
  c2.appendChild(s2);
  // range
  let c3 = document.createElement('td');
  let i3 = document.createElement('input'); i3.type = 'range'; i3.name = key; i3.min = spec[0]; i3.max = spec[1]; i3.step = spec[2]; i3.value = spec[3]; i3.disabled = true;
  if (!spec[5]) i3.oninput = function(){ onInput(i3); };
  c3.appendChild(i3);
  // description
  let c4 = document.createElement('td');
  c4.appendChild(document.createTextNode(spec[4]));
  //
  row.appendChild(c0); row.appendChild(c1); row.appendChild(c2); row.appendChild(c3); row.appendChild(c4);
  tbody.appendChild(row);
 }
}
//
function onInput(range) {
 if (range.name in CONFIG) document.getElementById(range.name).textContent = range.value;
}
//
function doAssign(key,val) {
 if (key in CONFIG) document.getElementsByName(key)[0].value = document.getElementById(key).textContent  = val; 
}
//
function onChange(radio) {
 for (let key in CONFIG) {
  let range = document.getElementsByName(key)[0];
  let spec = CONFIG[key];
  if (key == radio.value & !spec[5]) { 
   range.disabled = false;
  } else {
   range.disabled = true;
  }
 }
}
//
function onMinMax() {
  let usec = document.getElementById('CH1_USEC').textContent;
  if (usec <= 0) return;
  else if (usec < 1500-250) doAssign('MIN',usec);
  else if (usec > 1500+250) doAssign('MAX',usec);
  else doAssign('MEAN',usec);

}
//
function onSubmit() {
 const D2 = function(s) { return ('0'+s).slice(-2); };
 let now = new Date();
 document.getElementsByName('JST')[0].value = now.getFullYear() + D2(now.getMonth()+1) + D2(now.getDate()) + D2(now.getHours()) + D2(now.getMinutes()) + D2(now.getSeconds());
 for (let key in CONFIG) document.getElementsByName(key)[0].disabled = false;
}
//
const PERIOD_MSEC = 500;
//
function startAjax() {
 var xhr = new XMLHttpRequest();
 xhr.open('GET','/json');
 xhr.onload = function() {
  let json = JSON.parse(xhr.response);
  //console.log(json);
  for (let key in json) doAssign(key,json[key]);
  setTimeout(startAjax,PERIOD_MSEC);
 }
 xhr.setRequestHeader('Content-Type','application/json');
 xhr.timeout = PERIOD_MSEC*2;
 xhr.send();
}
//
window.onload = onLoad();
startAjax();
</script>
</html>
)";


const char SERVER::HTML_SAVE[] = R"(
<!DOCTYPE HTML>
<html>
<head>
<meta name='viewport' content='width=device-width,initial-scale=1' />
<title>GyroM5Atom</title>
</head>
<body>
<h1>GyroM5Atom</h1>
parameters:
<ul>
</ul>
successfully saved!
</body>
<script>
const CONFIG = %s;
//
function onLoad() {
 let ul = document.getElementsByTagName('ul')[0];
 for (let key in CONFIG) {
  let spec = CONFIG[key];
  if (spec[5] < 2) {
   // list
   let li = document.createElement('li');
   li.innerHTML = key + ' = ' + spec[3] + ' ( ' + spec[4] + ' )';
   ul.appendChild(li);
  }
 }
}
//
window.onload = onLoad();
</script>
</html>
)";




////////////////////////////////////////////////////////////////////////////////
// class TimerMS{}: タイマ管理用ライブラリ
//  isUp(): 指定時間の経過有無（タイマ更新あり）
//  getFreq(): タイマ周期の参照[Hz]
//  getDelta(): タイマ経過の参照[msec]
//  touch(): タイマ更新（ウォッチドッグタイマ等で利用）
//  isOld(): 指定時間の経過有無（タイマ更新なし）
////////////////////////////////////////////////////////////////////////////////
class TimerMS {
  unsigned long last;
  int freq;
public:
  TimerMS() {
    last = 0;
    freq = 1;
  }

  bool isUp(int msec) {
    unsigned long now = millis();
    if (last + msec <= now) {
      freq = 1000 / (now - last); 
      last = now;
      return true;
    }
    return false;
  }
  int getFreq(void) {
    return freq;
  }
  int getDelta(void) {
    return millis() - last; 
  }
  void touch(void) {
    last = millis();
  }
  bool isOld(int msec) {
    unsigned long now = millis();
    return last + msec <= now;
  }
};



////////////////////////////////////////////////////////////////////////////////
// class CountHz{}: 周期計測用ライブラリ
//  getFreq(): 周期の取得[Hz]
//  touch(): カウント（例：ループ内で呼ぶ）
////////////////////////////////////////////////////////////////////////////////
class CountHZ {
  unsigned long last;
  int freq, loop;
public:
  CountHZ() {
    last = 0;
    freq = 0;
    loop = 0;
  }
  void touch(void) {
    unsigned long now = millis();
    loop++;
    if (last + 1000 <= now) {
      last = now;
      freq = loop;
      loop = 0;
    }
  }
  int getFreq(void) {
    unsigned long now = millis();
    return last + 1100 < now? 0: freq;
  }
};





////////////////////////////////////////////////////////////////////////////////
// class PulsePort{}: PWM信号の入出力ライブラリ
//  setupIn(): 入力ピンの初期化
//  getUsec(): 入力パルス幅[usec]
//  getFreq(): 入力パルス周波数[Hz]
//  setupMean(): 入力パルス平均
//  getUsecMean(): 入力パスル平均[usec]
//  attach(): 割り込み処理の再開
//  detach(): 割り込み処理の中止
//  setupOut(): 出力ピンの初期化
//  putUsec(): 出力パルス幅[usec]
//  putFreq(): 出力パルス周波数[Hz]
////////////////////////////////////////////////////////////////////////////////
// PWM watch dog timer
#include <Ticker.h>

// PWM pulse in
typedef struct {
  int pin;
  int tout;
  // for pulse
  int dstUsec;
  int prev;
  unsigned long last;
  // for freq
  int dstFreq;
  unsigned long lastFreq;
} InPulse;

// PWM pulse out
typedef struct {
  int pin;
  int freq;
  int bits;
  int duty;
  int usec;
  int dstUsec;
} OutPulse;

class PulsePort {
  static const int MAX = 4; // max of channels 

  static int InCH;   // number of in-channels
  static int OutCH;   // number of out-channels
  
  static InPulse IN[MAX]; // pwm in-pulse
  static OutPulse OUT[MAX]; // pwm out-pulse

  static Ticker WDT;      // watch dog timer
  static bool WATCHING;
  static float MEAN[MAX]; // mean of pwm in-pulse
  
  static void ISR(void *arg) {
    unsigned long tnow = micros();
    int ch = (int)arg;
    InPulse* pwm = &IN[ch];
    int vnow = digitalRead(pwm->pin);
    
    if (pwm->prev==0 && vnow==1) {
      // at up edge
      pwm->prev = 1;
      pwm->last = tnow;
      // for freq
      pwm->dstFreq = (tnow > pwm->lastFreq? 1000000/(tnow - pwm->lastFreq): 0);
      pwm->lastFreq = tnow;
    }
    else
    if (pwm->prev==1 && vnow==0) {
      // at down edge
      pwm->dstUsec = tnow - pwm->last;
      pwm->prev = 0;
      pwm->last = tnow;
    }
  }

  static void TSR(void) {
    unsigned long tnow = micros();
    for (int ch=0; ch<InCH; ch++) {
      InPulse* pwm = &IN[ch];
      if (pwm->last + pwm->tout < tnow) {
        pwm->dstUsec = 0;
        pwm->dstFreq = 0;
      }
    }
  }

public: 
  PulsePort() {
    // do nothing
  };
  static int setupIn(int pin, int toutUs=21*1000) {
    int ch = -1;
    if (InCH < MAX) {
      ch = InCH++;
      InPulse* pwm = &IN[ch];
      //
      pwm->pin = pin;
      pwm->tout = toutUs;
      // for pulse
      pwm->dstUsec = 0;
      pwm->prev = 0;
      pwm->last = micros();
      // for freq
      pwm->dstFreq = 0;
      pwm->lastFreq = micros();
      //
      pinMode(pin,INPUT);
      attachInterruptArg(pin,&ISR,(void*)ch,CHANGE);
      if (ch == 0) WDT.attach_ms(pwm->tout/1000,&TSR);
      WATCHING = true;
    }
    return ch;
  };
  static int getUsec(int ch) {
    if (ch >= 0 && ch < InCH) {
      InPulse* pwm = &IN[ch];
      return pwm->dstUsec;
    }
    return -1;
  }
  static int getFreq(int ch) {
    if (ch >= 0 && ch < InCH) {
      InPulse* pwm = &IN[ch];
      return pwm->dstFreq;
    }
    return -1;
  }
  
  static void detach(void) {
    if (InCH == 0) return;
    WDT.detach();
    for (int ch=0; ch<InCH; ch++) {
      InPulse *pwm = &IN[ch];
      detachInterrupt(pwm->pin);
    }
    WATCHING = false;
  };
  static void attach(void) {
    if (InCH == 0 || WATCHING) return;
    for (int ch=0; ch<InCH; ch++) {
      InPulse *pwm = &IN[ch];
      attachInterruptArg(pwm->pin,&ISR,(void*)ch,CHANGE);
      if (ch == 0) WDT.attach_ms(pwm->tout/1000,&TSR);
    }
    WATCHING = true;
  };

  // ESP32's PWM channel 0 and 1 have common frequency.
  static inline int CH2PWM(int ch) { return (ch)*2; };

  static int setupOut(int pin, int freq = 50, int bits = 16) {
    int ch = -1;
    if (OutCH < MAX) {
      ch = OutCH++;
      OutPulse* out = &OUT[ch];
      out->pin = pin;
      out->freq = freq;
      out->bits = bits;
      out->duty = (1 << bits);
      out->usec = 1000000/freq;
      //
      pinMode(out->pin,OUTPUT);
      ledcSetup(CH2PWM(ch),out->freq,out->bits);
      ledcWrite(CH2PWM(ch),0);
      ledcAttachPin(out->pin,CH2PWM(ch));
      //DEBUG.printf("setupOut: ch=%d freq=%d bits=%d usec=%d\n",ch,out->freq,out->bits,out->usec);
    }
    return ch;
  }
  static float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  static bool putUsec(int ch, float usec) {
    if (ch >= 0 && ch < OutCH) {
      OutPulse* out = &OUT[ch];
      int duty = mapFloat(usec, 0,out->usec, 0,out->duty);
      ledcWrite(CH2PWM(ch), duty);
      out->dstUsec = usec;
      return true;
    }
    return false;
  }
  static bool putFreq(int ch, int freq) {
    if (ch >= 0 && ch < OutCH) {
      OutPulse* out = &OUT[ch];
      out->freq = freq;
      out->usec = 1000000/freq;
      //
      ledcWrite(CH2PWM(ch),0);
      ledcDetachPin(out->pin);
      ledcSetup(CH2PWM(ch),out->freq,out->bits);
      ledcWrite(CH2PWM(ch),0);
      ledcAttachPin(out->pin,CH2PWM(ch));
      //DEBUG.printf("putFreq: ch=%d freq=%d bits=%d usec=%d\n",ch,out->freq,out->bits,out->usec);
      return true;
    }
    return false;
  }

  static void setupMean(bool first = false, int msec = 1000) {
    if (first) {
      unsigned long int timeout = millis() + msec;
      int count = 0;
      for (int ch=0; ch<MAX; ch++) MEAN[ch] = 0.0F;
      while (timeout > millis()) {
        for (int ch=0; ch<MAX; ch++) MEAN[ch] += getUsec(ch);
        count++;
        delay(5);
      }
      for (int ch=0; ch<MAX; ch++) MEAN[ch] /= count;
    }
//    detach();
//    if (PREFS.begin(M5LOGGER)) {
//      if (first)
//        PREFS.putBytes("servo", (uint8_t*)MEAN, sizeof(MEAN));
//      else
//        PREFS.getBytes("servo", (uint8_t*)MEAN, sizeof(MEAN));
//      PREFS.end();
//    }
//    attach();
  }
  static float getUsecMean(int ch) { return MEAN[ch]; }

  static void dump(void) {
    for (int ch=0; ch<InCH; ch++) {
      InPulse* pwm = &IN[ch];
      DEBUG.printf(" in(%d): pin=%2d pulse=%6d (usec) freq=%4d (Hz)\n", ch,pwm->pin,pwm->dstUsec,pwm->dstFreq);
    }
    for (int ch=0; ch<OutCH; ch++) {
      OutPulse* out = &OUT[ch];
      DEBUG.printf("out(%d): pin=%2d pulse=%6d (usec) freq=%4d (Hz)\n", ch,out->pin,out->dstUsec,out->freq);
    }
  }

};

// initialization for static class member
int PulsePort::InCH = 0;
int PulsePort::OutCH = 0;

InPulse PulsePort::IN[PulsePort::MAX];
OutPulse PulsePort::OUT[PulsePort::MAX];

Ticker PulsePort::WDT;
bool PulsePort::WATCHING = false;
float PulsePort::MEAN[PulsePort::MAX];



////////////////////////////////////////////////////////////////////////////////
// class ServoPID{}: PID（比例、積分、微分）制御アルゴリズム（QuickPIDのラッパ）
//  setup(): PID制御のパラメータ変更
//  loop(): PID制御の出力計算
////////////////////////////////////////////////////////////////////////////////
#include <QuickPID.h>

class ServoPID {
public:
  
  float Setpoint, Input, Output;
  float Min, Mean, Max;
  QuickPID* QPID;
  
  ServoPID(void) {
    //
    Setpoint = 0.0F;
    Input = 0.0F;
    Output = 0.0F;
    //
    Mean = 1500;
    Min = 1000 - Mean;
    Max = 2000 - Mean;
    //
    QPID = new QuickPID(&Input, &Output, &Setpoint, 1.0,0.0,0.0, QuickPID::Action::direct);
    QPID->SetAntiWindupMode(QuickPID::iAwMode::iAwClamp);
    QPID->SetMode(QuickPID::Control::automatic);
    QPID->SetOutputLimits(Min,Max);
    QPID->SetSampleTimeUs(1000000/50);
  }
  
  // PID setup
  void setup(float Kp, float Ki, float Kd, int MIN=1000, int MEAN=1500, int MAX=2000, int Hz=50) {
    Min = MIN - MEAN;
    Mean = MEAN;
    Max = MAX - MEAN;
  
    QPID->SetTunings(Kp,Ki,Kd);
    QPID->SetOutputLimits(Min,Max);
    QPID->SetSampleTimeUs(Hz>=50? 1000000/Hz: 1000000/50);
  }
  void setupT(float Kp, float Ti, float Td, int MIN=1000, int MEAN=1500, int MAX=2000, int Hz=50) {
    if (Ti <= 0.0) Ti = 1.0;
    float Ki = Kp/Ti;
    float Kd = Kp*Td;
    setup(Kp,Ki,Kd, MIN,MEAN,MAX,Hz);
  }
  void setupU(float Ku, float Tu, int MIN=1000, int MEAN=1500, int MAX=2000, int Hz=50) {
    if (Tu <= 0.0) Tu = 1.0;
    // Ziegler–Nichols method
    float Ti = 0.50*Tu;
    float Td = 0.125*Tu;
    float Kp = 0.60*Ku;
    float Ki = Kp/Ti;
    float Kd = Kp*Td;
    setup(Kp,Ki,Kd, MIN,MEAN,MAX,Hz);
  }
  
  // PID loop
  float loop(float SP, float PV) {
    // Compute PID
    Setpoint = (SP > 0? SP - Mean: 0.0);
    Input = PV;
    QPID->Compute();
    return SP > 0? Mean + constrain(Output,Min,Max): 0;
  }

  // debug print
  void debug(void) {
    DEBUG.printf("%.2f %.2f %.2f\n", Setpoint,Input,Output);
  }

};




////////////////////////////////////////////////////////////////////////////////
// class M5StackAHRS{}: 姿勢推定用ライブラリ（可変更新周期、座標変換などに対応）
//  setup(): AHRSの初期化
//  loop(): AHRSの更新
//  initAXIS(): 座標軸の変更（シャーシ固定系の変更）
//  initMEAN(): バイアスの更新（センサのキャリブレーション）
////////////////////////////////////////////////////////////////////////////////
class M5StackAHRS {
  /* AHRS */
  //#include <utility/MahonyAHRS.h>
  //---------------------------------------------------------------------------------------------------
  // Definitions
  
  //#define sampleFreq  25.0f     // sample frequency in Hz
  float sampleFreq = 25.0;
  #define twoKpDef  (2.0f * 1.0f) // 2 * proportional gain
  #define twoKiDef  (2.0f * 0.0f) // 2 * integral gain
  
  //#define twoKiDef  (0.0f * 0.0f)
  
  //---------------------------------------------------------------------------------------------------
  // Variable definitions
  
  volatile float twoKp = twoKpDef;                      // 2 * proportional gain (Kp)
  volatile float twoKi = twoKiDef;                      // 2 * integral gain (Ki)
  volatile float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
  volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

  //---------------------------------------------------------------------------------------------------
  // IMU algorithm update
  
  void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float *pitch,float *roll,float *yaw) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
  
  
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
  
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
  
      // Estimated direction of gravity and vector perpendicular to magnetic flux
      halfvx = q1 * q3 - q0 * q2;
      halfvy = q0 * q1 + q2 * q3;
      halfvz = q0 * q0 - 0.5f + q3 * q3;
  
      
  
      // Error is sum of cross product between estimated and measured direction of gravity
      halfex = (ay * halfvz - az * halfvy);
      halfey = (az * halfvx - ax * halfvz);
      halfez = (ax * halfvy - ay * halfvx);
  
      // Compute and apply integral feedback if enabled
      if(twoKi > 0.0f) {
        integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
        integralFBy += twoKi * halfey * (1.0f / sampleFreq);
        integralFBz += twoKi * halfez * (1.0f / sampleFreq);
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
      }
      else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
      }
  
      // Apply proportional feedback
      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }
  
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));   // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
  
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  
  
    *pitch = asin(-2 * q1 * q3 + 2 * q0* q2); // pitch
    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); // roll
    *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);  //yaw
  
    *pitch *= RAD_TO_DEG;
      *yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //  8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      *yaw   -= 8.5;
      *roll  *= RAD_TO_DEG;
  
    ///Serial.printf("%f    %f    %f \r\n",  pitch, roll, yaw);
  }
  
  //---------------------------------------------------------------------------------------------------
  // Fast inverse square-root
  // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
  
  float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
  #pragma GCC diagnostic warning "-Wstrict-aliasing"
    y = y * (1.5f - (halfx * y * y));
    return y;
  }

  
  /* IMU values */
  float accl[3] = {0.0,0.0,0.0};
  float gyro[3] = {0.0,0.0,0.0};
  /* IMU mean values */
  float ACCL[3] = {0.0,0.0,0.0};
  float GYRO[3] = {0.0,0.0,0.0};
  /* IMU temp */
  float temp = 0.0F;
  /* AHRS */
  float pitch = 0.0F;
  float roll = 0.0F;
  float yaw = 0.0F;
  
  /* Time */
  uint32_t Now = 0;
  uint32_t lastUpdate = 0;
  float deltat = 1.0f;
  
  /* Axis = Body Fixed Frame */
  float X[3] = {1.0,0.0,0.0};
  float Y[3] = {0.0,1.0,0.0};
  float Z[3] = {0.0,0.0,1.0};

  /* LPF */
  //FilterLP LPF[6];
  
  /* Vetor Operations */
  float dot(float* a,float* b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
  float norm(float* a) { return sqrt(dot(a,a)); }
  void mul(float* a,float k,float* b) {
    b[0] = k*a[0];
    b[1] = k*a[1];
    b[2] = k*a[2];
  }
  void add(float* a,float* b,float* c) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
  }
  void sub(float* a,float* b,float* c) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
  }
  void dup(float* a,float* b) {
    b[0] = a[0];
    b[1] = a[1];
    b[2] = a[2];
  }
  void normalize(float* a) { 
    float na = norm(a);
    a[0] /= na;
    a[1] /= na;
    a[2] /= na;
  }
  void cross(float* a,float* b, float* c) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
  }

public:
  /* initialize */
  void initMEAN(int msec = 2000) {
    for (int i=0; i<3; i++) {
      ACCL[i] = 0.0F;
      GYRO[i] = 0.0F;
    }
    
    int N = 0;
    unsigned long int timeout = millis() + msec;
    while (millis() < timeout) {
      M5.IMU.getGyroData(&gyro[0],&gyro[1],&gyro[2]);
      M5.IMU.getAccelData(&accl[0],&accl[1],&accl[2]);
      for (int i=0; i<3; i++) {
        ACCL[i] += accl[i];
        GYRO[i] += gyro[i];
      }
      N++;
      delay(1);
    }
  
    for (int i=0; i<3; i++) {
      ACCL[i] /= N;
      GYRO[i] /= N;
    }
  }
  
  void initAXIS(int xdir = 1) {
    // initial X0
    int xabs = abs(xdir);
    if (xabs>=1 && xabs<=3) {
      X[0] = (xabs==1? xdir/xabs: 0);
      X[1] = (xabs==2? xdir/xabs: 0);
      X[2] = (xabs==3? xdir/xabs: 0);
    }
    
    // Z := Anti Gravity
    dup(ACCL,Z);
    normalize(Z);
  
    // X := X0 - (Z,X0)Z
    float zx = dot(Z,X);
    float zxZ[3];
    mul(Z,zx,zxZ);
    sub(X,zxZ,X);
    normalize(X);
  
    // Y := Z x X
    cross(Z,X,Y);
    normalize(Y);
  }
  
  void setup(int msec = 2000, int xdir = 1) {
    M5.IMU.Init();
    initMEAN(msec);
    initAXIS(xdir);
  }
  
  void loop(float *gyro_=NULL, float *accl_=NULL, float *ahrs_=NULL, float *temp_=NULL) {
    // put your main code here, to run repeatedly:
    M5.IMU.getGyroData(&gyro[0], &gyro[1], &gyro[2]);
    M5.IMU.getAccelData(&accl[0], &accl[1], &accl[2]);
    M5.IMU.getTempData(&temp);

    // remove bias
    for (int i=0; i<3; i++) gyro[i] -= GYRO[i];
    
    // time update
    Now = millis();
    deltat = ((Now - lastUpdate) / 1000.0);
    lastUpdate = Now;
    sampleFreq = 1.0/deltat;

    //M5.IMU.getAhrsData(&pitch,&roll,&yaw);
    MahonyAHRSupdateIMU(dot(gyro,X)*DEG_TO_RAD,dot(gyro,Y)*DEG_TO_RAD,dot(gyro,Z)*DEG_TO_RAD, dot(accl,X),dot(accl,Y),dot(accl,Z), &pitch,&roll,&yaw);

    // copy results
    if (gyro_) {
      gyro_[0] = dot(gyro,X);
      gyro_[1] = dot(gyro,Y);
      gyro_[2] = dot(gyro,Z);
    }
    if (accl_) {
      accl_[0] = dot(accl,X);
      accl_[1] = dot(accl,Y);
      accl_[2] = dot(accl,Z);
    }
    if (ahrs_) {
      ahrs_[0] = roll;
      ahrs_[1] = pitch;
      ahrs_[2] = yaw;
    }
    if (temp_) {
      *temp_ = temp;
    }
  }
  void debug() {
    DEBUG.println("M5StackAHRS:");
    DEBUG.printf(" gyro=(%.2f,%.2f,%.2f)\n",gyro[0],gyro[1],gyro[2]);
    DEBUG.printf(" accl=(%.2f,%.2f,%.2f)\n",accl[0],accl[1],accl[2]);
    DEBUG.printf(" ahrs=(%.2f,%.2f,%.2f)\n",roll,pitch,yaw);
    DEBUG.printf(" GYRO=(%.2f,%.2f,%.2f)\n",GYRO[0],GYRO[1],GYRO[2]);
    DEBUG.printf(" ACCL=(%.2f,%.2f,%.2f)\n",ACCL[0],ACCL[1],ACCL[2]);
    DEBUG.printf(" X=(%.2f,%.2f,%.2f)\n",X[0],X[1],X[2]);
    DEBUG.printf(" Y=(%.2f,%.2f,%.2f)\n",Y[0],Y[1],Y[2]);
    DEBUG.printf(" Z=(%.2f,%.2f,%.2f)\n",Z[0],Z[1],Z[2]);   
  }
  
  int getFreq(void) { return int(1.0/deltat); }
#if 0
  float getAccT(void) { return LPF[0].update(dot(accl,X)); }
  float getAccL(void) { return LPF[1].update(dot(accl,Y)); }
  float getAccV(void) { return dot(accl,Z); }
  float getYawRate(void) { return LPF[2].update(dot(gyro,Z)); }
  float getRoll(void) {
    float Roll = - atan2(dot(accl,X),dot(accl,Z));
    return LPF[3].update(Roll * (180.0/PI));
  }
  float getPitch(void) {
    float Pitch = atan2(dot(accl,Y),dot(accl,Z));
    return LPF[4].update(Pitch * (180.0/PI));
  }
  float getTraction(float G0, float y0=0.1) {
    float x = getAccT()/G0;
    return fabs(x)>=1.0? y0: sqrt(1.-x*x); 
  }
#endif

};

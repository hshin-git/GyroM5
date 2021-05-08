//////////////////////////////////////////////////
// M5StickC project:
//   Steering Assist Gyroscope for Radio Control car 
// PID control law:
//   error = ch1_in - Kg*yaw_rate
//   ch1_out = Kp*( error + Ki*LPF(error) + Kd*HPF(error) )
// URL:
//   https://github.com/hshin-git/GyroPID
//////////////////////////////////////////////////
#include <M5StickC.h>

//////////////////////////////////////////////////
// Global constants
//////////////////////////////////////////////////
// PWM parameters
const int PWM_HERZ = 50;
const int PWM_BITS = 16;
const int PWM_CH1 = 0;
const int PWM_CH2 = 1;
//
const int PWM_DMAX = (1<<PWM_BITS);
const int PWM_CYCL = 1000000/PWM_HERZ;
const int PWM_WAIT = PWM_CYCL + 5000;
//
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;
const int RANGE_MAX = 100;
// GPIO pins
const int CH1_IN = 0;
const int CH3_IN = 36;
const int CH1_OUT = 26;


//////////////////////////////////////////////////
// LCD helpers
//////////////////////////////////////////////////
void lcd_clear(int bg_color=TFT_WHITE, int fg_color=TFT_BLACK) {
  M5.Lcd.fillScreen(bg_color);
  M5.Lcd.setTextColor(fg_color);
  M5.Lcd.setCursor(0,0);
}
void lcd_header(char *text) {
  lcd_clear();
  M5.Lcd.printf(text);
}


//////////////////////////////////////////////////
// 5Vin watcher
//////////////////////////////////////////////////
int lastVinTime = 0;
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
  float vin = M5.Axp.GetVinData()*1.7 /1000;
  float usb = M5.Axp.GetVusbinData()*1.7 /1000;
  //Serial.printf("vin,usb = %f,%f\n",vin,usb);
  if ( vin < 3.0 && usb < 3.0 ) {
    if ( lastVinTime + 5000 < millis() ) {
      axp_halt();
    }
  } else {
    lastVinTime = millis();
  }
}


//////////////////////////////////////////////////
// Low/High Pass Filters
//////////////////////////////////////////////////
float OMEGA_LPF[4];
float CH1DT_LPF[4];
float CH1ER_LPF[4];
float CH1ER_HPF[4];
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
  float y = alpha*(x - xp) + (1.-alpha)*yp;
  buf[0] = y;
  buf[1] = x;
  return y;
}


//////////////////////////////////////////////////
// Zero calibrators
//////////////////////////////////////////////////
float OMEGA_ZERO = 0.0;
float CH1DT_ZERO = 0.0;
void zero_calibration(void) {
  lcd_header("\nGyroPID\nWAIT INPUT\n");
  for (int n=0; n<10; n++) {
    while (pulseIn(CH1_IN,HIGH,PWM_WAIT)==0) vin_watch();
  }
  for (int n=0; n<200; n++) {
    int ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
    float omg[3];
    M5.MPU6886.getGyroData(&omg[0],&omg[1],&omg[2]);
    CH1DT_ZERO = lpf_update(CH1DT_LPF, map(ch1, 0,PWM_CYCL, 0,PWM_DMAX));
    OMEGA_ZERO = lpf_update(OMEGA_LPF, omg[2] * M5.MPU6886.gRes);
    //
    if (n%10==0) {
      lcd_header("\nZERO INPUT\n");
      M5.Lcd.printf(" CH1: %7.2f\n",CH1DT_ZERO);
      M5.Lcd.printf(" OMG: %7.2f\n",OMEGA_ZERO);
    }
  }
}


//////////////////////////////////////////////////
// GyroPID parameter table
//////////////////////////////////////////////////
#include <Preferences.h>
Preferences PARAM;
// CH1 parameters
const int DMIN = 0; // CH1 minimum
const int DMAX = 1; // CH1 maximum
const int ZERO = 2; // CH1 neutral (not in use)
// PID parameters
const int I_KG = 3; // PID gain KG
const int I_KP = 4; // PID gain KP
const int I_KI = 5; // PID gain KI
const int I_KD = 6; // PID gain KD
//
int CONF[8];
// conf print
void conf_show() {
  lcd_header("\nCONF\n");
  // duty range
  M5.Lcd.printf(" DMIN: %6d\n",CONF[DMIN]);
  M5.Lcd.printf(" DMAX: %6d\n",CONF[DMAX]);
  //M5.Lcd.printf(" ZERO: %6d\n",CONF[ZERO]);  
  // gain param
  M5.Lcd.printf(" KG: %6d\n",CONF[I_KG]);
  M5.Lcd.printf(" KP: %6d\n",CONF[I_KP]);
  M5.Lcd.printf(" KI: %6d\n",CONF[I_KI]);
  M5.Lcd.printf(" KD: %6d\n",CONF[I_KD]);
}
// conf ch1 duty range
void conf_range() {
  int ch1,val;
  for (int n=0; n<2; n++) {
    delay(500);
    while (true) {
      ch1 = pulseIn(CH1_IN,HIGH,PWM_WAIT);
      val = map(ch1, 0,PWM_CYCL, 0,PWM_DMAX);
      ledcWrite(PWM_CH1,(ch1>0? val: 0));
      lcd_clear();
      M5.Lcd.printf("\n%s\n", (n? "LEFT": "RIGHT"));
      M5.Lcd.printf("[A] SAVE\n");
      M5.Lcd.printf("[B] CANCEL\n");
      M5.Lcd.printf(" CH1: %6d\n",ch1);
      M5.Lcd.printf(" VAL: %6d\n",val);
      delay(50);
      M5.update();
      if (M5.BtnA.isPressed()) {
        if (ch1>0) CONF[(n? DMAX: DMIN)] = val;
        break;
      }
      if (M5.BtnB.isPressed()) {
        return;
      }
    } 
  }
  // save
  if (ch1>0) {
    if (CONF[DMAX] < CONF[DMIN]) {
      int temp = CONF[DMAX];
      CONF[DMAX] = CONF[DMIN];
      CONF[DMIN] = temp;
    }
    PARAM.putBytes("CONF",&CONF,sizeof(CONF));
  }
  conf_show();
  delay(5*1000);  
}
// conf gyro gain
void conf_value(int pin, int pos, char *str) {
  int ch1,val;
  int old = CONF[pos];
  delay(500);
  // Gain
  while (true) {
    ch1 = pulseIn(pin,HIGH,PWM_WAIT);
    val = map(ch1, PULSE_MIN,PULSE_MAX, -RANGE_MAX,RANGE_MAX);
    lcd_header(str);
    M5.Lcd.printf("[A] SAVE\n");
    M5.Lcd.printf("[B] CANCEL\n");
    M5.Lcd.printf(" CH1: %6d\n",ch1);
    M5.Lcd.printf(" NEW: %6d\n",val);
    M5.Lcd.printf(" OLD: %6d\n",old);
    delay(50);
    M5.update();
    if (M5.BtnA.isPressed()) {
      if (ch1>0) CONF[pos] = val;
      break;
    }
    if (M5.BtnB.isPressed()) {
      return;
    }
  }
  // save
  if (ch1>0) {
    PARAM.putBytes("CONF",&CONF,sizeof(CONF));    
  }
  conf_show();
  delay(5*1000);
}
//
void conf_menu() {
  unsigned long enter = millis();
  int pos = 0;
  delay(500);
  while (millis() < enter + 5*1000) {
    lcd_clear();
    M5.Lcd.printf("\nCONF MENU\n");
    M5.Lcd.printf("[A] SELECT\n");
    M5.Lcd.printf("[B] NEXT\n");
    M5.Lcd.printf(" %s PID KG\n",  (pos==0? ">": " "));
    M5.Lcd.printf(" %s PID KP\n",  (pos==1? ">": " "));
    M5.Lcd.printf(" %s PID KI\n",  (pos==2? ">": " "));
    M5.Lcd.printf(" %s PID KD\n",  (pos==3? ">": " "));
    M5.Lcd.printf(" %s CH1 ZERO\n",(pos==4? ">": " "));
    M5.Lcd.printf(" %s CH1 DUTY\n",(pos==5? ">": " "));
    delay(200);   
    M5.update();
    if (M5.BtnA.isPressed()) {
      switch (pos) {
        case 0: conf_value(CH1_IN,I_KG,"\nPID KG\n"); break;
        case 1: conf_value(CH1_IN,I_KP,"\nPID KP\n"); break;
        case 2: conf_value(CH1_IN,I_KI,"\nPID KI\n"); break;
        case 3: conf_value(CH1_IN,I_KD,"\nPID KD\n"); break;
        case 4: zero_calibration(); break;
        case 5: conf_range(); break;
        default: break;
      }
    }
    if (M5.BtnB.isPressed()) {
      enter = millis();
      pos = (pos + 1) % 6;
    }
    //delay(200);
  }
}


//////////////////////////////////////////////////
// CH3 mode switcher
//////////////////////////////////////////////////
// CH3 PWM width and mode
int CH3_USEC = 0;
int CH3_MODE = 0;
//
void ch3_select() {
  unsigned long enter = millis();
  delay(500);
  while (millis() < enter + 5*1000) {
    lcd_clear();
    M5.Lcd.printf("\nCH3 MODE\n");
    M5.Lcd.printf("[B] NEXT\n");    
    if (CH3_USEC > 0) {
      M5.Lcd.printf(" %s PID TAB\n", (CH3_MODE==0? ">": " "));
      M5.Lcd.printf(" %s PID KG\n",  (CH3_MODE==1? ">": " "));
      M5.Lcd.printf(" %s PID KP\n",  (CH3_MODE==2? ">": " "));
      M5.Lcd.printf(" %s PID KI\n",  (CH3_MODE==3? ">": " "));
      M5.Lcd.printf(" %s PID KD\n",  (CH3_MODE==4? ">": " "));
      M5.Lcd.printf(" %s PID OFF\n", (CH3_MODE==5? ">": " "));
    } else {
      M5.Lcd.printf(" %s PID TAB\n", (CH3_MODE==0? ">": " "));
      M5.Lcd.printf(" %s PID OFF\n", (CH3_MODE==1? ">": " "));
    }
    delay(200);   
    M5.update();
    if (M5.BtnB.isPressed()) {
      enter = millis();
      if (CH3_USEC>0)
        CH3_MODE = (CH3_MODE + 1) % 6;
      else
        CH3_MODE = (CH3_MODE + 1) % 2;
    }
    //delay(200);
  }
}


//////////////////////////////////////////////////
// put your setup code here, to run once:
//////////////////////////////////////////////////
void setup() {
  // (1) Initialize the M5StickC object
  M5.begin();
  M5.MPU6886.Init();
  M5.Axp.ScreenBreath(10);
  //while (!setCpuFrequencyMhz(80));  

  // (2) Initialize GPIO
  pinMode(CH1_IN,INPUT);
  pinMode(CH3_IN,INPUT);
  
  pinMode(CH1_OUT,OUTPUT);
  ledcSetup(PWM_CH1,PWM_HERZ,PWM_BITS);
  ledcAttachPin(CH1_OUT,PWM_CH1);

  // (3) Initialize parameters
  PARAM.begin("GYRO");
  PARAM.getBytes("CONF",&CONF,sizeof(CONF));

  // (4) Initialize filters
  lpf_init(OMEGA_LPF,2.0/100);
  lpf_init(CH1DT_LPF,2.0/100);
  lpf_init(CH1ER_LPF,2.0/ 50);
  hpf_init(CH1ER_HPF,50./100);  

  // (5) Calibrate zero points 
  zero_calibration();

  // (6) Do others
  //Serial.begin(115200);  
}


//////////////////////////////////////////////////
// put your main code here, to run repeatedly:
//////////////////////////////////////////////////
// PWM widths in usec
int CH1_USEC = 0;
int CH2_USEC = 0;
// IMU values
float OMEGA[3];
float ACCEL[3];
// Main loop counter
int LOOP = 0;

void loop() {
  int ch1_duty,ch1_dout;
  int ch3_gain,KG,KP,KI,KD;
  float omega,error;
  bool drift = false;
  
  // (1) Input PWM values
  CH1_USEC = pulseIn(CH1_IN,HIGH,PWM_WAIT);
  if (LOOP%25 == 0) CH3_USEC = pulseIn(CH3_IN,HIGH,PWM_WAIT);

  // (2) Input IMU values
  M5.MPU6886.getGyroData(&OMEGA[0],&OMEGA[1],&OMEGA[2]);
  //if (LOOP%25 == 0) M5.MPU6886.getAccelData(&ACCEL[0],&ACCEL[1],&ACCEL[2]);

  // (3) Compute PWM value by PID
  ch1_duty = map(CH1_USEC, 0,PWM_CYCL, 0,PWM_DMAX);
  ch3_gain = map(CH3_USEC, PULSE_MIN,PULSE_MAX, -RANGE_MAX,RANGE_MAX);

  // CH3 mode -> See ch3_select().
  KG = CONF[I_KG];
  KP = CONF[I_KP];
  KI = CONF[I_KI];
  KD = CONF[I_KD];    
  if (CH3_USEC > 0) {
    switch (CH3_MODE) {
      case 1: KG = ch3_gain; break;
      case 2: KP = ch3_gain; break;
      case 3: KI = ch3_gain; break;
      case 4: KD = ch3_gain; break;
      case 5: KP = 50; KG = KI = KD = 0; break;
      default: break;
    }
  } else {
    switch (CH3_MODE) {
      case 1: KP = 50; KG = KI = KD = 0; break;
      default: break;
    }
  }
  
  // CH1 duty out
  omega = (OMEGA[2] * M5.MPU6886.gRes) - OMEGA_ZERO;
  error = (ch1_duty - CH1DT_ZERO) - (KG/0.5)*omega;
  ch1_dout = int(CH1DT_ZERO + (KP/50.0)*(error + (KI/50.0)*lpf_update(CH1ER_LPF,error) + (KD/50.0)*hpf_update(CH1ER_HPF,error)));
  ch1_dout = constrain(ch1_dout, CONF[DMIN],CONF[DMAX]);
  ch1_dout = (CH1_USEC>0? ch1_dout: 0);

  // Drifting?
  drift = omega * (ch1_dout - CH1DT_ZERO) < -500. ? true: false;

  // (4) Output PWM value
  ledcWrite(PWM_CH1,ch1_dout);
  
  // (5) Update LCD
  if (LOOP%10 == 0) {
    lcd_clear(drift? TFT_PINK: TFT_WHITE);
    M5.Lcd.printf("\nINPUT (us)\n");
    M5.Lcd.printf(" CH1:%6d\n",CH1_USEC);
    //M5.Lcd.printf(" CH2:%6d\n",CH2_USEC);
    M5.Lcd.printf(" CH3:%6d\n",CH3_USEC);
    M5.Lcd.printf("\nOMEGA (rad/s)\n");
    M5.Lcd.printf("  X:%7.2f\n",OMEGA[0] *M5.MPU6886.gRes);
    M5.Lcd.printf("  Y:%7.2f\n",OMEGA[1] *M5.MPU6886.gRes);
    M5.Lcd.printf("  Z:%7.2f\n",OMEGA[2] *M5.MPU6886.gRes);
    //M5.Lcd.printf("\nACCEL (G)\n");
    //M5.Lcd.printf("  X:%7.2f\n",ACCEL[0]);
    //M5.Lcd.printf("  Y:%7.2f\n",ACCEL[1]);
    //M5.Lcd.printf("  Z:%7.2f\n",ACCEL[2]);
    M5.Lcd.printf("\nDUTY (16bit)\n");
    M5.Lcd.printf("   I:%6d\n",ch1_duty);
    M5.Lcd.printf("   O:%6d\n",ch1_dout);
    M5.Lcd.printf("   E:%6d\n",int(error));
    M5.Lcd.printf("\nGAIN (0-100)\n");
    M5.Lcd.printf(" G/P:%3d/%3d\n",KG,KP);
    M5.Lcd.printf(" I/D:%3d/%3d\n",KI,KD);
    // watch vin and buttons
    vin_watch();
    M5.update();
    if (M5.BtnA.isPressed()) conf_menu();
    if (M5.BtnB.isPressed()) ch3_select();
  }
  
  // (6) Do others
  LOOP = (LOOP + 1) % 50;
}

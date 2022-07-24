#include <M5Atom.h>
#include "GyroM5Atom.hpp"


// GPIO of M5Atom
const int BTM_PIN[] = {22,19,23,33};
const int GRV_PIN[] = {26,32};


// PWM I/O Ports
PulsePort PWM_IO;


// PID Controller
ServoPID PID_CH1;


// FREQ Counter
CountHZ LOOP_HZ;


// AHRS of M5Atom
M5StackAHRS M5_AHRS;
float ACCL[3] = {0.0,0.0,0.0};
float GYRO[3] = {0.0,0.0,0.0};
float AHRS[3] = {0.0,0.0,0.0};


// CONFIG SERVER
SERVER WWW;


// CONTROL PARAMETERS
#define CNF_KG  (WWW.CONF.KG/50.0 * 500./180.0) 
#define CNF_KP  (WWW.CONF.KP/50.0)
#define CNF_KI  (WWW.CONF.KI/250.0)
#define CNF_KD  (WWW.CONF.KD/5000.0)
#define CNF_REV (WWW.CONF.REV)
#define CNF_MIN (WWW.CONF.MIN)
#define CNF_MAX (WWW.CONF.MAX)
#define CNF_MEAN  (WWW.CONF.MEAN)
#define CNF_FREQ  (WWW.CONF.FREQ)
#define CNF_AXIS  (WWW.CONF.AXIS%2? (1+(WWW.CONF.AXIS-1)/2): -(1+(WWW.CONF.AXIS-1)/2))


// MONITOR
float CH1_FREQ = 50;
float CH1_USEC = 1500;
float PID_FREQ = 100;
float PID_USEC = 1500;
float IMU_RATE = 0;
float IMU_PITCH = 0;
float IMU_ROLL = 0;


void setup() {
 
  // put your setup code here, to run once:
  M5.begin(true,true,false); //Init M5Atom-Matrix(Serial, I2C, LEDs).
  M5.IMU.Init();

  // CONF
  WWW.setup();
  WWW.lookFloat("CH1_FREQ",&CH1_FREQ);
  WWW.lookFloat("CH1_USEC",&CH1_USEC);
  WWW.lookFloat("IMU_PITCH",&IMU_PITCH);
  WWW.lookFloat("IMU_ROLL",&IMU_ROLL);
  WWW.lookFloat("IMU_RATE",&IMU_RATE);
  WWW.lookFloat("PID_FREQ",&PID_FREQ);  
  WWW.lookFloat("PID_USEC",&PID_USEC);

  // AHRS
  M5_AHRS.setup(1000,CNF_AXIS);
  
  // PID
  PID_CH1.setup(CNF_KP,CNF_KI,CNF_KD,CNF_MIN,CNF_MEAN,CNF_MAX,400);

  // GPIO
  #if 1
  PWM_IO.setupIn(GRV_PIN[0]);
  PWM_IO.setupOut(GRV_PIN[1],CNF_FREQ);
  #else
  PWM_IO.setupIn(BTM_PIN[0]);
  PWM_IO.setupOut(BTM_PIN[1],CNF_FREQ);
  #endif

}

void loop() {

  // put your main code here, to run repeatedly:
  M5_AHRS.loop(GYRO,ACCL,AHRS);
  LOOP_HZ.touch();
  //
  IMU_PITCH = AHRS[0];
  IMU_ROLL = AHRS[1];
  IMU_RATE = GYRO[2];
  CH1_FREQ = PWM_IO.getFreq(0);
  CH1_USEC = PWM_IO.getUsec(0);
  PID_FREQ = LOOP_HZ.getFreq();
  PID_USEC = PID_CH1.loop(CH1_USEC, (CNF_REV? -CNF_KG*IMU_RATE: CNF_KG*IMU_RATE));
  //
  PWM_IO.putUsec(0, (CH1_USEC>0? PID_USEC: CH1_USEC));

  // config
  M5.update();
  if (M5.Btn.wasPressed() & !WWW.isWake()) {
    WWW.start();
    while (WWW.isWake()) {
      WWW.loop();
      //
      M5_AHRS.loop(GYRO,ACCL,AHRS);
      LOOP_HZ.touch();
      //
      IMU_PITCH = AHRS[0];
      IMU_ROLL = AHRS[1];
      IMU_RATE = GYRO[2];
      CH1_FREQ = PWM_IO.getFreq(0);
      CH1_USEC = PWM_IO.getUsec(0);
      //PID_FREQ = LOOP_HZ.getFreq();
      PID_USEC = PID_CH1.loop(CH1_USEC,(CNF_REV? -CNF_KG*IMU_RATE: CNF_KG*IMU_RATE));
      //
      PWM_IO.putUsec(0, CH1_USEC);
    };
    PID_CH1.setup(CNF_KP,CNF_KI,CNF_KD,CNF_MIN,CNF_MEAN,CNF_MAX,400);
    PWM_IO.putFreq(0,CNF_FREQ);
    M5_AHRS.setup(1000,CNF_AXIS);
    DEBUG.print("AXIS = "); DEBUG.println(CNF_AXIS);
  }

}

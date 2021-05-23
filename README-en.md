[日本語](README.md) | [English](README-en.md)

# GyroM5

- GyroM5 is an OSS for turning your M5StickC into steering assit gyro of RC drift car.
- Sketch [GyroM5.ino](GyroM5/GyroM5.ino) installed M5SitckC stabilizes drift driving of your RC car.

![GyroM5](https://user-images.githubusercontent.com/64751855/117384511-1d46f000-af1e-11eb-854e-45ee149e4671.jpg)


---


# DEMO
This Tamiya RC car (SU-01) with GyroM5 is performing "RWD drifting".

https://user-images.githubusercontent.com/64751855/117535983-a1d76280-b033-11eb-9f59-ec6aaef0b9b0.mp4


# Features
GyroM5 has unique features.

- Feedback control <br> Auto steering to follow yaw rate by PID control
- Parameter setting <br> Setting PID control parameters by CH1 signal
- Remote gain tuning <br> Tuning a PID control parameter by CH3 signal
- End point setting <br> Setting steering end point by CH1 signal
- IMU calibration <br> Auto calibration of zero points in CH1 and IMU
- Monitoring display <br> Displaying RC/IMU signals and PID parameters in LCD
- Drift detection <br> Detecting counter-steer and appearing by LCD


# Requirement
These hardwares are required for GyroM5. 

- Hobby RC car　<br> RC car equipped with standard and separated receiver/servo units.
- Standard PC <br> PC installed with Arduino IDE and equipped with USB.
- M5StickC <br> "M5StickC" not "M5StickC Plus" is required.
- Parts for wire harness <br> One servo extention cable, and one pin headder (8-pin male).
- Soldering tool <br> For assembling wire harness.



# Usage
The outline of usage is as follows. The details are in next section.

## Hardware setting
1. Install Arduino IDE on your PC.
2. Setup Arduino IDE for ESP/M5StickC.
3. Connect your PC and M5StickC with USB.
4. Install GyroM5.ino on your M5StickC.
5. Install GyroM5/M5StickC on your RC car with LCD up.

## Software setting
1. Turn on your GyroM5/M5StickC.
2. Turn on your RC car.
3. Setup steering end point.
4. Setup initial PID control gains (KG=50, KP=50, KI=30, KD=10).
5. Run RC car and adjust PID control gains.

## Daily usage
1. Turn on your GyroM5/M5StickC.
2. Turn on your RC car.
3. Run RC car and adjust PID control gains.


---
# Note
The details are as follows.

## Wiring
Wire GyroM5/M5StickC to RC receiver/servo units, as explained in the table below.

|M5StickC |in/out |RC units |
|---- |---- |---- |
|G26  |in | Reciever CH1|
|G36 |in | Reciever CH3|
|G0 |out | Servo CH1|
|GND |in | Reciever minus|
|5Vin |in | Reciever plus|

An example image of assembled wire harness is as follows.

![WireHarness](https://user-images.githubusercontent.com/64751855/119204830-865b6580-bad1-11eb-9ab6-f055a49f4d88.jpg)

Caution:
Signal levels in M5StickC (3.3v) and RC units (5.0v or more) are generally different.
My RC units use 5.0-6.0v and work no trouble with directly connected M5StickC. 
But higher volotage (over 6.0v) RC units may damage your M5StickC.

![GyroInstall](https://user-images.githubusercontent.com/64751855/117384355-b75a6880-af1d-11eb-88ad-850f1de2ef77.jpg)


## Turning On/off
M5Stick is known to have bugs in its power managment.
Find hints for trouble-shooting with google search like keyword "m5stickc not turning on".



## Monitoring LCD
GyroM5 has five states below.
One state transits to anothr state at button [A]/[B] or timeout event.

![GyroM5](https://user-images.githubusercontent.com/64751855/117535959-70f72d80-b033-11eb-8e9c-6c60c3ccc51d.png)

- State "Operating" is the home, transits to "Setting" by [A] and transits to "CH3 selecting" by [B].
- State other than "Operating" accepts A/B button or returns to "Operating" by timeout.
- In "Gain setting", send integer value by CH1 signal, save the desired value by [A] or cancel by [B]. 
- Remote gain tuning by CH3 is initially disabled. 

|state|transition|descripition|
|----|----|----|
|Waiting |RC signal |waits for PWM signal from RC receiver|
|Calibrating |Timeout |calibrates zero points in CH1 and gyrosensor, dont move RC car|
|Operating |[A],[B] |displays RC signals, IMU inputs and PID gains|
|SettingGain |[A],[B] |sets PID gains and CH1 end points|
|SettingCH3 |[B] |binds CH3 to PID gains|


## Tuning
GyroM5's control algorithm is explained for tuning parameters.


### Algorithm
GyroM5 uses generic feedback control algorithm "PID control". 

![PID_wikipedia](https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/800px-PID_en.svg.png)

In the PID control, the plant/process is RC car.
The target r, the output y and the control u are as follows.

- target: r = ch1_in = CH1 input from RC receiver
- output: y = kg*wz = Yaw rate of RC car
- control: u = ch1_out = CH1 output to RC servo

GyroM5 attempts to minimize error value e by adjusting control variable u.

- error: e = r - y = ch1_in - Kg*wz
- control: u = PID(e) = Kp * (e + Ki * LPF(e) + Kd * HPF(e))

LPF (Low Pass Filter) is used for integral operator, HPF (High Pass Filter)  is used for derivative operator.


### Parameters
You can confirm/adjust PID gain parameters by LCD, A/B buttons and CH1.
PID gains are normalized from -100 to 100, PID gains (in lowercase) are related to PID gains (in uppercase) as follows.

- Yaw rate "wz" is in (radian per sec): <br> IMU sensored values in physical units.
- Input/output "ch1" is in 16bits (0〜64k): <br> Pulse width (0〜20ms=1000ms/50Hz) in 16bit (0〜2^16-1) integer.
- Observation Gain: Kg = KG/0.5 <br> Larger Kg becomes, smaller yaw rate per steering becomes.
- Proportional Gain: Kp = KP/50.0 <br> Larger Kp becomes, more fastly error decreases but may vibrate.
- Integral Gain: Ki = KI/50.0 <br> Larger Ki becomes, more slowly error decreases, and smaller final error is.
- Derivative Gain: Kd = KD/50.0 <br> Larger Kd becomes, more quickly error decreases but may vibrate.

Recommended initial setting is gain parameters "KG=50, KP=60, KI=30, KD=10".
Special parameters "KG=KI=KD=0 and KP=50" is as same as "pass throw: u=r". 
The plus/minus sign of KG is used for normal/reverse operation in steering servo.


### Realtime
GyroM5 is approximately realtime control system, and its control frequency (cycle) is 50Hz (20ms).

- GyroM5 inputs CH1 in every 20ms cycle, and outputs CH1 within 20ms, but inputs CH3 at 500ms intervals.
- Since function pluseIn(...) blocks to read PWM, sequential reading both CH1 and CH3 takes over 20ms.


## Testing
My RC car for testing GyroM5 is as follows.

![UpperView](https://user-images.githubusercontent.com/64751855/117554986-370b4300-b096-11eb-9ef8-50a00980d9fc.jpg)

|item |model |
|----|----|
|chassis |Tamiya SU-01|
|body |Tamiya Jimmny Willy (SJ30) |
|tire |TOPLINE drift tire|
|RC TX |Tamiya fine spec 2.4GHz|
|RC RX |Tamiya TRE-01|
|RC ESC |Tamiya TRE-01|
|RC servo |Yokomo S-007|
|battery |7.4V LiPo 1100mAh|
|motor | 370 type|

Hints for "RWD drifting" are listed bellow.

![LowerView](https://user-images.githubusercontent.com/64751855/117554999-51ddb780-b096-11eb-81c1-7907ea12db07.jpg)

- Larger steering angle is better for controlability.
- Faster steering servo is also better.
- Slippy tires are easier to drift by low power motor.



# Roadmap
The followings are some ideads for improving your GyroM5.

- Wireless setting GyroM5 by smartphone
- Automatic tuning of PID gain parameters
- Assiting not only steering but also throttle 
- Reading PWM input without blocking
- Automatic detection of installed direction
- Recording and analysis of driving data


# Author
The author bought a small RC car kit (Tamiya SU-01) for indoor playing under COVID-19.
After purchasing, I watched the RC car YouTube channel and became interested in "RC drift car" that did not exist in my childhood.
The "RC drift car" is already established as a genre of RC car,
and the shortest course to play "RC drift car" is to get dedicated products like Yokomo YD-2.

But in my case,
I noticed the "RC drift car" after purchasing the kit,
and I believed that any RC car can perform "stable drift driving" by high speed control.
So I tried to make steering assit gyro to stabilize RWD drift driving for my small car.

I enjoyed making this GyroM5,
and I thought this may be a good material to learn programming and control algorithm while playing RC car.
So I release the source code of GyroM5 for expecting that someone use this material for STEM education,
or persuading your parents to buy hobby RC car. (^_^)

I am happy if somebody could reproduce GyroM5 or customize it by themselves.


---

# Reference


## Radio Control Car
- [Yokomo YD-2](https://teamyokomo.com/product/dp-yd2/)
- [Tamiya SU-01](https://www.tamiya.com/japan/products/product_info_ex.html?genre_item=7101)
- [RC Car Shop "Genkikko-san"](https://genkikkosan.com/)

## Automobile Drifting
- [Automotive Vehicle Dynamics](https://www.amazon.co.jp/dp/4501419202/)
- [Vehicle Running Stability Analysis and Spin Control](https://www.tytlabs.com/japanese/review/rev321pdf/321_013ono.pdf)
- [On the dynamics of automobile drifting](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.103.9227&rep=rep1&type=pdf)
- [Analysis and control of high sideslip manoeuvres](https://www.tandfonline.com/doi/abs/10.1080/00423111003746140?journalCode=nvsd20)
- [Stabilization of steady-state drifting for a RWD vehicle](http://dcsl.gatech.edu/papers/avec10.pdf)

## Software
- [M5StickC Library](https://github.com/m5stack/M5StickC)
- [M5StickC non official reference](https://lang-ship.com/reference/unofficial/M5StickC/)
- [M5Stack official documents](https://github.com/m5stack/m5-docs/blob/master/docs/ja/README_ja.md)
- [Arduino IDE](https://www.arduino.cc/en/software)
- [PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

## Hardware
- [M5StickC device](https://www.switch-science.com/catalog/5517/)
- [pin header (male)](https://www.amazon.co.jp/dp/B012HY288S/)
- [servo extention cable](https://www.amazon.co.jp/dp/B00W9ST610/)


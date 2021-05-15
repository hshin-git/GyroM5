[日本語](README.md) | [English](README-en.md)

# GyroM5

- GyroM5 is an OSS for turning your M5SticC to steering assit unit of RC car.
- M5StickC has IMU to measure yaw rate, hardware PMW to command servo and LCD to adjust control parameters, is suited for this purpose.
- M5SitckC installed GyroM5.ino works as a gyro unit for RC car.

![GyroM5](https://user-images.githubusercontent.com/64751855/117384511-1d46f000-af1e-11eb-854e-45ee149e4671.jpg)


---


# DEMO
This Tamiya RC car (SU-01) with GyroM5 is performing "RWD drifting".

https://user-images.githubusercontent.com/64751855/117535983-a1d76280-b033-11eb-9f59-ec6aaef0b9b0.mp4


# Features
GyroM5 has unique functions.

- Feedback control <br> Auto steering to follow yaw rate by PID control
- Parameter setting <br> Setting PID control parameters by CH1 signal
- Remote gain tuning <br> Tuning a PID control parameter by CH3 signal
- End point setting <br> Setting steering end point by CH1 signal
- IMU calibration <br> Auto calibration of zero points in CH1 and IMU
- Monitoring display <br> Displaying RC/IMU signals and PID parameters in LCD
- Drift detection <br> Detecting counter-steer and appearing by LCD


# Requirement
These hardware is needed to use GyroM5. 

- Hobby RC car　<br> RC car equipped with standard and separated receiver/servo units.
- Standard PC <br> PC installed with Arduino IDE and equipped with USB.
- M5StickC <br> "M5StickC" not "M5StickC Plus" is required.
- Parts for wire harness <br> one servo extention cable, and one pin headder (male 8-pin)
- Soldering tool <br> for assembling wire harness



# Usage
The outlines are below. The details are in next section.


## Setup hardware
1. Install Arduino IDE on your PC.
2. Setup Arduino IDE for ESP/M5StickC.
3. Connect your PC and M5StickC with USB.
4. Install GyroM5.ino on your M5StickC.
5. Install GyroM5/M5StickC on your RC car with LCD up.

## Initlal setup
1. Turn on your GyroM5/M5StickC.
2. Turn on your RC car.
3. Setup steering end point.
4. Setup initial PID control gains (KG=50, KP=50, KI=30, KD=10).
5. Run RC car and adjust PID control gains.

## Ordinaly usage
1. Turn on your GyroM5/M5StickC.
2. Turn on your RC car.
3. Run RC car and adjust PID control gains.


---
# Note
The detalis are below.

## Wiring
Wiring GyroM5/M5StickC with RC receiver/servo is in table below.

|M5StickC |in/out |RC Resciver/Servo |
|---- |---- |---- |
|G0  |in | Reciever CH1|
|G36 |in | Reciever CH3|
|G26 |out | Servo CH1|
|GND |in | Reciever minus|
|5Vin |in | Reciever plus|

An example image of assembled wire harness is below.

![GyroWiring](https://user-images.githubusercontent.com/64751855/117226919-aee82c00-ae50-11eb-96f3-b1b861cb95c2.jpg)

Caution:
Signal levels in M5StickC (3.3V) and RC units (5.0v or more) are generally diferent.
My RC units use 5.0-6.0v and work with directly connected M5StickC in no trouble. 
But higher volotage (over 6.0v) RC units may damage your M5StickC.

![GyroInstall](https://user-images.githubusercontent.com/64751855/117384355-b75a6880-af1d-11eb-88ad-850f1de2ef77.jpg)


## Turn On/off
M5Stick is known to have bugs in its power managment.
Find hints for trouble-shooting with google search like keyword "m5stickc not turning on".



## LCD monitor
GyroM5 has five states below.
One state transits to anothr state at botton [A],[B] or timeout event.

GyroM5は、以下の5種類の画面状態を遷移します。状態遷移は、ボタン[A]、[B]操作及びタイムアウト時に発生します。

![GyroM5](https://user-images.githubusercontent.com/64751855/117535959-70f72d80-b033-11eb-8e9c-6c60c3ccc51d.png)

- 「制御画面」がホーム状態となり、この状態からボタン[A]で「設定画面」へ、ボタン[B]で「CH3選択」へ遷移します。
- 他の状態の場合、ボタン操作が画面に表示してあるか、無操作のタイムアウトでホーム状態「制御画面」へ戻ります。
- パラメータ設定は、送信機からステアリングCH1を操作して数値を与えて、ボタン[A]で保存（ボタン[B]で取消）します。
- 起動直後の「CH3選択」は、PIDパラメータ表の参照が有効で、CH3ゲイン指定が無効です。

|画面状態|遷移条件|画面解説|
|----|----|----|
|起動画面|RC受信機の起動|ジャイロ起動後、RC受信機からPWM信号を受信するまで待機|
|較正画面|タイムアウト|CH1ニュートラル位置、IMUバイアスのサンプリング（この間、送信機操作や車体振動がNG）|
|制御画面|ボタン[A],[B]|PID制御状態（RC受信機入力、ジャイロ入力、サーボ制御量、パラメータなど）の表示|
|設定画面|ボタン[A],[B]|PID制御パラメータ、操舵CH1エンドポイントの保存（電源オフ後も残る）|
|CH3選択|ボタン[B]|CH3入力とPID制御パラメータの対応付けの選択|


## Tuning
GyroM5's control algorithm is explained for tuning parameters.

GyroM5チューニング時の参考情報として、制御アルゴリズムを解説します。

### Algorithm
GyroM5は、汎用的なフィードバック制御アルゴリズムのPID制御（下図はWikipediaから引用）を利用します。

![PID_wikipedia](https://upload.wikimedia.org/wikipedia/commons/thumb/4/43/PID_en.svg/800px-PID_en.svg.png)

PID制御における目標値r、出力値yおよび操作量uとRCカー（Plant/Process）の関係は以下のとおりです。

- 目標値: r = ch1_in = RC受信機からのCH1入力
- 出力値: y = kg*wz = RCカーの車体ヨーレート
- 操作量: u = ch1_out = RCサーボへのCH1出力

つまりRC受信機（送信機）からのCH1入力rを車体ヨーレートyの目標値と解釈して、
両者の偏差eをゼロに近づけるフィードバック制御により、サーボへのCH1出力uを自動調整します。

- 偏差: e = r - y = ch1_in - Kg*wz
- 操作量: u = PID(e) = Kp * (e + Ki * LPF(e) + Kd * HPF(e))

LPFは積分演算を模擬する「Low Pass Filter」の略称、HPFは微分演算を模擬する「High Pass Filter」の略称です。

フィードバック制御の結果、グリップ走行時はニュートラルステアに近い回頭性、ドリフト走行時はヨーレートの安定性を期待できます。


### Parameters
PID制御のパラメータ（Kg、Kp、Ki、Kd）は、走行コンディションにより調整すべきであり、LCD画面で確認＆変更できます。
PID制御の設定値（大文字）は、数値を-100〜100に規格化しており、PID制御の計算値（小文字）との関係は以下の通りです。

- 角速度"wz"は（ラジアン/秒）単位: <br>慣性センサ（IMU）計測値をセンサ感度に応じて物理量へ変換した数値です。
- 入出力"ch1"は16ビット数（0〜64k）: <br>PWMパルス幅（0ms〜20ms=1000ms/50Hz）を示す16ビット数（0〜2^16-1）です。
- 測定ゲイン: Kg = KG/0.5 <br>大きくするとヨーレートに敏感となり、ステアリング量に対するヨーレートは小さくなります。
- 比例ゲイン: Kp = KP/50.0 <br>大きくすると目標値に早く近づきますが、大きすぎるとハンチングします。
- 積分ゲイン: Ki = KI/50.0 <br>大きくすると目標値に近づくのが遅れますが、最終的な偏差を減らせます。
- 微分ゲイン: Kd = KD/50.0 <br>大きくすると目標値により早く近づきますが、大きすぎるとハンチングします。

テスト用RCカーの場合、設定値「KG=50、KP=60、KI=30、KD=10」程度でドリフト走行できました。
なお特別な設定値「KG=KI=KD=0、KP=50」は、制御なし「入力を出力へスルー：u=r」と同じです。
ステアリング用サーボを逆転モードで使う場合、ゲインKGをマイナスにすれば対応可能と思います。


### Real-time
GyroM5は、制御周波数が約50Hz、入出力処理が20ms以内のリアルタイム制御システムです。

- 「約50Hz」の理由は、PWMパルス幅計測の標準関数pulseIn(...)がブロックするので、複数CH入力時に20msを超えるからです。
- プログラム上、CH1を20ms間隔で入力する一方、CH3を500ms間隔で入力する方式で「約20ms（頻度1/25で時間超過）」を満たします。


## Test environment
作者のようにSU-01シャーシでRWDドリフト走行を試みる人は少ないと思いますが、
参考までにテスト用のRCカー、メカ及びジャイロ搭載例の写真と諸元を記します。

![UpperView](https://user-images.githubusercontent.com/64751855/117554986-370b4300-b096-11eb-9ef8-50a00980d9fc.jpg)

|項目 |型番 |
|----|----|
|シャーシ|タミヤ製SU-01|
|ボディ|タミヤ製ジムニーウイリー（SJ30）|
|タイヤ|TOPLINE製Mシャーシ用ドリフトタイヤ|
|送信機|タミヤ製ファインスペック2.4GHz|
|受信機|タミヤ製TRE-01|
|アンプ|タミヤ製TRE-01|
|サーボ|ヨコモ製S-007|
|バッテリ|7.4V LiPo 1100mAh|
|モータ|ノーマル370型DCモーター|

ドリフト走行に関連する注意点を列挙します。

![LowerView](https://user-images.githubusercontent.com/64751855/117554999-51ddb780-b096-11eb-81c1-7907ea12db07.jpg)

- シャーシに関しては、ステアリング用ナックルとシャーシの干渉部分を削りステアリング角度を45度ぐらいまで増やしました。
- サーボに関しては、ファインスペック付属のTSU-03だと制御が遅れてハンチングしたので、ある程度の高速なサーボが必要です。
- モータに関しては、ノーマルだとLiPoバッテリと組み合わせないと、スピードが出たときにトルク不足でドリフト移行が難しいです。
- タイヤに関しては、駆動系が非力なので、なるべく滑りやすいタイヤが良いです。


# Roadmap

RCカー用ジャイロ自作を通して、気付いた改良アイデアなどを列挙します。
いずれ対応したいと思いますが、趣味で開発しているので、いつ対応できるか分かりません。
ご自身で改良にチャレンジすれる際の参考になればと思います。

- パラメータ設定のスマホ対応　<br>スマホのGUI画面からジャイロ設定（PIDゲイン等）を複数管理して変更可能とする。
- パラメータ調整の完全自動化 <br>車体、路面やタイヤに応じたPIDゲインの最適化を強化学習などで完全自動化する。
- スロットル制御のアシスト <br>ドリフト走行の安定化には、ステアリングとスロットルの同時制御が必要です。
- 加速度センサの有効利用 <br>ヨーレートと水平加速度から車体スリップ角を推定してトラクション制御を高度化する。
- ジャイロ固定方向の自動検出 <br>鉛直方向を起動時に自動検出して車体ヨーレート成分を決定する。
- PWM入力方式の改良 <br>PWM入力にブロック方式の関数pulseIn(...)を廃止して割り込み方式へと変更する。
- 外部電源との完全連動 <br>M5StickCの内蔵バッテリーを無効化して、RCアンプBECの給電のみでオン/オフ動作させる。
- 走行データの記録分析 <br>走行データをSDカード等に記録して事後分析できるようにする（M5StickCからM5Stackへ変更？）。

M5StickCは、WiFi/Bluetoothを備える点、外部GPIOが5本ある点、6軸IMUを備える点、割り込み処理できる点から、
ほとんどの改良案はハードウェア的には実現可能と思いますので、あとはソフトウェアつまりアイデア次第だと思います。


# Author

作者は、コロナ禍で屋内遊びをさがす中、初代グラスホッパー（笑）以来めっちゃ久しぶりにRCカーキット（小型のタミヤSU-01シャーシ）を購入しました。
購入後、RCカー系YouTubeチャンネルを見て、子ども時代に存在しなかったドリフト用RCカー（通称、ドリラジ）の動きに興味を持ちました。
ツルツルのタイヤで横滑りさせながらRCカーを走らせるアレです。
ドリフト用RCカーは、ジャンルとして確立しており、たとえばヨコモYD-2のように専用設計で完成度の高い製品が存在します。
純粋にRCカーのドリフト走行を楽しみたければ、ドリフト専用のシャーシやジャイロ製品を入手するのが最短コースです。

自分の場合、ドリラジの存在に気付いたのがRCカーキット購入後だったこと、
どんなRCカーでも上手く制御できればドリフト走行（を安定化）可能と信じていたことから、
タミヤ最安（実売価格≒4K円）のSU-01シャーシを自作ジャイロで制御してドリフト走行にチャレンジしました。
やや回り道しましたが、ほぼノーマル（シャーシを削り舵角を増やすだけ）のSU-01シャーシでドリフト走行できました。

RCカー用ジャイロの自作は、プログラムやパラメータの変更によりRCカーの走行特性を変えられるので楽しい開発でした。
RCカー好きの人なら自作ジャイロの操縦性を楽しみつつ、プログラミングや制御アルゴリズムを習得する良い素材（STEM教育の素材）と思います。
趣味でRCカーやプログラミングを楽しむ若い人が増えて欲しいとの願いから、開発したRCカー用ジャイロGyroM5のソースコードを公開します。
この記事を参考に、部品を集めてGyroM5を再現する人、改造して「オレ専用ジャイロ」を開発する人、が出てくれば自分はハッピーです。


---

# Reference


## Hobby RC Car
- [Yokomo YD-2](https://teamyokomo.com/product/dp-yd2/)
- [Tamiya SU-01](https://www.tamiya.com/japan/products/product_info_ex.html?genre_item=7101)
- [RC Car Shop "Genkikko-san"](https://genkikkosan.com/)

## Automobile Drifting
- [自動車の運動と制御](https://www.amazon.co.jp/dp/4501419202/)
- [車両運動の安定性解析と制御への応用](https://www.tytlabs.com/japanese/review/rev321pdf/321_013ono.pdf)
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


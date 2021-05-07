# R/Cカー用ジャイロ"GyroPID"

---

高機能なR/Cカー用ジャイロをM5StickCマイコン（実売価格3K円）で自作する方法を紹介します。
M5StickCは、ヨーレート（回頭角速度）計測用のIMU（慣性計測ユニット）、サーボ制御用のハードウェアPWM（パルス幅変調）、パラメータ調整用のLCD（液晶ディスプレイ）を備えており、R/Cカー用ジャイロ自作に最適です。
プログラム"GyroPID.ino"をArduinoIDEでコンパイル＆転送してR/C受信機及びサーボと接続するだけで、M5StickC単体でR/Cカー用ジャイロとして機能します。

![ラジコン画像](https://user-images.githubusercontent.com/64751855/117384511-1d46f000-af1e-11eb-854e-45ee149e4671.jpg)


# 動機など

自分は、コロナ禍で屋内遊びを探す中で、小学生の頃から約30年ぶりにタミヤ製R/Cカーキット（小型のSU-01シャーシ）を購入しました。
購入後YouTubeのR/Cカー関連チャンネルを観るうちに、子供時代に存在しなかったドリフト用R/Cカー（通称、ドリラジ）の面白い動きに興味を持ちました。
ドリフト用R/Cカーは、すでにジャンルとして確立しており、たとえばヨコモ製YD-2のように専用シャーシやジャイロなど完成度の高い製品群が存在します。
純粋にR/Cカーのドリフト走行を楽しみたければ、ドリフト専用のシャーシやジャイロの製品を入手するのが最短コースだと思います。

自分の場合、ドリラジの存在に気付いたのがR/Cカーキット購入後だったこと、
どのR/Cカーでも上手く制御できればドリフト走行（を安定化）可能と思い込んだことから、
タミヤ最安のSU-01シャーシ（実売価格4K円）を自作ジャイロで制御してドリフト走行の安定化に挑戦しました。
やや回り道しましたが結果的に、舵角を増やす改造だけのSU-01シャーシでドリフト走行の安定化に成功しました。

R/Cカー用ジャイロの自作は、プログラムやパラメータの変更によりR/Cカーの走行特性を変えられるので楽しい開発でした。
おそらくR/Cカー好きの人なら自作ジャイロの操縦性を楽しみつつ、プログラミングや制御の仕組みを体験して理解する良い素材になると思います。
趣味でR/Cカーやプログラミングを楽しむ人が増えて欲しいとの思いから、RCカー用ジャイロGyroPIDのソースコードを公開します。
この記事を参考に、部品を揃えてGyroPIDを再現する人、改造して「オレ専用ジャイロ」を開発する人、が出てくると自分はハッピーです。


# 特長など

このR/Cカー用ジャイロGyroPIDは、低価格のジャイロ製品に見られない以下の特長を備えています。

- PID自動制御 <br>車体ヨーレート（回頭角速度）を目標としてステアリング角を自動制御する。
- PID設定保存 <br>ジャイロ制御パラメータ（PIDゲインなど）を自由に変更して保存できる。
- CH1範囲保存 <br>ステアリング角のエンドポイントを保存して自動制御時の制約として守る。
- IMU自動較正 <br>ジャイロ起動時にIMU（慣性計測ユニット）のバイアスを自動較正する。
- PID制御表示 <br>ジャイロ状態（受信機入力、IMU入力、PID設定値、サーボ出力）をLCD表示する。
- ドリフト検出 <br>ドリフト走行（回頭方向と操舵方向が逆）を自動検出してLCD背景色を変更する。
- オープンソース <br>ソースコードが公開されていて、誰でも理解して自由に改造できる。


# 使い方

R/Cカー用ジャイロGyroPIDの使い方（本体準備、初期設定から通常利用の流れ）を概説します。
詳しいR/Cメカとの接続方法、M5StickCの起動方法、PID制御パラメータの解釈は後半を参照ください。

## 本体準備
1. 手持ちのコンピュータにArduinoIDE（開発環境）をインストールする
2. コンピュータとM5StickCをUSBケーブルで接続してプログラム書き込みを確認する
3. GyroPIDプログラム"GyroPID.ino"をArduinoIDEからM5StickCへ書き込む
4. M5StickCの電源をオンしてR/Cメカと接続して動作確認する
5. GyroPIDをR/Cカーのシャーシに固定する（本体LCD画面が鉛直上向き）

## 初期設定
1. GyroPIDを起動する（起動しない場合、USB給電等で起動）
2. R/Cメカを起動する（R/Cメカ起動後、USB給電を外す）
3. GyroPIDの操舵エンドポイントを設定する
4. GyroPIDのPID制御パラメータを設定する（初期値：KG=50、KP=50、KI=30、KD=10）
5. R/Cカーを走らせて、必要によりPID制御パラメータを調整する

## 通常利用
1. GyroPIDを起動する（起動しない場合、USB給電等で起動）
2. R/Cメカを起動する（R/Cメカ起動後、USB給電を外す）
3. R/Cカーを走らせて、必要によりPID制御パラメータを調整する



# 制御方法

GyroPIDは、
ステアリング指示値"ch1_in"をヨーレート計測値"omega"の目標と解釈して、
両者の残差"error:=ch1_in-kg*omega"をゼロに近づけるPID制御により、
ステアリング指令値"ch1_out:=PID(error)"を自動調整します。

- error := ch1_in - kg * omega
- ch1_out := PID(error) = kp * (error + ki * LPF(error) + kd * HPF(error))

PIDは制御ルール「比例Proportional、積分Integral、微分Differential」の略称、
LPFは積分器を模擬する「Low Pass Filter」の略称、HPFは微分器を模擬する「High Pass Filter」の略称です。

つまりジャイロは、R/C送信機からのステアリング指示値"ch1_in"を「ステアリング角度"ch1_out"の目標」ではなく「車体ヨーレート"omega"の目標」と解釈します。
このジャイロが期待通りに車体ヨーレートを制御できた場合、
グリップ走行時のニュートラルに近い回頭性、ドリフト走行時のヨーレートの安定性が高まるので、
理屈の上ではR/Cカーが操縦しやすくなると思います（笑）。

PID制御パラメータ（kg、kp、ki、kd）は、タイヤ、車体や路面の状況により調整すべきであり、ジャイロ画面で確認＆変更できます。
テスト用R/Cカーの場合、PID制御の設定パラメータはKG=50、KP=60、KI=30、KD=10ぐらいの数値でドリフト走行が安定しました。

PID制御パラメータ（大文字の設定値）は、数値0〜100程度に規格化しており、PID制御パラメータ（小文字の計算値）と以下の関係です。

- 角速度"omega"は（ラジアン/秒）単位 <br>慣性センサ（IMU）計測値をIMU感度に応じて物理量へ変換した数値です。
- 入出力"ch1"は16ビット数（0〜65535） <br>PWMパルス幅（0ms〜20ms=1000ms/50Hz）を示す16ビット数（0〜2^16-1）です。
- 目標ゲイン <br>kg = KG/0.5
- 比例ゲイン <br>kp = KP/50.0
- 積分ゲイン <br>ki = KI/50.0
- 微分ゲイン <br>kd = KD/50.0

ジャイロの制御周期は「概ね20ms（約50Hz）」であり、入力から出力までの遅延もR/Cサーボへの指令間隔と同じ20ms以内です。
「概ね20ms」の理由は、PWM幅計測に用いた標準関数pulseIn(...)がブロックするので、複数CHのシリアル入力時に20msを超えるからです。
プログラム上、CH1を20ms間隔で入力する一方、CH3を500ms間隔で入力する方式で「概ね20ms（ただし頻度1/25で時間オーバー）」を満たします。


# 画面表示

R/Cカー用ジャイロGyroPIDは、以下の5種類の画面状態を遷移します。状態遷移は、ボタン[A]、[B]操作及びタイムアウト時に発生します。
「制御画面」がホーム状態となり、この状態からボタン[A]で「設定画面」へ、ボタン[B]で「CH3選択」へ遷移します。
他の状態の場合、ボタン操作が画面に表示してあるか、無操作のタイムアウトでデフォルト状態「制御画面」へ戻ります。
パラメータ設定は、送信機からステアリングCH1を操作して数値を与えて、ボタン[A]で保存（ボタン[B]で取消）します。
起動直後の「CH3選択」は、PIDパラメータ表の参照が有効で、CH3ゲイン指定が無効です。

![GyroPID](https://user-images.githubusercontent.com/64751855/117384421-df49cc00-af1d-11eb-9d54-8f4c3f73440f.gif)

|画面状態|遷移条件|解説|
|----|----|----|
|起動画面|R/C受信機の起動|ジャイロ起動後、R/C受信機からPWM信号を受信するまで待機|
|較正画面|タイムアウトTO|CH1ニュートラル位置、IMUバイアスをサンプリングで決定（送信機操作、車体運搬はNG）|
|制御画面|ボタン[A],[B]|PID制御中の画面、R/C受信機入力、ジャイロ入力、サーボ制御量、PID制御パラメータを表示|
|設定画面|ボタン[A],[B]|PID制御パラメータ、操舵CH1サーボのエンドポイントを設定（電源オフ後も残る）|
|CH3選択|ボタン[B]|CH3入力とPID制御パラメータの対応付けを選択|


# 接続方法

GyroPID利用時のM5StickCマイコンのGPIO端子とR/Cメカ類（受信機、サーボ）との接続方法は下記の通りです。

|GPIO |in/out |R/Cメカ端子 |
|---- |---- |---- |
|G0  |in | R/C受信機CH1のシグナル端子|
|G36 |in | R/C受信機CH3のシグナル端子|
|G26 |out | R/CサーボCH1のシグナル端子|
|GND |in/out | R/CアンプBECのマイナス端子|
|5Vin |in | R/CアンプBECのプラス端子|

ワイヤハーネスは、CH1入出力用にR/Cサーボ用のコネクタ付き延長ケーブル1本を中央で切断して、8ピンヘッダ（オス）とハンダ付けすれば完成です。
ゲイン調整にCH3入力を利用する場合、同様にコネクタ付き延長ケーブルを用いるか、信号線（単線）のみCH3とG32を接続すれば機能します。
なおCH3をジャイロに接続していない場合、ジャイロはPID制御のパラメータ表を参照します。

![ジャイロ配線](https://user-images.githubusercontent.com/64751855/117226919-aee82c00-ae50-11eb-96f3-b1b861cb95c2.jpg)

信号の電圧レベルに関しては、M5StickC側が3.3Vなのに対して、R/Cメカ側が通常5.0V以上と高くなる点に十分ご注意ください。
テスト環境（タミヤ製TRE-01、HobbyWing製QuicRun-1060）では、直結で問題なく動いていますが、おそらく許容範囲内でも保証範囲外だと思います。
たとえば走行用バッテリを高電圧サーボに直結している場合、電圧レベルコンバータを省略するとM5StickCが破損（不可逆に故障）する可能性が高いです。

![ジャイロ搭載](https://user-images.githubusercontent.com/64751855/117384355-b75a6880-af1d-11eb-88ad-850f1de2ef77.jpg)


# 電源管理

M5StickCは、電源管理に不具合が残っており、電源ボタンを押しても、素直に電源オンしない場合があります。
原因は、内蔵バッテリの電圧低下、GPIOピンの電圧レベル、電源管理チップAXPの不具合等に起因するそうです。
たとえばキーワード「M5StickC　起動しない」のGoogle検索で、いくつかの対処方法が見つかりますので、
電源まわりのトラブル解決の参考にしてください。

自分のテスト環境でも、電源オンする場合/しない場合が起きますが、M5StickCから配線を外してUSB給電すると高確率で起動に成功します。
M5StickC関連ブログ記事に「G0と3V3を直結してUSB給電すれば起動」や「電源投入時のG0電圧に応じてリセット動作」等の記載があります。
内蔵バッテリの充電不足を除いた場合、おそらく電源ボタンを押したタイミングのGPIOピン電圧によりM5StickC起動の成否が決まるようです。

なおジャイロ起動後、電源オフの操作は不要です。
ジャイロのプログラムが、5Vin端子からBEC電圧の低下を検出するとM5StickCの電源を自発的にオフします。


# テスト環境

自分のようにSU-01シャーシでRWDドリフトを試みる人は少ないと思いますが、
参考までにテスト用のR/Cカー、メカ及びジャイロ搭載例の写真と諸元を記します。

![シャーシ画像](https://user-images.githubusercontent.com/64751855/117116225-1064b800-adc9-11eb-898f-fba45874e475.jpg)

|項目 |型番 |
|----|----|
|シャーシ|タミヤ製SU-01|
|ボディ|タミヤ製ジムニーウイリー（SJ30）|
|タイヤ|タミヤ製スーパードリフトタイヤ|
|送信機|タミヤ製ファインスペック2.4GHz|
|受信機|タミヤ製TRE-01|
|アンプ|タミヤ製TRE-01|
|サーボ|ヨコモ製S-007|
|バッテリ|7.4V LiPo 1100mAh|
|モータ|ノーマル370型DCモーター|

サーボに関しては、ファインスペック付属のTSU-03だと制御が遅れてハンチングしたので、ある程度の高速なサーボが必須だと思います。
モータに関しては、ノーマルだとLiPoバッテリと組み合わせないと、スピードが出たときにトルク不足でドリフト移行が難しいと思います。
タイヤに関しては、駆動系が非力なので、滑りやすいタイヤが良いと思います。
この他の項目に関しては、いかなる組み合わせでも大丈夫と思います。


# 改良案

R/Cカー用ジャイロ自作を通して、気付いた改良アイデアなどを列挙します。
いずれ対応したいと思いますが、趣味で開発しているので、いつ対応できるか自分でも分かりません。
ご自身で改良にチャレンジする場合の参考になれば幸いです。

- パラメータ設定のスマホ対応　<br>スマホのGUI画面からジャイロ設定（PIDゲイン等）を複数管理して変更可能とする。
- パラメータ調整の完全自動化 <br>車体、路面やタイヤに応じたPIDゲインの最適化を強化学習などで完全自動化する。
- スロットル制御のアシスト <br>ドリフト走行の安定化には、ステアリングとスロットルの同時制御が必要です。
- 加速度センサの有効活用 <br>ヨーレートと水平加速度から車体スリップ角を推定してトラクション制御を高度化する。
- ジャイロ固定方向の自由化 <br>鉛直方向を起動時に自動検出して車体ヨーレート成分を決定する。
- PWM入力方式の改良 <br>PWM入力にブロック方式の関数pulseIn()を廃止して割り込み方式へと変更する。
- 外部電源との連動 <br>M5StickCの内蔵バッテリーを無効化して、R/CアンプBECの給電のみでオン/オフ動作させる。
- 走行データの記録 <br>走行データをSDカード等に記録して事後分析できるようにする（M5StickCからM5Stackへ変更？）。

M5StickCは、WiFi/Bluetooth機能を備える点、外部GPIOが5本ある点、6軸IMU機能を備える点、割り込み処理できる点から、
ほとんどの改良案はハードウェア的には実現可能と思いますので、あとはアイデアとソフトウェア次第だと思います。


# 参考資料

R/Cカー用ジャイロGyroPIDの開発にあたり、参考にした資料などを列挙します。
「元気っ子さん」は、ヨコモYD-2の「レンタカー」サービスを提供中で、GyroPID搭載R/Cカーのテスト走行でお世話になっています。

## R/Cカー関連
- [ヨコモ](https://teamyokomo.com/)
- [タミヤ](https://www.tamiya.com/japan/rc/index.html)
- [R/Cカー練習場「元気っ子さん」](https://genkikkosan.com/)

## ドリフト走行
- [On the dynamics of automobile drifting](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.103.9227&rep=rep1&type=pdf)
- [Analysis and control of high sideslip manoeuvres](https://www.tandfonline.com/doi/abs/10.1080/00423111003746140?journalCode=nvsd20)
- [Stabilization of steady-state drifting for a RWD vehicle](http://dcsl.gatech.edu/papers/avec10.pdf)

## ソフトウェア関連
- [PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [Arduino IDE](https://www.arduino.cc/en/software)

## ハードウェア関連
- [M5StickC非公式日本語リファレンス](https://lang-ship.com/reference/unofficial/M5StickC/)
- [M5Stack公式ドキュメント](https://github.com/m5stack/m5-docs/blob/master/docs/ja/README_ja.md)
- [M5StickC本体例](https://www.switch-science.com/catalog/5517/)
- [ピンヘッダ（オス）例](https://www.amazon.co.jp/dp/B012HY288S/)
- [サーボ延長ケーブル例](https://www.amazon.co.jp/dp/B00W9ST610/)


以上

[日本語](README.md) | [English](README-en.md)

# GyroM5

- GyroM5は、ドリフトRCカーのステアリングジャイロをM5StickCで自作するためのOSSです。
- 本格的なPID制御アルゴリズム採用により、ステアリングのアシスト機能を高い自由度で設定できます。
- M5StickCにv1スケッチ[GyroM5.ino](GyroM5/GyroM5.ino)をインストールしてRCユニットと接続すれば完成です。
- 新型v2スケッチ[GyroM5v2.ino](GyroM5v2/GyroM5v2.ino)は、スマートフォン連携、サーボ周波数可変等に対応です。

![ラジコン画像](https://user-images.githubusercontent.com/64751855/117384511-1d46f000-af1e-11eb-854e-45ee149e4671.jpg)


---


# DEMO
GyroM5搭載のタミヤ製RCカー（SU-01シャーシ）がRWDドリフト走行するデモ動画です。

<video width="320" height="240" muted controls>
  <source type="video/mp4" src="https://user-images.githubusercontent.com/64751855/117535983-a1d76280-b033-11eb-9f59-ec6aaef0b9b0.mp4">
</video>

https://user-images.githubusercontent.com/64751855/117535983-a1d76280-b033-11eb-9f59-ec6aaef0b9b0.mp4


# Features

GyroM5（v1/v2スケッチ）は、機能面でユニークな特徴を備えています。
"PID制御"は、汎用性の高いフィードバック制御アルゴリズムであり、パラメータ4個の調整により高い自由度でセッティングできます。

- フィードバック制御機能 <br>車体ヨーレートを目標に操舵角をフィードバック制御（PID制御）します。
- 制御パラメータ設定機能 <br>フィードバック制御のパラメータをCH1から設定して保存できます。
- リモートゲイン調整機能 <br>フィードバック制御のパラメータをCH3からリモート調整できます。
- エンドポイント設定機能 <br>ステアリング角度のエンドポイントを設定して保存できます。
- 慣性センサ自動較正機能 <br>ジャイロ起動時に慣性センサ（IMU）のバイアスを自動較正します。
- 状態ディスプレイ機能 <br>稼動状態（RCユニット入出力、IMU入力、PID設定値）をLCD表示します。
- ドリフト走行検出機能 <br>ドリフト走行（回頭方向と操舵方向が逆）を検出してLCD背景色を変更します。

新型v2スケッチの追加機能は、次の通りです。
M5StickCの能力（WiFi通信、６軸IMU等）をv1スケッチより引き出して、RCカーをより楽しくするファームウェアです。

- スマホ連携機能 <br>WiFi通信によりスマートフォン経由で各種設定ができます。
- サーボPWM可変機能 <br>サーボPWM周波数を50-400Hzの範囲で変更できます。
- グラフ表示機能 <br>PID制御データ（目標値、入出力）をグラフ表示します。
- データ保存機能 <br>WiFi通信によりPID制御データをダウンロードできます。
- 鉛直検出機能 <br>起動時に鉛直方向を自動検出できシャーシ固定が自由です。
- 時刻表示機能 <br>現在時刻をLCD画面に表示します。



# Requirement
GyroM5の利用に必要なハードウェアを列挙します。

- ホビー用RCカー　<br>標準的なRCユニットを搭載して、受信機とステアリング用サーボを3線（-+S）で接続するRCカーです。
- 標準的パソコン <br>Arduino IDEをインストールでき、USBインターフェイスを備えるスケッチ書き込み用パソコンです。
- M5StickC <br>LCD解像度の低い方が"M5StickC"です。高い方の"M5StickC Plus"ではありません。
- ワイヤハーネス部品 <br>サーボ延長ケーブルを1本、オス型ピンヘッダを1個（8ピン以上）使います。
- ハンダ付け機材 <br>ワイヤハーネスの組み立てに使います。

M5StickCやワイヤハーネス部品は、Amazon等のネット通販で入手可能です。


# Usage
GyroM5の使い方（本体準備、初期設定から通常利用の流れ）を概説します。
RCユニットとの接続方法、M5StickCの起動方法、制御パラメータの調整方法は後半を参照ください。

## 本体準備
1. 手持ちのパソコンにArduino IDE（開発環境）をインストールする
2. Arduino IDEの開発ボード設定をESP32/M5StickC向けに変更する
3. パソコンとM5StickC開発ボードをUSBケーブルで接続する
4. スケッチ[GyroM5.ino](GyroM5/GyroM5.ino)をArduino IDE経由でM5StickCへ書き込む
6. GyroM5（M5StickC）をRCカーに固定（LCD画面が上向き）してRCユニットと接続する

## 初期設定
1. GyroM5を起動する（起動しない場合、USB給電等で起動）
2. RCユニットを起動する（RCユニット起動後、USB給電を外す）
3. GyroM5の操舵エンドポイントを設定する
4. GyroM5のPID制御パラメータを設定する（初期値：KG=50、KP=50、KI=30、KD=10）
5. RCカーを走らせて、必要によりPID制御パラメータを微調整する

## 通常利用
1. GyroM5を起動する（起動しない場合、USB給電等で起動）
2. RCユニットを起動する（RCユニット起動後、USB給電を外す）
3. RCカーを走らせて、必要によりPID制御パラメータを微調整する


---
# Note
GyroM5の詳しい解説を記します。

## 接続方法

GyroM5利用時のM5StickCのGPIO端子とRCユニット（受信機、サーボ）端子との接続方法は下記の通りです。

|M5StickC |in/out |RCユニット |
|---- |---- |---- |
|G26  |in | RC受信機CH1のシグナル端子|
|G36 |in | RC受信機CH3のシグナル端子|
|G0 |out | RCサーボCH1のシグナル端子|
|GND |in | RCアンプBECのマイナス端子|
|5Vin |in | RCアンプBECのプラス端子|

ワイヤハーネス（接続ケーブル）は、CH1入出力用にRCサーボ用のコネクタ付き延長ケーブル1本を中央で切断して、8ピンヘッダ（オス）とハンダ付けすれば完成です。

![ワイヤハーネス](https://user-images.githubusercontent.com/64751855/119204830-865b6580-bad1-11eb-9ab6-f055a49f4d88.jpg)

ゲイン調整用にCH3入力を利用する場合、信号線（単線）のみ受信機CH3とG32を接続すれば機能します。
なおCH3をジャイロに接続しない場合、ジャイロはPID制御の静的なパラメータ表のみ参照します。

![ジャイロ搭載](https://user-images.githubusercontent.com/64751855/117384355-b75a6880-af1d-11eb-88ad-850f1de2ef77.jpg)

信号の電圧レベルに関しては、M5StickC側が3.3Vなのに対して、RCユニット側が通常5.0V以上と高くなる点に十分ご注意ください。
テスト環境（タミヤ製TRE-01、HobbyWing製QuicRUN-1060）では、直結で問題なく動いていますが、許容範囲内でも保証範囲外と思います。
たとえばRCユニット側の電圧レベルが6Vを超える場合、レベルコンバータを省略するとM5StickCが破損（不可逆に故障）する恐れがあります。

M5StickCのGPIO端子は、プログラムにより自由に変更できますが、G0端子を入力用に使うことは避けたほうが良いです。
G0端子は、内部的にプルアップされており、電源投入時にG0端子がLowレベルだとM5StickCが起動しないことがありました。
G0端子が出力用の場合、相手側の端子が入力用のハイインピーダンスとなるので、この問題を回避できるようです。


## 起動方法

M5StickCは、電源管理に不具合が残っており、電源ボタンを押しても、素直に起動しない場合があります。
原因は、電源管理チップAXPの不具合、内蔵バッテリの電圧低下、GPIOピンの電圧レベル等に起因するそうです。
たとえばキーワード「M5StickC　起動しない」のGoogle検索で、複数の対処方法が見つかりますので、トラブル解決の参考にしてください。

自分のテスト環境でも、起動に成功/失敗する場合が起きますが、M5StickCのワイヤハーネスを取り外してUSB給電すると高確率で起動します。
M5StickC関連ブログ記事に「G0と3V3を直結してUSB給電すれば起動」や「電源投入時のG0電圧に応じてリセット動作」等の記載があります。
内蔵バッテリの充電不足を除いた場合、電源投入時点のGPIOピン（おそらくはG0ピンの）電圧によりM5StickC起動の成否が決まるようです。

なおジャイロ起動後、電源オフ操作は不要です。
GyroM5プログラムは、5Vin端子でBEC電圧の低下を検出すると電源をオフします。


## 画面遷移v1
GyroM5v1は、5種類の画面状態を遷移します。状態遷移は、ボタン[A]、[B]操作及びタイムアウト時に発生します。

![GyroM5v1](https://user-images.githubusercontent.com/64751855/124378819-7ea4f880-dcee-11eb-800e-6e4c70756847.png)

- 「HOME」がホーム状態となり、この状態からボタン[A]で「CONF」へ、ボタン[B]で「CH3G」へ遷移します。
- 他の状態の場合、ボタン操作が画面に表示してあるか、無操作のタイムアウトでホーム状態「HOME」へ戻ります。
- パラメータ設定は、送信機からステアリングCH1を操作して数値を与えて、ボタン[A]で保存（ボタン[B]で取消）します。
- 起動直後の「CH3G」は、PIDパラメータ表の参照が有効で、CH3ゲイン指定が無効です。

|画面|遷移|解説|
|----|----|----|
|WAIT|RC受信機の起動|ジャイロ起動後、RC受信機からPWM信号を受信するまで待機|
|INIT|タイムアウト|CH1ニュートラル位置、IMUバイアスのサンプリング（この間、送信機操作や車体振動がNG）|
|HOME|ボタン[A],[B]|PID制御状態（RCユニット入出力、IMU入力、PIDパラメータなど）の表示|
|CONF|ボタン[A],[B]|PID制御パラメータ、操舵CH1エンドポイントの保存（電源オフ後も残る）|
|CH3G|ボタン[B]|CH3入力とPID制御パラメータの対応付けの選択|


## 画面遷移v2
GyroM5v2は、5種類の画面状態を遷移します。状態遷移は、ボタン[A]、[B]操作及びタイムアウト時に発生します。

![GyroM5v2](https://user-images.githubusercontent.com/64751855/124378834-90869b80-dcee-11eb-9067-8011bbfd12fb.png)

- 「HOME」がホーム状態となり、この状態からボタン[A]で「WIFI」へ、ボタン[B]で「ENDS」へ遷移します。
- 他の状態の場合、ボタン操作が画面に表示してあるか、無操作のタイムアウトでホーム状態「制御画面」へ戻ります。
- 「WIFI」状態は、WIFIアクセスポイントとなり、スマートフォン等からQRコード読み取りWiFi接続できます。

|画面|遷移|解説|
|----|----|----|
|WAIT|RC受信機の起動|ジャイロ起動後、RC受信機からPWM信号を受信するまで待機|
|INIT|タイムアウト|CH1ニュートラル位置、IMUバイアスのサンプリング（この間、送信機操作や車体振動がNG）|
|HOME|ボタン[A],[B]|PID制御状態（RCユニット入出力、IMU入力、PIDパラメータなど）の表示|
|WIFI|ボタン[A]|WIFIアクセスポイントとなりスマートフォン等経由で接続して設定|
|ENDS|ボタン[B]|CH1エンドポイントの設定|

WiFi機能を利用する場合、GyroM5のボタン[A]を押してください。GyroM5は、WIFIモードに入るとWiFiアクセスポイントとして機能します。
GyroM5のWiFiアクセスポイントへ接続後、LCD画面のテキストまたはQRコードで指定されるURLを開くと以下の画面が表示されます。
なおWiFi接続後、ルーティングに失敗して画面が表示されない場合は「モバイルデータ通信をオフ」にしてください。
この画面から各パラメータの設定機能、走行データのダウンロード機能を利用できます。
パラメータの設定後、GyroM5は通常のPID制御モード「HOME」へ自動復帰します。

![SettingByWiFi](https://user-images.githubusercontent.com/64751855/124377656-f6bbf000-dce7-11eb-93ab-0ea7cc6a0294.png)


## 調整方法
GyroM5チューニング時の参考情報として、制御アルゴリズムを解説します。

### 制御アルゴリズム
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


### 制御パラメータ
PID制御のパラメータ（Kg、Kp、Ki、Kd）は、走行コンディションにより調整すべきであり、LCD画面で確認＆変更できます。
PID制御の整数値（大文字）は、数値を-100〜100に規格化しており、PID制御の実数値（小文字）との関係は以下の通りです。

- 角速度"wz"は（ラジアン/秒）単位: <br>慣性センサ（IMU）計測値をセンサ感度に応じて物理量へ変換した数値です。
- 入出力"ch1"は16ビット数（0〜64k）: <br>PWMパルス幅（0ms〜20ms=1000ms/50Hz）を示す16ビット数（0〜2^16-1）です。
- 観測ゲイン: Kg = KG/0.5 <br>大きくするとヨーレートに敏感となり、ステアリング量に対する目標ヨーレートは小さくなります。
- 比例ゲイン: Kp = KP/50.0 <br>大きくするとカウンタステア量が多くなりますが、大きすぎるとハンチングします。
- 積分ゲイン: Ki = KI/50.0 <br>大きくするとカウンタステア応答が遅くなりますが、最終的な偏差を減らせます。
- 微分ゲイン: Kd = KD/50.0 <br>大きくするとカウンタステア応答が早くなりますが、大きすぎるとハンチングします。

テスト用RCカーの場合、設定値「KG=50、KP=60、KI=30、KD=10」程度でドリフト走行できました。
なお特別な設定値「KG=KI=KD=0、KP=50」は、制御なし「入力を出力へスルー：u=r」と同じです。
ステアリング用サーボを逆転モードで使う場合、ゲインKGをマイナスにすれば対応可能と思います。


### リアルタイム性
GyroM5は、制御周波数が約50Hz、入出力処理が20ms以内のリアルタイム制御システムです。

- 「約50Hz」の理由は、PWMパルス幅計測の標準関数pulseIn(...)がブロックするので、複数CH入力時に20msを超えるからです。
- プログラム上、CH1を20ms間隔で入力する一方、CH3を500ms間隔で入力する方式で「約20ms（頻度1/25で時間超過）」を満たします。


## 試験環境

作者のようにSU-01シャーシでRWDドリフト走行を試みる人は少ないと思いますが、
参考までにテスト用RCカー、ユニット及びジャイロ搭載例の写真と諸元を記します。

![シャーシ表側](https://user-images.githubusercontent.com/64751855/117554986-370b4300-b096-11eb-9ef8-50a00980d9fc.jpg)

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

![シャーシ裏側](https://user-images.githubusercontent.com/64751855/117554999-51ddb780-b096-11eb-81c1-7907ea12db07.jpg)

- シャーシに関しては、ステアリング用ナックルとシャーシの干渉部分を削りステアリング角度を45度ぐらいまで増やしました。
- サーボに関しては、ファインスペック付属のTSU-03だと制御が遅れてハンチングしたので、ある程度の高速なサーボが必要です。
- モータに関しては、ノーマルだとLiPoバッテリと組み合わせないと、スピードが出たときにトルク不足でドリフト移行が難しいです。
- タイヤに関しては、駆動系が非力なので、なるべく滑りやすいタイヤが良いです。


# Roadmap

RCカー用ジャイロ自作を通して、気付いた改良アイデアなどを列挙します。
いずれ対応したいと思いますが、趣味で開発しているので、いつ対応できるか分かりません。
ご自身で改良にチャレンジすれる際の参考になれば幸いです。

- パラメータ設定のスマホ対応（v2対応）　<br>スマホのGUI画面からジャイロ設定（PIDゲイン等）を複数管理して変更可能とする。
- パラメータ調整の完全自動化 <br>車体、路面やタイヤに応じたPIDゲインの最適化を強化学習などで完全自動化する。
- スロットル制御のアシスト <br>ドリフト走行の安定化には、ステアリングとスロットルの同時制御が必要です。
- 加速度センサの有効利用 <br>ヨーレートと水平加速度から車体スリップ角を推定してトラクション制御を高度化する。
- ジャイロ固定方向の自動検出（v2対応） <br>鉛直方向を起動時に自動検出して車体ヨーレート成分を決定する。
- PWM入力方式の改良（v2対応） <br>PWM入力にブロック方式の関数pulseIn(...)を廃止して割り込み方式へと変更する。
- 外部電源との完全連動（v2対応） <br>M5StickCの内蔵バッテリーを無効化して、RCアンプBECの給電のみでオン/オフ動作させる。
- 走行データの記録分析（v2対応） <br>走行データをSDカード等に記録して事後分析できるようにする（M5StickCからM5Stackへ変更？）。
- サーボ周波数可変機能（v2対応） <br>サーボPWM周波数をパラメータで変更可能とする。

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
やや回り道しましたが、ほぼノーマル（舵角を増やしただけ）のSU-01シャーシでドリフト走行できました。

RCカー用ジャイロの自作は、プログラムやパラメータの変更によりRCカーの走行特性を変えられるので楽しい開発でした。
RCカー好きの人なら自作ジャイロの操縦性を楽しみつつ、プログラミングや制御アルゴリズムを習得する良い素材（STEM教育の素材）と思います。
趣味でRCカーやプログラミングを楽しむ若い人が増えて欲しいとの願いから、開発したRCカー用ジャイロGyroM5のソースコードを公開します。
この記事を参考に、部品を集めてGyroM5を再現する人、改造して「オレ専用ジャイロ」を開発する人、が出てくれば自分はハッピーです。

(^_^)


---

# Reference

RCカー用ジャイロGyroM5の開発にあたり、参考にした資料などを列挙します。
「元気っ子さん」は、初心者に親切なラジコン屋さんで、作者がGyroM5搭載カーの試験走行、ヨコモYD-2の体験走行等でお世話になっています。

## ホビー用RCカー関連
- [ヨコモYD-2](https://teamyokomo.com/product/dp-yd2/)
- [タミヤSU-01](https://www.tamiya.com/japan/products/product_info_ex.html?genre_item=7101)
- [RCカー練習場「元気っ子さん」](https://genkikkosan.com/)

## ドリフト走行の理屈
- [自動車の運動と制御](https://www.amazon.co.jp/dp/4501419202/)
- [車両運動の安定性解析と制御への応用](https://www.tytlabs.com/japanese/review/rev321pdf/321_013ono.pdf)
- [On the dynamics of automobile drifting](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.103.9227&rep=rep1&type=pdf)
- [Analysis and control of high sideslip manoeuvres](https://www.tandfonline.com/doi/abs/10.1080/00423111003746140?journalCode=nvsd20)
- [Stabilization of steady-state drifting for a RWD vehicle](http://dcsl.gatech.edu/papers/avec10.pdf)

## ソフトウェア関連
- [M5StickC Library](https://github.com/m5stack/M5StickC)
- [M5StickC非公式日本語リファレンス](https://lang-ship.com/reference/unofficial/M5StickC/)
- [M5Stack公式ドキュメント](https://github.com/m5stack/m5-docs/blob/master/docs/ja/README_ja.md)
- [Arduino IDE](https://www.arduino.cc/en/software)
- [PID Controller - Wikipedia](https://en.wikipedia.org/wiki/PID_controller)

## ハードウェア関連
- [M5StickC本体例](https://www.switch-science.com/catalog/5517/)
- [ピンヘッダ（オス）例](https://www.amazon.co.jp/dp/B012HY288S/)
- [サーボ延長ケーブル例](https://www.amazon.co.jp/dp/B00W9ST610/)


以上

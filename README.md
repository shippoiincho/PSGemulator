# PSGもどき for CH32V003

CH32V003 で PSG (AY-3-8910など) をエミュレートします。
サンプリング周波数との兼ね合いはありますが、複数個の PSG のエミュレーションにも対応しています。
ホストとの接続は I2C を使います。

## 使い方
I2C の SDA/SCL (PC1/2)をホストに接続します。適当にプルアップ抵抗をかませてください。<br>



音声は PC4 に PWM で出力されますので、適当なフィルタをかませて、ライン入力などにつなぎます。
そのまま圧電ブザーなどにつないでもOKです<br>

I2C アドレス(デフォで 0x10) に、レジスタNo、設定値の順にデータを送ると、PSG のレジスタに書き込まれます。
読み出しには対応していません。<br>
PSGを複数個エミュレートする場合は、レジスタNo に 0x10 つづ加算します。
PSG2 のミキサーを操作する場合は 0x17 になります。<br>

レジスタ 0x0f に書き込みを行うとPSGの設定がリセットされます。この時に 0を書き込むと 3.58MHz に、それ以外の場合は 4MHz に基準クロックが変更できます。
複数個のエミュレーションを行う場合、0x1f に値を書き込むと PSG2 がリセットされますが、基準クロックは PSG1 (0x0f)に合わせられます。
基準クロックは個別に変更はできません。(できるようにしてもいいんだけど…)<br>

マイコン単体で使う場合など、内部オシレータを使う場合は、クロックの設定が 48MHz HSI になっていることを確認してください。<br>
8 ピン版でも使えるようにしているつもりですが、使えないピンがありましたら変更してください。<br>

## 解説

発音の方は、各CH ごとに矩形波を生成して、合成してるだけです。
合成した結果を約 190kHz (48MHz/256) の PWM で 256階調で出力しています。<br>

ノイズ生成は fmgen のコードを参考にしています。<br>
ノイズやエンベロープの処理が意外と重いので、 beepmidi よりも同時発声数は少なくなります。<br>

I2C は割り込みで処理しています。
エラーが発生したらI2Cをリセットするようにしています。

## 制限事項
- たまに I2C が固まります

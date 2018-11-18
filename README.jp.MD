# C++のXM430サーボモーターAPI
XM430サーボモーターの管理するの簡単なC++APIです。ロボットを制御するためのプログラムですが、いろいろなアプリケーション使えります。
試験環境はラスベリーパイ３B+上で[U2D2](http://www.robotis.us/u2d2/)で繋がれたモーターでした。サーボモーターの詳しく情報は[ここ](http://support.robotis.com/en/product/actuator/dynamixel_x/xm_series/xm430-w350.htm#bookmark5)です。
<br />　**エラーや頼み　新しい機能がぜひ待っています**

## インストールと必要環境
そのAPIはdynamixel－SDKに頼っています。そのリンクでライブラリーをダウンロードとインストールをしてください。ラスベリーパイのためにSBCのライブラリーが要ります。
そしてそのGITHUBをダウンロードして、MakeFileをmakeします。MakeFileの中で __SOURCES += yourcode.cpp__を書くまま、自分のコードを使えります。

## サンプルコード

```c
//Create a servo motor: servo*(dynamyxel ID, mode, currentlimit, goalcurrent)
MotorXM430 servo1(1, 3, 800, 90);
	
//Accessing the motors function:
servo1.PrintOperatingMode();
	
//Controlling the motor:
servo1.TorqueON();
servo1.Goto(180);
usleep(500000);
printf("Motor %d, current position: %f\n",servo1.GetID(), servo1.ReadAngle());
	
//Adding another motor:
MotorXM430 servo2(2, 3, 800, 90);
printf("Motor %d, current position: %f\n",servo2.GetID(), servo2.ReadAngle());
```

<img src="/images/ExampleRUN.PNG" width="450">

## APIの基本
ーーー

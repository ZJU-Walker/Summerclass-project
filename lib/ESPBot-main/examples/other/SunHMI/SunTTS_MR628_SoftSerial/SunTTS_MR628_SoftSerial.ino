/*
1.项目名称：MR628语音合成模块Arduino测试程序
2.显示模块：无
3.使用软件：Arduino IDE
4.配套上位机：无
5.项目组成：MR628语音合成模块
6.项目功能：将文字转为语音读出来
7.主要原理：具体参考MR628数据手册
接线定义:
  黑线--GND
  红线--5V
  黄线--D42 可以使用任意空闲引脚

*/

//使用前必看！！！！！！！！！！！！！！
//由于Arduino
//IDE编码问题，串口直接发送中文语音模块无法正确识别，因此将语音合成数据放在“sound.h”文件中，
//此文件在Arduino
//IDE中显示乱码，无需在意。请使用记事本打开"sound.h"文件修改语音数据，切勿在Arduino
//IDE中直接修改， 否则会引起语音合成数据错误！

#include <SoftwareSerial.h>

#include "sound.h"
SoftwareSerial tts(100, 42);  // (RX, TX)(A8, A9)

void setup() { tts.begin(9600); }

void loop() {
  //<I>18  提示音 1
  //<I>19  提示音 2
  tts.print("<I>18");  //播放提示音1
  delay(2000);
  //<M>1  背景音乐 1
  //<M>2  背景音乐 2
  //<M>0  关闭音乐
  tts.print("<M>0");  //背景音乐 1
  delay(50);
  tts.print(sound0);  //发送语音合成指令
  delay(6000);
  tts.print(sound1);  //发送语音合成指令
  delay(6000);
  tts.print(sound2);  //发送语音合成指令
  delay(6000);
  //   mySerial.print("<C>");//取消合成
  //    delay(50);
}

#include "OOPConfig.h"
//灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
// 构造函数原型  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM, HardwareSerial
// *uartPort = &GRAYSCALE_DEFAULT_PORT,
// bool isDarkHigh = true, bool isOffset = false);  默认构造函数
// 7路灰度，仅使用串口2, Num灰度数量，仅使用指定串口 #define RX2 19 #define TX2
//请提前调节7路灰度，深色为高电平，非偏移量输出。

#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

// 使用串口构造函数
//如果传感器浅色高电平 （黑色时低电平） 使用下方构造函数 根据调试时3号指示灯灭
// Grayscale GraySensors(/*Num*/ 7, /**uartPort*/ &Serial2, /*isDarkHigh*/
// false,
//                     /*isOffset*/ false);  // 7路灰度使用串口2
//如果传感器浅色时低电平 （黑色时高电平） 使用下方构造函数 根据调试时3号指示灯亮
// Grayscale GraySensors(/*Num*/ 7, /**uartPort*/ &Serial2, /*isDarkHigh*/ true,
//                     /*isOffset*/ false);  // 7路灰度使用串口2
Grayscale GraySensors;  //也可使用默认构造函数。
// uart输出量的结构体
Grayscale::strOutput GrayUartOutputIO;
unsigned long readTime, readPeriod;
void setup() {
  // Serial.begin(9600, SERIAL_8N1, RX, TX);
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);
#endif
  delay(300);
  readPeriod = micros();
}

void loop() {
  if (micros() - readPeriod > 5000)  //每5ms读取一次
  {
    readPeriod = micros();
    readTime = micros();
    GrayUartOutputIO = GraySensors.readUart();  //读取串口数字量数据
    readTime = micros() - readTime;
  }
#ifdef DEBUG_INFO
  Serial.print((String) "黑色点数：" + GrayUartOutputIO.ioCount +
               "点变化数:" + GrayUartOutputIO.ioChangeCount + "偏移量：" +
               GrayUartOutputIO.offset);
  Serial.print("二进制IO值:");
  Serial.print(GrayUartOutputIO.ioDigital, BIN);
  Serial.print("ReadTime:");
  Serial.print(readTime);
  Serial.println("微秒");
  Serial.println("Time:");
  Serial.println(millis());
#endif
  // delay(1);//必须delay
}

#include "ESP32Car.h"
//灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
// 构造函数原型  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM, HardwareSerial *uartPort = &GRAYSCALE_DEFAULT_PORT,
//bool isDarkHigh = true, bool isOffset = false);  默认构造函数 7路灰度，仅使用串口2, Num灰度数量，仅使用指定串口
//#define RX2 5
//#define TX2 18

#define RX1 5
#define TX1 18

#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif
    

//2.使用串口构造函数

Grayscale GraySensors(/*Num*/7,/**uartPort*/&Serial1,/*isDarkHigh*/true,/*isOffset*/false);    //7路灰度使用串口2


//uart输出量的结构体
Grayscale::strOutput GraySensorsUartIoOutput;
void setup()
{
  //Serial1.begin(9600, SERIAL_8N1, RX2, TX2);
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);
#endif
  delay(300);
}

void loop()
{
GraySensorsUartIoOutput = GraySensors.readUart();//读取串口数字量数据


#ifdef DEBUG_INFO
  Serial.print("uartIo:");
      Serial.print(GraySensorsUartIoOutput.ioDigital);
        Serial.print(",");
        Serial.print(GraySensorsUartIoOutput.ioCount);
         Serial.print(",");
        Serial.print(GraySensorsUartIoOutput.ioChangeCount);
         Serial.print(",");
        Serial.println(GraySensorsUartIoOutput.offset);

#endif
  delay(1000);
}
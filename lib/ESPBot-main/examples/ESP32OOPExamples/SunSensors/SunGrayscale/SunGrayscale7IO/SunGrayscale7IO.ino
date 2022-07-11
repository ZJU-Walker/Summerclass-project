
#include "SunConfig.h"
//灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
// 构造函数原型  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM, HardwareSerial *uartPort = &GRAYSCALE_DEFAULT_PORT,
//bool isDarkHigh = true, bool isOffset = false);  默认构造函数 7路灰度，仅使用串口2, Num灰度数量，仅使用指定串口


 //3.使用io接口的构造函数 
 //Grayscale(byte Num, bool isDarkHigh, bool isOffset = false);  // Num灰度数量，仅使用IO口
Grayscale GraySensorsIO(/*Num*/7,/*isDarkHigh*/true); //七路灰度仅使用IO口


//io输出量的机构体
Grayscale::strOutput GraySensorsIoOutput;

void setup()
{
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);
#endif
  delay(300);
}

void loop()
{

GraySensorsIoOutput = GraySensorsIO.readIO();

#ifdef DEBUG_INFO

  Serial.print("io:");
           Serial.print(GraySensorsIoOutput.ioDigital);//打印7位数对应十进制制
        Serial.print(",");
        Serial.print(GraySensorsIoOutput.ioCount);//打印1的数量
         Serial.print(",");
        Serial.print(GraySensorsIoOutput.ioChangeCount);//打印与上一次比较发生变化的位数
         Serial.print(",");
        Serial.println(GraySensorsIoOutput.offset);//打印位置偏移量
#endif
  delay(1000);
}

#include "SunConfig.h"
//灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
// 构造函数原型  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM, HardwareSerial *uartPort = &GRAYSCALE_DEFAULT_PORT,
//bool isDarkHigh = true, bool isOffset = false);  默认构造函数 7路灰度，仅使用串口2, Num灰度数量，仅使用指定串口

//1.使用默认构造函数，七路灰度仅使用串口2,使用默认参数=Grayscale GraySensors(7,&Serial2,true,false); 
Grayscale GraySensors;      

//2.使用串口构造函数
//Grayscale GraySensors(/*Num*/7,/**uartPort*/&Serial2,/*isDarkHigh*/true,/*isOffset*/false);    //7路灰度使用串口2

 //3.使用io接口的构造函数 
 //Grayscale(byte Num, bool isDarkHigh, bool isOffset = false);  // Num灰度数量，仅使用IO口
Grayscale GraySensorsIO(/*Num*/7,/*isDarkHigh*/true); //七路灰度仅使用IO口

//Grayscale GraySensors(8, &Serial2);     //八路灰度使用串口2
//Grayscale GraySensorsIO(8,true); //八路灰度仅使用IO口
//Grayscale GraySensors(8,&Serial2,true,false);    //八路灰度仅使用IO口2
//io输出量的结构体
Grayscale::strOutput GraySensorsIoOutput;
//uart输出量的结构体
Grayscale::strOutput GraySensorsUartIoOutput;
void setup()
{
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);
#endif
  delay(300);
}

void loop()
{
GraySensorsUartIoOutput = GraySensors.readUart();//读取串口数字量数据
GraySensorsIoOutput = GraySensorsIO.readIO();

#ifdef DEBUG_INFO
  Serial.print("uartIo:");
      Serial.print(GraySensorsUartIoOutput.ioDigital);
        Serial.print(",");
        Serial.print(GraySensorsUartIoOutput.ioCount);
         Serial.print(",");
        Serial.print(GraySensorsUartIoOutput.ioChangeCount);
         Serial.print(",");
        Serial.println(GraySensorsUartIoOutput.offset);
  Serial.print("io:");
           Serial.print(GraySensorsIoOutput.ioDigital);
        Serial.print(",");
        Serial.print(GraySensorsIoOutput.ioCount);
         Serial.print(",");
        Serial.print(GraySensorsIoOutput.ioChangeCount);
         Serial.print(",");
        Serial.println(GraySensorsIoOutput.offset);
#endif
  delay(1000);
}
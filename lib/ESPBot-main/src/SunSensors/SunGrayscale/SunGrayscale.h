/**
 * @file SunGrayscale.h
 * @author igcxl (igcxl@qq.com)
 * @brief 灰度传感器库,目前兼容7路/8路串口和IO口采集，输出值统一为深色为高电平（1），浅色为0。一般深色少，浅色多，统一后效率提高。
 * @version 0.6
 * @date 2020-11-07
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef SUN_GRAYSCALE_H
#define SUN_GRAYSCALE_H

#include <Arduino.h>

#ifndef GRAYSCALE_DEFAULT_PORT
#ifdef ARDUINO_ARCH_ESP32
#define GRAYSCALE_DEFAULT_PORT Serial2  //循迹传感器默认串口
#endif
#endif
#define USE_GRAY_UART  //使用串口
//#define USE_GRAY_IO    //使用IO口

#ifndef GRAYSCALE_DEFAULT_NUM
#define GRAYSCALE_DEFAULT_NUM 7  //循迹传感器默认数量
#endif

#ifdef USE_GRAY_IO
#define GRAY_PIN1 49
#define GRAY_PIN2 47
#define GRAY_PIN3 43
#define GRAY_PIN4 41
#define GRAY_PIN5 39
#define GRAY_PIN6 37
#define GRAY_PIN7 35
//#define GRAY_PIN8  27
#endif



class Grayscale {
 private:
  byte _grayNum;              //灰度传感器数量
  HardwareSerial *_uartPort;  //使用的串口号
  bool _isDarkHigh;           //深色是否输出高电平
  bool _isOffset;             //输出是否是偏移量

 public:
   struct strOutput  //灰度传感器输出值结构体
  {
    unsigned int ioDigital;  // 2进制io字00000000 01110000，统一为高电平深色1
    byte ioCount;//统计深色的数量
    byte ioChangeCount;//与上一次比较电平发生变化的数量
    float offset;//如果传感器设置为偏移量输出，则输出结构体值仅返回偏移量值。
  };

  //使用io接口的构造函数
  Grayscale(byte Num, bool isDarkHigh,
            bool isOffset = false);  // Num灰度数量，仅使用IO口
 //使用串口的构造函数
  Grayscale(byte Num = GRAYSCALE_DEFAULT_NUM,
            HardwareSerial *uartPort = &GRAYSCALE_DEFAULT_PORT,
            bool isDarkHigh = true,
            bool isOffset = false);  //默认构造函数 7路灰度，仅使用串口2,
                                     // Num灰度数量，不使用偏移量模式
  void initUART();  //串口初始化
  void initIO();    // io初始化
  strOutput readUart();
  strOutput readIO();
  byte getNum();
  byte popCount(unsigned int ioData);  // 统计数据中1的数量
  float ioOffset(unsigned int ioData, float midpointOffset = 0);
};
#endif
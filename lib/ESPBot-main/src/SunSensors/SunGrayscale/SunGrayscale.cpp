/**
 * @file SunGrayscale.h
 * @author igcxl (igcxl@qq.com)
 * @brief 灰度传感器库
 * @version 0.4
 * @date 2021-03-15
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "SunGrayscale.h"

/**
 * @brief 构造函数 使用IO方式构造并初始化
 *
 */
Grayscale::Grayscale(byte Num, bool isDarkHigh, bool isOffset)
    : _grayNum(Num), _isDarkHigh(isDarkHigh), _isOffset(isOffset) {
  this->initIO();  // IO初始化
}
/**
 * @brief 构造函数 使用串口方式构造并初始化
 *
 */
Grayscale::Grayscale(byte Num, HardwareSerial *uartPort, bool isDarkHigh,
                     bool isOffset)
    : _grayNum(Num),
      _uartPort(uartPort),
      _isDarkHigh(isDarkHigh),
      _isOffset(isOffset) {
  this->initUART();  //串口初始化
}

/**
 * @brief 初始化串口
 *
 */

void Grayscale::initUART() { (*_uartPort).begin(9600); }
/**
 * @brief 初始化IO口
 *
 */
void Grayscale::initIO() {
#ifdef USE_GRAY_IO
  pinMode(GRAY_PIN1, INPUT);
  pinMode(GRAY_PIN2, INPUT);
  pinMode(GRAY_PIN3, INPUT);
  pinMode(GRAY_PIN4, INPUT);
  pinMode(GRAY_PIN5, INPUT);
  pinMode(GRAY_PIN6, INPUT);
  pinMode(GRAY_PIN7, INPUT);
#ifdef GRAY_PIN8
  pinMode(GRAY_PIN8, INPUT);
#endif
#endif
}

Grayscale::strOutput Grayscale::readIO() {
  byte half[2] = {0};  // half[0]是高位， half[1]是低位。
  static unsigned int lastIoDigital;
#ifdef USE_GRAY_IO
  bitWrite(half[1], 0, digitalRead(GRAY_PIN1));
  bitWrite(half[1], 1, digitalRead(GRAY_PIN2));
  bitWrite(half[1], 2, digitalRead(GRAY_PIN3));
  bitWrite(half[1], 3, digitalRead(GRAY_PIN4));
  bitWrite(half[1], 4, digitalRead(GRAY_PIN5));
  bitWrite(half[1], 5, digitalRead(GRAY_PIN6));
  bitWrite(half[1], 6, digitalRead(GRAY_PIN7));
#ifndef GRAY_PIN8
  bitWrite(half[1], 7, 0);
#else
  bitWrite(half[1], 7, digitalRead(GRAY_PIN8));
#endif

#endif
  Grayscale::strOutput grayOutput;

  grayOutput.ioDigital = (unsigned int)word(half[0], half[1]);  //数字量
  if (_isDarkHigh == false) {  //统一成深色高电平
    switch (_grayNum) {
      case 7:
        grayOutput.ioDigital =
            grayOutput.ioDigital ^ 0x007f;  //转换为深色高电平
        break;
      case 8:
        grayOutput.ioDigital =
            grayOutput.ioDigital ^ 0x00ff;  //转换为深色高电平
        break;
      default:
        break;
    }
  }
  grayOutput.ioChangeCount = popCount(
      lastIoDigital ^ grayOutput.ioDigital);  //与上一次状态比较的变化量
  lastIoDigital = grayOutput.ioDigital;
  grayOutput.ioCount = popCount(grayOutput.ioDigital);  //高电平的数量
  grayOutput.offset = ioOffset(grayOutput.ioDigital);   //偏移量

  return grayOutput;
}

/**
 * @brief Construct a new Grayscale::read object
 *
 * @param Data 返回的数据
 * @param isOffset 是否是偏移量模式
 */
Grayscale::strOutput Grayscale::readUart() {
  unsigned char y = 0;
  unsigned char USART_RX_STA[3] = {0};  //数据缓存区
  unsigned char Num = 0;                //数组计数
  static unsigned int lastUartIoDigital;
  Grayscale::strOutput grayOutputUart;
  ////////////////////////开始通讯////////////////////////////////////
  (*_uartPort).write(0x57);
  ///////////////////////////接收数字量数值开始///////////////////////////////
  if (_isOffset == false) {
    //读五次
    for (y = 0; y <= 5; y++) {
      delay(1);
      if ((*_uartPort).available() > 0) {
        USART_RX_STA[Num++] = (*_uartPort).read();  //依次读取接收到的数据
        if (Num == 1) {
          Num = 0;
          grayOutputUart.ioDigital = USART_RX_STA[0];
          break;
        }
      }
    }
    if (_isDarkHigh == false) {
      switch (_grayNum) {
        case 7:
          grayOutputUart.ioDigital =
              grayOutputUart.ioDigital ^ 0x007f;  //转换为深色高电平
          break;
        case 8:
          grayOutputUart.ioDigital =
              grayOutputUart.ioDigital ^ 0x00ff;  //转换为深色高电平
          break;
        default:
          break;
      }
    }

    grayOutputUart.ioCount = popCount(grayOutputUart.ioDigital);
    grayOutputUart.ioChangeCount =
        popCount(lastUartIoDigital ^ grayOutputUart.ioDigital);
    lastUartIoDigital = grayOutputUart.ioDigital;

    grayOutputUart.offset = ioOffset(grayOutputUart.ioDigital, 0.0);

  }
  ///////////////////////////数字量数值结束///////////////////////////////
  /////////////////////////接受偏移量数值开始///////////////////////////////
  else {
    unsigned int Receive_data = 0;  //数据缓存区
    for (y = 0; y <= 10; y++) {
      delay(1);
      if ((*_uartPort).available() > 0) {
        USART_RX_STA[Num++] = (*_uartPort).read();  //依次读取接收到的数据
        if (Num == 3) {
          Num = 0;

          Receive_data = USART_RX_STA[1];
          Receive_data <<= 8;
          Receive_data |= USART_RX_STA[2];
          break;
        }
      }
      if (USART_RX_STA[0] == 1) {
        grayOutputUart.offset = float(Receive_data);
      } else {
        grayOutputUart.offset = -float(Receive_data);
      }
    }
  }
  ///////////////////////////偏移量数值///////////////////////////////
  return grayOutputUart;
}

/**
 * @brief 获取传感器数量
 *
 * @return byte 返回传感器数量
 */
byte Grayscale::getNum() { return _grayNum; }

/**
 * @brief 统计数据二进制中1的数量（汉明重量Hamming Weight）
 * @note
 * https://blog.csdn.net/hitwhylz/article/details/10122617?utm_source=blogkpcl12
 *        number &=(number-1)
 * @param ioData 传感器IO返回值
 * @return unsigned int
 */
byte Grayscale::popCount(unsigned int ioData) {
  byte count;
  for (count = 0; ioData; count++) {
    ioData &= ioData - 1;
  }

  return count;
}

float Grayscale::ioOffset(unsigned int ioData, float midpointOffset) {
  float offsetDistance = 0;
  byte io_count = 0;
  if (_grayNum == 7) {
    //                  以中心点为坐标系原点,右为正值，左为负值
    //              bit15,14,13,12,11,10, 9, 8, 7 ,6, 5, 4, 3, 2, 1, 0
    const float ioDistance[7] = {-42.99, -28.66, -14.33, 0,
                                 14.33,  28.66,  42.99};
    for (int i = 0; i < 7; i++) {
      //逐个取每一位
      // unsigned int ioTemp=ioData>>i&1;
      unsigned int ioTemp = ioData >> i & 0x01;
      if (ioTemp == 1) {
        offsetDistance += ioDistance[i];

        io_count++;
      }
    }

    if (io_count == 0) {
      return -999.0;  //如果全浅色
    } else {
      if (offsetDistance == 0.0) {
        return 0.0;
      } else {
        offsetDistance = offsetDistance / io_count - midpointOffset;
        io_count = 0;

        return offsetDistance;
      }
    }
  }
  if (_grayNum == 8) {
    //                  以中心为坐标系原点,右为正值，左为负值
    //              bit15,14,13,12,11,10, 9, 8, 7 ,6, 5, 4, 3, 2, 1, 0
    const float ioDistance[8] = {-67, -44, -24, -7, 7, 24, 44, 67};
    for (int i = 0; i < 8; i++) {
      //逐个取每一位
      // unsigned int ioTemp=ioData>>i&1;
      unsigned int ioTemp = ioData >> i & 0x01;
      if (ioTemp == 1) {
        offsetDistance += ioDistance[i];

        io_count++;
      }
    }

    if (io_count == 0) {
      return -999.0;  //如果全浅色 返回-999.0
    } else {
      if (offsetDistance == 0.0) {
        return 0.0;
      } else {
        offsetDistance = offsetDistance / io_count - midpointOffset;
        io_count = 0;

        return offsetDistance;
      }
    }
   
  }
   return offsetDistance;//增加默认返回类型 20220609
}
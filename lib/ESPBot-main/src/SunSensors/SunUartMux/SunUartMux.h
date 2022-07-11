#pragma once
#include "Arduino.h"
class SUNUARTMUX {
 public:
 //构造函数
  SUNUARTMUX(uint8_t INHPin, uint8_t APin, uint8_t BPin);
  //方法
  void enableUART(uint8_t UARTNum);
  bool disable();

 protected:
  uint8_t _enablePin;
  uint8_t _APin;
  uint8_t _BPin;
};

/* Encoder Library, for measuring quadrature encoded signals
 *
 */
#pragma once  //防止头文件被多次引用

#include <Arduino.h>

class SunEncoder {
 private:
  bool attached = false;
  bool _flipEncoder = false;

 public:
  uint8_t _pinA;
  uint8_t _pinB;
  volatile long count = 0;  //编码器计数
  SunEncoder(uint8_t pinA, uint8_t pinB);
  ~SunEncoder();
  void init();
  inline long read() {
    if (_flipEncoder) {
      return -count;
    } else {
      return count;
    }
  }
  inline void write(int p) { this->count = p; }
  //翻转编码器计数方向
  void flipEncoder(bool flipEnc);
};

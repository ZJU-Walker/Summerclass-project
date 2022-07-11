
#include "SunEncoder.h"
void IRAM_ATTR encoderAISR(void* arg) {
  SunEncoder* object = (SunEncoder*)arg;
  if (digitalRead(object->_pinA) == digitalRead(object->_pinB))
    object->count++;
  else
    object->count--;
}

void IRAM_ATTR encoderBISR(void* arg) {
  SunEncoder* object = (SunEncoder*)arg;
  if (digitalRead(object->_pinA) == digitalRead(object->_pinB))
    object->count--;
  else
    object->count++;
}

SunEncoder::SunEncoder(uint8_t pinA, uint8_t pinB) : _pinA(pinA), _pinB(pinB) {
  //不要在这里初始化init
}
SunEncoder::~SunEncoder() {
  if (attached) {
    detachInterrupt(_pinA);
    detachInterrupt(_pinB);
  }
}

void SunEncoder::init() {
  if (attached) return;
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);
  attachInterruptArg(_pinA, encoderAISR, this, CHANGE);
  attachInterruptArg(_pinB, encoderBISR, this, CHANGE);
  // interrupts();
  this->attached = true;
}

void SunEncoder::flipEncoder(bool flipEnc) { _flipEncoder = flipEnc; }

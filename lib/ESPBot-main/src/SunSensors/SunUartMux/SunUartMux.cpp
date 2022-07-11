#include "SunUartMux.h"

SUNUARTMUX::SUNUARTMUX(uint8_t INHPin, uint8_t APin, uint8_t BPin)
    : _enablePin(INHPin), _APin(APin), _BPin(BPin) {
  pinMode(_enablePin, OUTPUT);
  pinMode(_APin, OUTPUT);
  pinMode(_BPin, OUTPUT);
}

void SUNUARTMUX ::enableUART(uint8_t UARTNum) {
  switch (UARTNum) {
    case 1:
      this->disable();
      digitalWrite(_enablePin, LOW);
      digitalWrite(_APin, LOW);
      digitalWrite(_BPin, LOW);
      break;
    case 2:
      this->disable();
      digitalWrite(_enablePin, LOW);
      digitalWrite(_APin, HIGH);
      digitalWrite(_BPin, LOW);
      break;
    case 3:
      this->disable();
      digitalWrite(_enablePin, LOW);
      digitalWrite(_APin, LOW);
      digitalWrite(_BPin, HIGH);
      break;
    case 4:
      this->disable();
      digitalWrite(_enablePin, LOW);
      digitalWrite(_APin, HIGH);
      digitalWrite(_BPin, HIGH);
      break;
    default:
      this->disable();
      break;
  }
}
bool SUNUARTMUX ::disable() {
  digitalWrite(_enablePin, HIGH);
  return true;
}

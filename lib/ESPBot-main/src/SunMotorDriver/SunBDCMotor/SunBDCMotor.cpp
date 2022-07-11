#include "SunBDCMotor.h"

//默认引脚
BDCMotor::BDCMotor()
    : _M1PWMA(M1PWM1),
      _M1PWMB(M1PWM2),
      _M2PWMA(M2PWM1),
      _M2PWMB(M2PWM2),
      _M3PWMA(M3PWM1),
      _M3PWMB(M3PWM2),
      _M4PWMA(M4PWM1),
      _M4PWMB(M4PWM2) {}

BDCMotor::BDCMotor(uint8_t M1PWMA, uint8_t M1PWMB, uint8_t M2PWMA,
                   uint8_t M2PWMB, uint8_t M3PWMA, uint8_t M3PWMB,
                   uint8_t M4PWMA, uint8_t M4PWMB)
    : _M1PWMA(M1PWMA),
      _M1PWMB(M1PWMB),
      _M2PWMA(M2PWMA),
      _M2PWMB(M2PWMB),
      _M3PWMA(M3PWMA),
      _M3PWMB(M3PWMB),
      _M4PWMA(M4PWMA),
      _M4PWMB(M4PWMB) {}
//三电机委托构造函数
BDCMotor::BDCMotor(uint8_t M1PWMA, uint8_t M1PWMB, uint8_t M2PWMA,
                   uint8_t M2PWMB, uint8_t M3PWMA, uint8_t M3PWMB)
    : BDCMotor(M1PWMA, M1PWMB, M2PWMA, M2PWMB, M3PWMA, M3PWMB, M4PWM1, M4PWM2) {
}
void BDCMotor::init() {
  // Initialize the pin states used by the motor driver shield
  // digitalWrite is called before and after setting pinMode.
  // It called before pinMode to handle the case where the board
  // is using an ATmega AVR to avoid ever driving the pin high,
  // even for a short time.
  // It is called after pinMode to handle the case where the board
  // is based on the Atmel SAM3X8E ARM Cortex-M3 CPU, like the Arduino
  // Due. This is necessary because when pinMode is called for the Due
  // it sets the output to high (or 3.3V) regardless of previous
  // digitalWrite calls.
  digitalWrite(_M1PWMA, LOW);
  digitalWrite(_M1PWMB, LOW);
  digitalWrite(_M2PWMA, LOW);
  digitalWrite(_M2PWMB, LOW);
  digitalWrite(_M3PWMA, LOW);
  digitalWrite(_M3PWMB, LOW);
  digitalWrite(_M4PWMA, LOW);
  digitalWrite(_M4PWMB, LOW);
  pinMode(_M1PWMA, OUTPUT);
  digitalWrite(_M1PWMA, LOW);
  pinMode(_M1PWMB, OUTPUT);
  digitalWrite(_M1PWMB, LOW);
  pinMode(_M2PWMA, OUTPUT);
  digitalWrite(_M2PWMA, LOW);
  pinMode(_M2PWMB, OUTPUT);
  digitalWrite(_M2PWMB, LOW);
  pinMode(_M3PWMA, OUTPUT);
  digitalWrite(_M3PWMA, LOW);
  pinMode(_M3PWMB, OUTPUT);
  digitalWrite(_M3PWMB, LOW);
  pinMode(_M4PWMA, OUTPUT);
  digitalWrite(_M4PWMA, LOW);
  pinMode(_M4PWMB, OUTPUT);
  digitalWrite(_M4PWMB, LOW);
}

//接线编号映射到建模编号
void BDCMotor::map2model(uint8_t map2M1, uint8_t map2M2, uint8_t map2M3,
                         uint8_t map2M4) {
  switch (map2M1) {
    case 1:
      this->_M1PWMA = M1PWM1;
      this->_M1PWMB = M1PWM2;
      break;
    case 2:
      this->_M1PWMA = M2PWM1;
      this->_M1PWMB = M2PWM2;
      break;
    case 3:
      this->_M1PWMA = M3PWM1;
      this->_M1PWMB = M3PWM2;
      break;
    case 4:
      this->_M1PWMA = M4PWM1;
      this->_M1PWMB = M4PWM2;
      break;
    default:;
  }
  switch (map2M2) {
    case 1:
      this->_M2PWMA = M1PWM1;
      this->_M2PWMB = M1PWM2;
      break;
    case 2:
      this->_M2PWMA = M2PWM1;
      this->_M2PWMB = M2PWM2;
      break;
    case 3:
      this->_M2PWMA = M3PWM1;
      this->_M2PWMB = M3PWM2;
      break;
    case 4:
      this->_M2PWMA = M4PWM1;
      this->_M2PWMB = M4PWM2;
      break;
    default:;
  }
  switch (map2M3) {
    case 1:
      this->_M3PWMA = M1PWM1;
      this->_M3PWMB = M1PWM2;
      break;
    case 2:
      this->_M3PWMA = M2PWM1;
      this->_M3PWMB = M2PWM2;
      break;
    case 3:
      this->_M3PWMA = M3PWM1;
      this->_M3PWMB = M3PWM2;
      break;
    case 4:
      this->_M3PWMA = M4PWM1;
      this->_M3PWMB = M4PWM2;
      break;
    default:;
      switch (map2M4) {
        case 1:
          this->_M4PWMA = M1PWM1;
          this->_M4PWMB = M1PWM2;
          break;
        case 2:
          this->_M4PWMA = M2PWM1;
          this->_M4PWMB = M2PWM2;
          break;
        case 3:
          this->_M4PWMA = M3PWM1;
          this->_M4PWMB = M3PWM2;
          break;
        case 4:
          this->_M4PWMA = M4PWM1;
          this->_M4PWMB = M4PWM2;
          break;
        default:;
      }
  }
}
// speed should be a number between -255 and 255
void BDCMotor::setM1Speed(int speed) {
  //翻转速度
  if (_flipM1) {
    speed = -speed;
  }

  speed = constrain(speed, -255, 255);  //限制输入范围

  if (speed > 0)
    analogWrite(_M1PWMB, speed), analogWrite(_M1PWMA, 0);
  else if (speed == 0)
    analogWrite(_M1PWMB, 0), analogWrite(_M1PWMA, 0);
  else if (speed < 0)
    analogWrite(_M1PWMA, -speed), analogWrite(_M1PWMB, 0);
}

// speed should be a number between -255 and 255
void BDCMotor::setM2Speed(int speed) {
  if (_flipM2) {
    speed = -speed;
  }
  speed = constrain(speed, -255, 255);

  if (speed > 0)
    analogWrite(_M2PWMA, speed), analogWrite(_M2PWMB, 0);
  else if (speed == 0)
    analogWrite(_M2PWMB, 0), analogWrite(_M2PWMA, 0);
  else if (speed < 0)
    analogWrite(_M2PWMB, -speed), analogWrite(_M2PWMA, 0);
}

// speed should be a number between -255 and 255
void BDCMotor::setM3Speed(int speed) {
  if (_flipM3) {
    speed = -speed;
  }
  speed = constrain(speed, -255, 255);

  if (speed > 0)
    analogWrite(_M3PWMB, speed),
        analogWrite(_M3PWMA,
                    0);  ///<赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (speed == 0)
    analogWrite(_M3PWMA, 0), analogWrite(_M3PWMB, 0);
  else if (speed < 0)
    analogWrite(_M3PWMA, -speed),
        analogWrite(
            _M3PWMB,
            0);  ///<高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

// speed should be a number between -255 and 255
void BDCMotor::setM4Speed(int speed) {
  if (_flipM4) {
    speed = -speed;
  }
  speed = constrain(speed, -255, 255);

  if (speed > 0)
    analogWrite(_M4PWMA, speed), analogWrite(_M4PWMB, 0);
  else if (speed == 0)
    analogWrite(_M4PWMA, 0), analogWrite(_M4PWMB, 0);
  else if (speed < 0)
    analogWrite(_M4PWMB, -speed), analogWrite(_M4PWMA, 0);
}

// set speed for both motors
// speed should be a number between -255 and 255
void BDCMotor::setSpeeds(int LeftSpeed, int RightSpeed) {
  setM1Speed(LeftSpeed);
  setM2Speed(LeftSpeed);
  setM3Speed(RightSpeed);
  setM4Speed(RightSpeed);
}

void BDCMotor::setSpeeds(int m1Speed, int m2Speed, int m3Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
}

void BDCMotor::setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed) {
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
  setM3Speed(m3Speed);
  setM4Speed(m4Speed);
}

void BDCMotor::motorsBrake() {
  analogWrite(_M1PWMA, 255);
  analogWrite(_M1PWMB, 255);
  analogWrite(_M2PWMA, 255);
  analogWrite(_M2PWMB, 255);
  analogWrite(_M3PWMA, 255);
  analogWrite(_M3PWMB, 255);
  analogWrite(_M4PWMA, 255);
  analogWrite(_M4PWMB, 255);
}

void BDCMotor::flipMotors(bool flipM1, bool flipM2, bool flipM3, bool flipM4) {
  _flipM1 = flipM1;
  _flipM2 = flipM2;
  _flipM3 = flipM3;
  _flipM4 = flipM4;
}
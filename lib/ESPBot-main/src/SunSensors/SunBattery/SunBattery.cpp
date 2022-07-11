#include "SunBattery.h"
Battery::Battery() {
  _Battery_Pin = VOLT_PIN;
  _Threshold = VOLT_THRESHOLD;
}
Battery::Battery(byte battery_pin, unsigned int threshold) {
  _Battery_Pin = battery_pin;
  _Threshold = threshold;
}

unsigned int Battery::read(void) {
  int Value = analogRead(_Battery_Pin);

  return (Value * 5.371);  //放大100倍
}

/**
 * @brief 连续多次检测为低电压输出真
 *
 * @return true
 * @return false
 */
bool Battery::is_Volt_Low(void) {
  int i = 0;
  for (i = 0; i < 5; ++i) {
    if (read() > _Threshold) {
      break;
    }
  }
  if (i == 5) {
    return true;
  } else {
    return false;
  }
}

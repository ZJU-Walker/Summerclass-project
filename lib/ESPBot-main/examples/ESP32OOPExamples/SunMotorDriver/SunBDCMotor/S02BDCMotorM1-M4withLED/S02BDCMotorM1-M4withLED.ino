/*
 * This example uses the SunBDCMotor library to drive each motor with the
 *  AS4950 4 channel Motor Driver Board for Arduino forward, then backward.
 * 每个轮子轮流正转后反转，同时正转亮红灯。反转亮蓝灯
 * 请检查轮子编号和理论建模编号是否一致，修改map2model
 * 请检查转向是否正确，修改flipMotors
 */
#define USE_FASTLED //是否使用FastLED
#include "OOPConfig.h"

BDCMotor motors;      //建立直流电机实例对象
CRGB leds[NUM_LEDS];  // 建立光带leds实例对象
void ledShow(int bright) {
  if (bright > 0) {
    leds[0] = CRGB::Red;
    FastLED.setBrightness(bright);
    FastLED.show();
  } else {
    bright = -bright;
    leds[0] = CRGB::Blue;
    FastLED.setBrightness(bright);
    FastLED.show();
  }
}
void setup() {
  motors.init();
  motors.map2model(1, 2, 3,
                   4);  //映射实际接线电机编号到理论建模编号，默认顺序1，2，3，4
  motors.flipMotors(false, false, false,
                    false);  //根据实际转向进行调整false or true 黑色PCB电机
                             // false  绿色PCB电机true

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  Serial.begin(BAUDRATE);
  Serial.println("AS4950 Quad Motor Driver Board for Arduino");
  motors.setSpeeds(100, 100, 100, 100);
  ledShow(100);
  delay(5000);
  motors.motorsBrake();  //观察急刹效果
  ledShow(-100);
  delay(5000);
}

void loop() {
  // run M1 motor with positive speed
  //正转加速

  for (int speed = 0; speed <= 255; speed++) {
    motors.setM1Speed(speed);
    ledShow(speed);
    delay(2);
  }
  //正转减速
  for (int speed = 255; speed >= 0; speed--) {
    motors.setM1Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M1 motor with negative speed
  //反转加速
  for (int speed = 0; speed >= -255; speed--) {
    motors.setM1Speed(speed);
    ledShow(speed);
    delay(2);
  }
  //反转减速
  for (int speed = -255; speed <= 0; speed++) {
    motors.setM1Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M2 motor with positive speed

  for (int speed = 0; speed <= 255; speed++) {
    motors.setM2Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--) {
    motors.setM2Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M2 motor with negative speed

  for (int speed = 0; speed >= -255; speed--) {
    motors.setM2Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++) {
    motors.setM2Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M3 motor with positive speed

  for (int speed = 0; speed <= 255; speed++) {
    motors.setM3Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--) {
    motors.setM3Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M3 motor with negative speed

  for (int speed = 0; speed >= -255; speed--) {
    motors.setM3Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++) {
    motors.setM3Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M4 motor with positive speed

  for (int speed = 0; speed <= 255; speed++) {
    motors.setM4Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--) {
    motors.setM4Speed(speed);
    ledShow(speed);
    delay(2);
  }

  // run M4 motor with negative speed

  for (int speed = 0; speed >= -255; speed--) {
    motors.setM4Speed(speed);
    ledShow(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++) {
    motors.setM4Speed(speed);
    ledShow(speed);
    delay(2);
  }

  delay(500);
}
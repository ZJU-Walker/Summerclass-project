/*
 * This example uses the SunBDCMotor library to drive each motor with the
 *  AS4950 4 channel Motor Driver Board for Arduino forward, then backward.
 * 每个轮子同时正转后反转，同时正转亮红灯。反转亮蓝灯
 * 请检查轮子编号和理论建模编号是否一致，修改map2model
 * 请检查转向是否正确，修改flipMotors
 */
#define USE_FASTLED  //是否使用FastLED
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
  motors.flipMotors(true, true, true,
                    true);  //根据实际转向进行调整false or true   绿色PCB电机
  // motors.flipMotors(false, false, false,false);
  // //根据实际转向进行调整false or true 黑色PCB电机

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  Serial.begin(BAUDRATE);
  Serial.println("AS4950 Quad Motor Driver Board for Arduino");
   delay(1000);//等待
  motors.setSpeeds(100, 100, 100, 100);
  Serial.println("速度PWM:100");
  ledShow(100);
  delay(5000);
  motors.motorsBrake();  //观察急刹效果
  Serial.println("急刹");
  delay(1000);
  motors.setSpeeds(-100, -100, -100, -100);
  Serial.println("速度PWM:-100");
  ledShow(-100);
  delay(5000);
}

void loop() {
  // run M1-M4 motor with positive speed

  for (int speed = 0; speed < 256; speed++) {
    motors.setSpeeds(speed, speed, speed, speed);
    ledShow(speed);
    delay(100);
    Serial.println(speed);
    //观察电机死区，观察四个电机转向是否一致
  }
  motors.motorsBrake();  //观察急刹效果
  Serial.println("急刹");
  delay(5000);
  // run M1-M4 motor with postive speed
  motors.setSpeeds(100, 100, 100, 100);  //观察四个电机转向是否一致
  Serial.println("速度PWM:100");
  delay(5000);
  motors.setSpeeds(0, 0, 0, 0);  //观察惯性停车效果
  Serial.println("惯性刹车");
  delay(5000);
  for (int speed = 0; speed > -256; speed--) {
    motors.setSpeeds(speed, speed, speed, speed);
    ledShow(speed);
    delay(100);
    Serial.println(speed);
    //观察电机死区，观察四个电机转向是否一致
  }
  motors.motorsBrake();  //观察急刹效果
  Serial.println("急刹");
  delay(5000);
  // run M1-M4 motor with negative speed
  motors.setSpeeds(-100, -100, -100, -100);  //观察四个电机转向是否一致
  Serial.println("速度PWM:-100");
  delay(5000);
  motors.setSpeeds(0, 0, 0, 0);  //观察惯性停车效果
  Serial.println("惯性刹车");
  delay(5000);
}
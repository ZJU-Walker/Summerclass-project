/**
 * @file S3YD_BDCMotorOpenLoopControl.ino 有刷直流电机开环控制
 * @author igcxl (igcxl@qq.com)
 * @brief 有刷直流电机开环控制
 * @note 有刷直流电机开环控制
 * @version 0.9
 * @date 2022-04-08
 * @copyright Copyright © igcxl.com 2022
 *
 */
#include "POPConfig.h"          //ESP32小车库文件
//#include "analogWrite.h"


void setup() {
  pinMode(M1PWM1, OUTPUT);
  pinMode(M2PWM1, OUTPUT);
  pinMode(M3PWM1, OUTPUT);
  pinMode(M4PWM1, OUTPUT);
  pinMode(M1PWM2, OUTPUT);
  pinMode(M2PWM2, OUTPUT);
  pinMode(M3PWM2, OUTPUT);
  pinMode(M4PWM2, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  delay(5000);
  setSpeeds(100, 100, 100, 100);
  delay(5000);
  //电机死区测试
  for (int i = 0; i < 256; i++) {
    int iConstrain = i;
    // int iConstrain = constrain(i, 0, 255-motorDeadZone);
    // //将i限制在0-（255-motorDeadZone）区间，大于返回（255-motorDeadZone）
    // ，小于返回0
    setSpeeds(iConstrain, iConstrain, iConstrain, iConstrain);
    Serial.println(i);
    Serial.println(iConstrain);
    delay(100);
  }
}

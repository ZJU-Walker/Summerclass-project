/**
 * @file ESP32S3_BDCMotorOpenLoopControl.ino 有刷直流电机开环控制
 * @author igcxl (igcxl@qq.com)
 * @brief 有刷直流电机开环控制
 * @note 有刷直流电机开环控制
 * @version 0.9
 * @date 2022-04-08
 * @copyright Copyright © igcxl.com 2022
 *
 */

//#include "analogWrite.h"
//******************PWM引脚和电机驱动引脚***************************//

#define M1PWM1 13  // M1电机控制PWM引脚1 左轮电机，根据实际接线修改引脚
#define M1PWM2 14  // M1电机控制PWM引脚2 左轮电机，根据实际接线修改引脚

#define M2PWM1 10  // M2电机控制PWM引脚1 右轮电机，根据实际接线修改引脚
#define M2PWM2 12  // M2电机控制PWM引脚2 右轮电机，根据实际接线修改引脚

#define M3PWM1 40  // M3电机控制PWM波
#define M3PWM2 41  // M3电机控制PWM波

#define M4PWM1 16  // M4电机控制PWM波
#define M4PWM2 15  // M4电机控制PWM波
//******************电机启动初始值 **********************//
int motorDeadZone =
    0;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试

/****************************电机驱动函数*********************************************
函数功能：驱动四路电机运动
入口参数：motor1 M1电机驱动PWM[-255,255]；motor1  M2电机驱动PWM[-255,255]
**************************************************************************/
void setSpeeds(int motor1, int motor2, int motor3, int motor4) {
  // pwm限幅
  motor1 = constrain(motor1, -255 + motorDeadZone, 255 - motorDeadZone);
  motor2 = constrain(motor2, -255 + motorDeadZone, 255 - motorDeadZone);
  motor3 = constrain(motor3, -255 + motorDeadZone, 255 - motorDeadZone);
  motor4 = constrain(motor4, -255 + motorDeadZone, 255 - motorDeadZone);

  if (motor1 > 0)
    analogWrite(M1PWM2, motor1 + motorDeadZone),
        analogWrite(M1PWM1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motor1 == 0)
    analogWrite(M1PWM2, 0), analogWrite(M1PWM1, 0);
  else if (motor1 < 0)
    analogWrite(M1PWM1, -motor1 + motorDeadZone),
        analogWrite(
            M1PWM2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motor2 > 0)
    analogWrite(M2PWM2, motor2 + motorDeadZone),
        analogWrite(M2PWM1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motor2 == 0)
    analogWrite(M2PWM2, 0), analogWrite(M2PWM1, 0);
  else if (motor2 < 0)
    analogWrite(M2PWM1, -motor2 + motorDeadZone),
        analogWrite(
            M2PWM2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motor3 > 0)
    analogWrite(M3PWM1, motor3 + motorDeadZone),
        analogWrite(M3PWM2,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motor3 == 0)
    analogWrite(M3PWM2, 0), analogWrite(M3PWM1, 0);
  else if (motor3 < 0)
    analogWrite(M3PWM2, -motor3 + motorDeadZone),
        analogWrite(
            M3PWM1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motor4 > 0)
    analogWrite(M4PWM1, motor4 + motorDeadZone),
        analogWrite(M4PWM2,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motor4 == 0)
    analogWrite(M4PWM1, 0), analogWrite(M4PWM2, 0);
  else if (motor4 < 0)
    analogWrite(M4PWM2, -motor4 + motorDeadZone),
        analogWrite(
            M4PWM1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

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

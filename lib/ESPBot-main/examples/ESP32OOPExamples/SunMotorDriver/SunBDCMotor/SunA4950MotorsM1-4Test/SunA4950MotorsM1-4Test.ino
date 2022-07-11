/**
 * @file A4950MotorShieldTest.ino
 * @author igcxl (igcxl@qq.com)
 * @brief A4950MotorShield库测试程序，四个电机正转加速 刹车 反转
 * @version 0.3
 * @date 2021-07-19
 * 
 * @copyright Copyright © igcxl.com 2020 All right reserved.
 * 
 */


#include "SunConfig.h"
/*
 * This example uses the A4950MotorShield library to drive each motor with the
 *  A4950 Dual Motor Driver Shield for Arduino forward, then backward. 

 */



A4950MotorShield motors;

void setup()
{
  motors.init(); //电机初始化，初始化引脚模式和pwm频率和电机死区
  //motors.init(1,0); //电机初始化，初始化引脚模式和pwm频率和电机死区,1高频，0默认，死区pwm
  //motors.init(0,30); //电机初始化，初始化引脚模式和pwm频率和电机死区,1高频，0默认，死区pwm
  Serial.begin(BAUDRATE);
  Serial.println("A4950 Quad Motor Driver Shield for Arduino");
}

void loop()
{
  // run M1-M4 motor with positive speed

  for (int i = 0; i < 256; i++)
  {
    motors.setSpeeds(i, i, i, i);
    delay(100);
    Serial.println(i);
    //观察电机死区，观察四个电机转向是否一致
  }
  motors.motorsBrake();//观察急刹效果
  delay(5000);
  // run M1-M4 motor with negative speed
  motors.setSpeeds(-100, -100, -100, -100); //观察四个电机转向是否一致
  delay(5000);
  motors.setSpeeds(0, 0, 0, 0);//观察惯性停车效果
  delay(5000);
}
/**
 * @file A4950MotorShieldTest.ino
 * @author igcxl (igcxl@qq.com)
 * @brief A4950MotorShield库测试程序
 * @version 0.3
 * @date 2020-10-30
 * 
 * @copyright Copyright © igcxl.com 2020 All right reserved.
 * 
 */




//********************循迹传感器***************************//
#define LeftGrayscale 42   //左循迹传感器引脚
#define RightGrayscale 48  //左循迹传感器引脚
#define BLACK HIGH         //循迹传感器检测到黑色为“HIGH”,灯灭
#define WHITE LOW          //循迹传感器检测到白色为“LOW”，灯亮
//******************电机对象***************************//
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
    bool LeftGrayscaleValue = digitalRead(LeftGrayscale);
  bool RightGrayscaleValue = digitalRead(RightGrayscale);
    if (LeftGrayscaleValue == BLACK &&
      RightGrayscaleValue == BLACK)  //都是黑色，前进
  {
      motors.setSpeeds(100,100);
  }
  if (LeftGrayscaleValue == WHITE &&
      RightGrayscaleValue == WHITE)  //都是白色，后退
  {
     motors.setSpeeds(100,100);
  }
  if (LeftGrayscaleValue == BLACK &&
      RightGrayscaleValue == WHITE)  //偏右，左转纠正
  {
     motors.setSpeeds(0,100);
  }
  if (LeftGrayscaleValue == WHITE &&
      RightGrayscaleValue == BLACK)  //偏左，右转纠正
  {
     motors.setSpeeds(100,0);
  }

 
}
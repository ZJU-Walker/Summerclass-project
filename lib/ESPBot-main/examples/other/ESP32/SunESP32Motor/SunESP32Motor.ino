//任务要求：从出发区出发，巡线一圈，停在返回区
//******************电机驱动规格***************************//
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，三路循迹传感器
#include "analogWrite.h"

//******************电机驱动PWM引脚***************************//
#define LeftMotorAIN1   32  //左轮电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2   33  //左轮机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1   26  //右轮电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2   25  //  右轮电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚

//********************循迹传感器***************************//
#define LeftGrayscale 17  //左循迹传感器引脚，根据实际接线修改引脚
#define MiddleGrayscale 16  //中循迹传感器引脚，根据实际接线修改引脚
#define RightGrayscale 4  //左循迹传感器引脚，根据实际接线修改引脚
#define BLACK HIGH  //循迹传感器检测到黑色为“HIGH”,灯灭（黑），输出高电平1
#define WHITE LOW  //循迹传感器检测到白色为“LOW”，灯亮（白），输出低电平0



//******************电机启动初始值 **********************//
int motorDeadZone =    0;
//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
  函数功能：驱动两路电机运动
  入口参数：motora a电机驱动PWM；motorb  b电机驱动PWM；取值范围[-255,255]
**************************************************************************/
void RunMotors(int motora, int motorb) {
  if (motora > 0)
    analogWrite(LeftMotorAIN2, motora + motorDeadZone),
                analogWrite(LeftMotorAIN1,
                            0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(LeftMotorAIN2, 0), analogWrite(LeftMotorAIN1, 0);
  else if (motora < 0)
    analogWrite(LeftMotorAIN1, -motora + motorDeadZone),
                analogWrite(
                  LeftMotorAIN2,
                  0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(RightMotorBIN2, motorb + motorDeadZone),
                analogWrite(RightMotorBIN1,
                            0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(RightMotorBIN2, 0), analogWrite(RightMotorBIN1, 0);
  else if (motorb < 0)
    analogWrite(RightMotorBIN1, -motorb + motorDeadZone),
                analogWrite(
                  RightMotorBIN2,
                  0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

void brake()  //急刹车，停车
{
  digitalWrite(RightMotorBIN1, HIGH);
  digitalWrite(RightMotorBIN2, HIGH);
  digitalWrite(LeftMotorAIN1, HIGH);
  digitalWrite(LeftMotorAIN2, HIGH);
}
void stop()  //普通刹车，停车，惯性滑行
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

unsigned long lastTime;
bool timeflag = 1;
//==========================================================

void setup() {
  pinMode(LeftMotorAIN1, OUTPUT);  //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);

  Serial.begin(115200);


}

void loop() {

  //  brake();
  //delay(2000);
  //  RunMotors(150,150);
  //  delay(2000);
  //brake();
  //delay(2000);
  //  RunMotors(-150,-150);
  //   delay(2000);
  //    RunMotors(0, 0);
  //     delay(2000);
  for (int i = 0; i < 256; i++)
  {

    RunMotors(i, i);
    Serial.println(i);
    delay(100);
  }
}

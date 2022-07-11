/**
 * @file BDCMotorOpenLoopControl.ino 有刷直流电机开环控制
 * @author igcxl (igcxl@qq.com)
 * @brief 有刷直流电机开环控制
 * @note 有刷直流电机开环控制
 * @version 0.9
 * @date 2021-06-02
 * @copyright Copyright © igcxl.com 2019
 * 
 */

#include "analogWrite.h"  
//******************PWM引脚和电机驱动引脚***************************//


#define AIN1 32  // A电机控制PWM波
#define AIN2 33  // A电机控制PWM波

#define BIN1 26  // B电机控制PWM波
#define BIN2 25  // B电机控制PWM波

#define CIN1 27  // C电机控制PWM波
#define CIN2 14  // C电机控制PWM波

#define DIN1 13  // D电机控制PWM波
#define DIN2 12  // D电机控制PWM波
//******************电机启动初始值 **********************//
int motorDeadZone = 30;//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：赋值给PWM寄存器 
入口参数：PWM
**************************************************************************/
void Set_PWM(int motora, int motorb, int motorc,int motord) { 
  if (motora > 0)        analogWrite(AIN2, motora+motorDeadZone), analogWrite(AIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motora == 0)   analogWrite(AIN2, 0), analogWrite(AIN1,0);   
  else if (motora < 0)   analogWrite(AIN1, -motora+motorDeadZone), analogWrite(AIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)        analogWrite(BIN2,motorb+motorDeadZone),analogWrite(BIN1, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorb == 0)   analogWrite(BIN2, 0),analogWrite(BIN1, 0);
  else if (motorb < 0)   analogWrite(BIN1,-motorb+motorDeadZone),analogWrite(BIN2, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorc > 0)        analogWrite(CIN1,motorc+motorDeadZone),analogWrite(CIN2,0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motorc == 0)   analogWrite(CIN2, 0), analogWrite(CIN1, 0);
  else if (motorc < 0)   analogWrite(CIN2, -motorc+motorDeadZone), analogWrite(CIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motord > 0)        analogWrite(DIN1,motord+motorDeadZone),analogWrite(DIN2, 0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if(motord == 0)   analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motord < 0)   analogWrite(DIN2, -motord+motorDeadZone), analogWrite(DIN1, 0);//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN2, OUTPUT);
  Serial.begin(9600);  
}

void loop() {
Set_PWM(100,100,100,100 );
delay(5000);
//电机死区测试
for(int i = 0; i< 256; i++)
{
int iConstrain = i;
//int iConstrain = constrain(i, 0, 255-motorDeadZone); //将i限制在0-（255-motorDeadZone）区间，大于返回（255-motorDeadZone） ，小于返回0
Set_PWM(iConstrain,iConstrain,iConstrain,iConstrain);
Serial.println(i);
Serial.println(iConstrain);
delay(100);
}
}

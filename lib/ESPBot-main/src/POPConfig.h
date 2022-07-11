#pragma once

#define USE_BDC_MOTOR_BORAD
#define USE_ENCODER//是否使用编码器
#if CONFIG_IDF_TARGET_ESP32
#include "pins_ESP32.h"
#elif CONFIG_IDF_TARGET_ESP32S2
#include "pins_ESP32S312K.h"
#elif CONFIG_IDF_TARGET_ESP32C3
#include "pins_ESP32S312K.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "pins_ESP32S3YD.h"
#endif
//修改为源地版S3
/*
#if defined(ARDUINO_ESP32S3_DEV)
#include "pins_ESP32S312K.h"
#elif defined(ARDUINO_NodeMCU_32S)
#include "pins_ESP32.h"
#else
#error "This library only supports boards with an AVR, ESP32."
#endif
*/

#if defined(ARDUINO_ARCH_ESP32)
#include "SunSensors/SunGrayscale/SunGrayscale.h"  //灰度传感器库
#endif
//******************电机启动初始值 **********************//
int motorDeadZone =
    0;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/****************************电机驱动函数*********************************************
函数功能：驱动二路电机运动
入口参数：motor1 M1电机驱动PWM[-255,255]；motor1  M2电机驱动PWM[-255,255]
**************************************************************************/
void setSpeeds(int motor1, int motor2) {
  // pwm限幅
  motor1 = constrain(motor1, -255 + motorDeadZone, 255 - motorDeadZone);
  motor2 = constrain(motor2, -255 + motorDeadZone, 255 - motorDeadZone);

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
}

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

void brake()  //急刹车，停车
{
  analogWrite(M1PWM1, 255);
  analogWrite(M1PWM2, 255);
  analogWrite(M2PWM1, 255);
  analogWrite(M2PWM2, 255);
  analogWrite(M3PWM1, 255);
  analogWrite(M3PWM2, 255);
  analogWrite(M4PWM1, 255);
  analogWrite(M4PWM2, 255);
}

/*
todo:开发适合 esp32 encoder库
void readEncoder(uint8_t pin1, uint8_t pin2)
{

}
*/

/*****函数功能：外部中断读取M1编码器数据，A B向均触发
 * 四倍频注意外部中断是跳变沿触发********/
volatile long EncoderCount[4];  //编码器计数
void readM1EncoderA() {
  if (digitalRead(M1ENA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M1ENB) == LOW)
      EncoderCount[0]--;  //根据另外一相电平判定方向
    else
      EncoderCount[0]++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M1ENB) == LOW)
      EncoderCount[0]++;  //根据另外一相电平判定方向
    else
      EncoderCount[0]--;
  }
}

void readM1EncoderB() {
  if (digitalRead(M1ENB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M1ENA) == LOW)
      EncoderCount[0]++;  //根据另外一相电平判定方向
    else
      EncoderCount[0]--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M1ENA) == LOW)
      EncoderCount[0]--;  //根据另外一相电平判定方向
    else
      EncoderCount[0]++;
  }
}
/*****函数功能：外部中断读取M2编码器数据，注意外部中断是跳变沿触发********/
void readM2EncoderA() {
  if (digitalRead(M2ENA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M2ENB) == LOW)
      EncoderCount[1]--;  //根据另外一相电平判定方向
    else
      EncoderCount[1]++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M2ENB) == LOW)
      EncoderCount[1]++;  //根据另外一相电平判定方向
    else
      EncoderCount[1]--;
  }
}

void readM2EncoderB() {
  if (digitalRead(M2ENB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M2ENA) == LOW)
      EncoderCount[1]++;  //根据另外一相电平判定方向
    else
      EncoderCount[1]--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M2ENA) == LOW)
      EncoderCount[1]--;  //根据另外一相电平判定方向
    else
      EncoderCount[1]++;
  }
}
/*****函数功能：外部中断读取M3编码器数据，注意外部中断是跳变沿触发********/
void readM3EncoderA() {
  if (digitalRead(M3ENA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M3ENB) == LOW)
      EncoderCount[2]--;  //根据另外一相电平判定方向
    else
      EncoderCount[2]++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M3ENB) == LOW)
      EncoderCount[2]++;  //根据另外一相电平判定方向
    else
      EncoderCount[2]--;
  }
}

void readM3EncoderB() {
  if (digitalRead(M3ENB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M3ENA) == LOW)
      EncoderCount[2]++;  //根据另外一相电平判定方向
    else
      EncoderCount[2]--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M3ENA) == LOW)
      EncoderCount[2]--;  //根据另外一相电平判定方向
    else
      EncoderCount[2]++;
  }
}

/*****函数功能：外部中断读取M3编码器数据，注意外部中断是跳变沿触发********/
void readM4EncoderA() {
  if (digitalRead(M4ENA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M4ENB) == LOW)
      EncoderCount[3]--;  //根据另外一相电平判定方向
    else
      EncoderCount[3]++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M4ENB) == LOW)
      EncoderCount[3]++;  //根据另外一相电平判定方向
    else
      EncoderCount[3]--;
  }
}

void readM4EncoderB() {
  if (digitalRead(M4ENB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(M4ENA) == LOW)
      EncoderCount[3]++;  //根据另外一相电平判定方向
    else
      EncoderCount[3]--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(M4ENA) == LOW)
      EncoderCount[3]--;  //根据另外一相电平判定方向
    else
      EncoderCount[3]++;
  }
}
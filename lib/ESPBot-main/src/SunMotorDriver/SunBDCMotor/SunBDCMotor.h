/**
 * @file SunBDCmotor.h
 * @author Sunny (igcxl@qq.com)
 * @brief as4950四路直流有刷电机驱动板驱动库
 * @version 0.5
 * @date 2022-06-11
 * @copyright Copyright (c) 2022
 * */
#pragma once  //防止头文件被多次引用

#include <Arduino.h>

#ifndef USE_BDC_MOTOR_BORAD
#define USE_BDC_MOTOR_BORAD
#define M1PWM1 17  // M1电机控制PWM引脚1 左轮电机，根据实际转动方向对调了引脚
#define M1PWM2 18  // M1电机控制PWM引脚2 左轮电机，根据实际转动方向对调了引脚

#define M2PWM1 15  // M2电机控制PWM引脚1 右轮电机
#define M2PWM2 16  // M2电机控制PWM引脚2 右轮电机

#define M3PWM1 2   // M3电机控制PWM波
#define M3PWM2 42  // M3电机控制PWM波

#define M4PWM1 41  // M4电机控制PWM波，根据实际转动方向对调了引脚
#define M4PWM2 40  // M4电机控制PWM波，根据实际转动方向对调了引脚
#endif

//四路
class BDCMotor {
 public:
  BDCMotor();
  //用户自定义构造函数，函数重载
  BDCMotor(uint8_t M1PWMA, uint8_t M1PWMB, uint8_t M2PWMA, uint8_t M2PWMB,
           uint8_t M3PWMA, uint8_t M3PWMB, uint8_t M4PWMA, uint8_t M4PWMB);
  BDCMotor(uint8_t M1PWMA, uint8_t M1PWMB, uint8_t M2PWMA, uint8_t M2PWMB,
           uint8_t M3PWMA, uint8_t M3PWMB);
  //初始化函数1
  void init();
  //接线编号映射到建模编号
  void map2model(uint8_t map2M1=1,uint8_t map2M2=2,uint8_t map2M3=3,uint8_t map2M4=4);

  //设置电机1速度
  void setM1Speed(int speed);
  //设置电机2速度
  void setM2Speed(int speed);
  //设置电机3速度
  void setM3Speed(int speed);
  //设置电机4速度
  void setM4Speed(int speed);
  //设置左右侧电机速度
  void setSpeeds(int LeftSpeed, int RightSpeed);
  //设置3个电机速度，来福轮
  void setSpeeds(int m1Speed, int m2Speed, int m3Speed);
  //设置4个电机速度，麦克纳姆轮
  void setSpeeds(int m1Speed, int m2Speed, int m3Speed, int m4Speed);
  //快速停Fast stop，慢速停slow stop setSpeeds(0,0)
  void motorsBrake();
  //翻转电机转动方向
  void flipMotors(bool flipM1, bool flipM2, bool flipM3, bool flipM4);

 private:
  //电机引脚
  uint8_t _M1PWMA;
  uint8_t _M1PWMB;
  uint8_t _M2PWMA;
  uint8_t _M2PWMB;
  uint8_t _M3PWMA;
  uint8_t _M3PWMB;
  uint8_t _M4PWMA;
  uint8_t _M4PWMB;
  //翻转电机转动方向变量 默认不翻转
  bool _flipM1 = false;
  bool _flipM2 = false;
  bool _flipM3 = false;
  bool _flipM4 = false;
};

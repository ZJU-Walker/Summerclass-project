//引脚编号
//适配ESP32-S3YD主控版本：V0.1以上

#pragma once

//******************电压检测引脚***************************//
#ifndef VOLT_PIN
#define VOLT_PIN A255  ///<电压检测引脚无
#endif
//******************FASTLED设置***************************//
#ifdef USE_FASTLED
#define NUM_LEDS 1  //每个灯珠需要60ma
#define LED_PIN 48
//#define CLOCK_PIN 48
#define LED_TYPE WS2812B  // LED灯带型号,见源地S3原理图
#define COLOR_ORDER GRB  // RGB灯珠中红色、绿色、蓝色LED的排列顺序
#endif
//*******PWM引脚和电机驱动引脚根据实际方向修改引脚*********//
#ifdef USE_BDC_MOTOR_BORAD
#define M1PWM1 18  // M1电机控制PWM引脚1 左轮电机
#define M1PWM2 17  // M1电机控制PWM引脚2 左轮电机

#define M2PWM1 15  // M2电机控制PWM引脚1 右轮电机
#define M2PWM2 16  // M2电机控制PWM引脚2 右轮电机

#define M3PWM1 2   // M3电机控制PWM波
#define M3PWM2 42  // M3电机控制PWM波

#define M4PWM1 40  // M4电机控制PWM波
#define M4PWM2 41  // M4电机控制PWM波
#endif

//******************编码器引脚***************************//
#ifdef USE_ENCODER
#define M1ENA 5  // M1路电机编码器引脚A，外部中断
#define M1ENB 4  // M1路电机编码器引脚B，外部中断

#define M2ENA 7  // M2路电机编码器引脚A，外部中断
#define M2ENB 6  // M2路电机编码器引脚B

#define M3ENA 21  // M3路电机编码器引脚A，外部中断
#define M3ENB 14  // M3路电机编码器引脚B

#define M4ENA 39  // M4路电机编码器引脚A，外部中断
#define M4ENB 38  // M4路电机编码器引脚B
#endif
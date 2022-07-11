
#pragma once



//引脚编号



//******************电压检测引脚***************************//
#ifndef VOLT_PIN
#define VOLT_PIN A0  ///<电压检测引脚A0
#endif

//******************PWM引脚和电机驱动引脚***************************//

#define M1PWM1 3  // M1电机控制PWM引脚1 左轮电机，根据实际接线修改引脚
#define M1PWM2 4  // M1电机控制PWM引脚2 左轮电机，根据实际接线修改引脚

#define M2PWM1 39  // M2电机控制PWM引脚1 右轮电机，根据实际接线修改引脚
#define M2PWM2 38  // M2电机控制PWM引脚2 右轮电机，根据实际接线修改引脚

#define M3PWM1 40  // M3电机控制PWM波
#define M3PWM2 41  // M3电机控制PWM波

#define M4PWM1 16  // M4电机控制PWM波
#define M4PWM2 15  // M4电机控制PWM波
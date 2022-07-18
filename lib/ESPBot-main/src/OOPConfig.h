#pragma once
#define USE_OLED             //是否使用OLED
#define USE_BDC_MOTOR_BORAD  //是否使用4通道直流电机驱动板
#define USE_ENCODER          //是否使用编码器
//#define USE_MPU6050_TOCKN
#define USE_MPU6050_DMP  //是否使用MPU6050_DMP
#define USE_KINEMATICS   //是否使用运动学库
#define USE_ODOMETRY     //是否使用里程计
#define USE_CARP2P       //是否使用点整定库
//#define USE_STEP_MOTOR_BORAD//是否使用4通道步进电机驱动板
//#define USE_FASTLED//是否使用FastLED

#define BAUDRATE 115200
#define PWM_MAX 255
#define PWM_MIN -PWM_MAX

#include "SunController/SunPID/SunPID.h"  //PID控制器库
#include "pins_ESP32S3YD.h"               //修改为源地版S3

//******************FASTLED设置***************************//
#ifdef USE_FASTLED
#include "FastLED.h"  //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
#endif

//******************七路灰度***************************//
#ifdef ARDUINO_ARCH_ESP32
#include "SunSensors/SunGrayscale/SunGrayscale.h"  //灰度传感器库
#endif

//******************电机驱动库设置***************************//
#ifdef USE_BDC_MOTOR_BORAD
#include "SunMotorDriver/SunBDCMotor/SunBDCMotor.h"
#endif

//******************编码器库***************************//
#ifdef USE_ENCODER
#include "SunSensors/SunEncoder/SunEncoder.h"
#endif

#define TIMER_PERIOD 10
#define BUTTON_PIN 0  // 47

//******************机器人底盘构型***************************//
#define TYPE_BALL1 1
#define TYPE_BLANCE2 2
#define TYPE_DIFFERENTIAL_DRIVE3 3
#define TYPE_OMNI3 4
#define TYPE_ACKERMAN 5
#define TYPE_MECANUM_X 6
#define TYPE_SKID4WD 7
#define TYPE_OMNI_X 8
#define TYPE_DEFAULT_2WD_4WD 9

#define BASETYPE TYPE_MECANUM_X  //根据构型修改
#define BT(T) (BASETYPE == TYPE_##T)

#if BT(MECANUM_X)
#define WHEELS_NUM 4
#define WHEEL_DIAMETER \
  0.065  // wheel's diameter in meters 0.06、0.065、0.08 必须保留小数点 单位m
#define LR_WHEELS_DISTANCE \
  0.18  // distance between left and right wheels左右轮距，单位m 左右轮距
        // 必须保留小数点
#define FR_WHEELS_DISTANCE \
  0.1415  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m  0.165 共轴 0.19 必须保留小数点
#define CENTER_WHEELS_DISTANCE 0.1  //  三轮旋转中心到轮距离，单位m
#elif BT(OMNI3)
#define WHEELS_NUM 3          //主动轮数量
#define WHEEL_DIAMETER 0.075  // wheel's diameter in meters 0.06、0.065、0.08
#define CENTER_WHEELS_DISTANCE 0.10  // 三轮旋转中心到轮自距离，单位m

#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距

#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#elif BT(ACKERMAN)
#define WHEELS_NUM 2          //主动轮数量
#define WHEEL_DIAMETER 0.065  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距

#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#define MAX_STEERING_ANGLE \
  0.415  // max steering angle. This only applies to Ackermann steering

#elif BT(SKID4WD)
#define WHEELS_NUM 4
#define WHEEL_DIAMETER 0.060  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距

#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#elif BT(DEFAULT_2WD_4WD)
#define WHEELS_NUM 4
#define WHEEL_DIAMETER 0.060  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距

#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m
#else

#define WHEELS_NUM 4
#define WHEEL_DIAMETER 0.065  // wheel's diameter in meters 0.06、0.065、0.08
#define LR_WHEELS_DISTANCE \
  0.19  // distance between left and right wheels左右轮距，单位m 左右轮距

#define FR_WHEELS_DISTANCE \
  0.165  // distance between front and rear wheels. Ignore this if you're on
         // 2WD/ACKERMANN 前后轴距，单位m

#endif
//*******************机器人底盘电机参数***************************//
//电机参数
//根据实际使用电机选择宏定义,务必和实际使用型号一致！！！
#define USE_12V366RPM13PPR30_MOTOR  //尾部绿色PCB
                                    //详细参数：https://item.taobao.com/item.htm?spm=a1z10.3-c-s.w4002-15726392041.15.1e936ab8Hc8eQf&id=45347924687
//#define USE_12V333RPM11PPR30_MOTOR  //尾部黑色PCB
//详细参数：https://item.taobao.com/item.htm?spm=a1z0d.6639537.1997196601.1054.79cb7484Vto1GY&id=609636943444

#ifdef USE_12V366RPM13PPR30_MOTOR
#define MAX_RPM 366  // 电机额定转速  2020电机规格，请根据实际电机数据填写
#define MOTOR_TORQUE \
  1  // motor's torque  2020电机规格 1kg.cm，请根据实际电机数据填写
#define COUNTS_PER_REV 1560  // wheel encoder's num of ticks per rev 390*4=1560

bool FLIP_MOTOR[] = {true, true, true, true};
//编码器计数方向翻转变量数组，false不修正,true需要修正
bool FLIP_ENCODER[] = {false, true, false, true};
#endif
#ifdef USE_12V333RPM11PPR30_MOTOR
#define MAX_RPM 333  // 电机额定转速  2021第二期电机规格，请根据实际电机数据填写
#define MOTOR_TORQUE \
  1  // motor's torque  2020电机规格 1kg.cm，请根据实际电机数据填写
#define COUNTS_PER_REV \
  1320  // wheel encoder's num of ticks per rev 11*30*4=1320
bool FLIP_MOTOR[] = {false, false, false, false};
bool FLIP_ENCODER[] = {true, false, true, false};
#endif
//******************运动学库***************************//

#ifdef USE_KINEMATICS
#include "SunWMR/SunKinematics/SunKinematics.h"
#endif
//******************里程计库***************************//
#ifdef USE_ODOMETRY
#include "SunWMR/SunOdometry/SunWheelOdometry.hpp"
#endif
//******************点镇定***************************//
#ifdef USE_CARP2P
#include "SunWMR/SunCar/SunCar.hpp"
#endif
#ifdef USE_MPU6050_DMP  //是否使用MPU6050_DMP


const int XAccelOffset = 472;
const int YAccelOffset = -774;
const int ZAccelOffset = 1534;
const int XGyroOffset = -214;
const int YGyroOffset = 63;
const int ZGyroOffset = -2;

#endif
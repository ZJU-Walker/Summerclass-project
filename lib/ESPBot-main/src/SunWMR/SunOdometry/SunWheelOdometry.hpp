/**
 * @file SunWheelOdometry.h
 * @author igcxl (igcxl@qq.com)
 * @brief 轮式里程计 ESP32 S3
 * @version 0.2
 * @date 2022-07-03
 * 方向定义
 *            ^+X
 *            |
 *            |
 * +Y<--------|

轮式里程计误差来源：

(1) 积分时有限的分辨率；
(2) 不同的车轮直径；
(3) 轮子接触点发生变化；
(4) 非均匀的地面接触以及不同位置变化的摩擦系数导致打滑；
关于错误的消除：
确定的误差可以通过合适的标定(calibration)消除；
非确定的误差需要通过误差模型进行描述，这将导致非确定的位置估计结果。
todo：
提供线性坐标校准函数

@copyright Copyright (c) 2022

**/
#pragma once

#include "Arduino.h"
#include "SunWMR/SunKinematics/SunKinematics.h"     //移动单元运动学库
#ifndef TIMER_PERIOD
#define TIMER_PERIOD 10
#endif

//class Kinematics;//前向引用声明,与Kinematis类相互包含，需要前向引用声明防止报错

class WheelOdometry {
 public:
  WheelOdometry(Kinematics *kinematic);

  struct output  //输出机器人全局坐标系下的位姿
  {
    float position_x;     // x位置 单位mm 或 m
    float position_y;     // y位置 单位mm 或 m
    float heading_theta;  // 航向角 单位rad 或 度
  };

  struct output botPosition;

  // 4WD 麦克纳姆轮轮式里程计 输入编码器脉冲数 输出小车位姿 单位: mm 
  output getPositon_mm(int pulses1, int pulses2, int pulses3, int pulses4);
  //4WD 麦克纳姆轮轮式里程计 输入编码器脉冲数和IMU偏转角（弧度制） 输出小车位姿 单位: mm 
  output getPositon_mm(int pulses1, int pulses2, int pulses3, int pulses4,float imu_yaw);
 private:
  Kinematics *_kinematic;
};



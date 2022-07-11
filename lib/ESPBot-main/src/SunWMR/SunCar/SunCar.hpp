/**
 * @file SunCar.h
 * @author igcxl (igcxl@qq.com)
 * @brief 小车全局运动库
 * @version 0.2
 * @date 2021-07-25
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

@copyright Copyright (c) 2021

**/
#pragma once

#include "Arduino.h"
#include "SunWMR/SunOdometry/SunWheelOdometry.hpp"  //轮式里程计库

//class    WheelOdometry;  //前向引用声明,与WheelOdometry类相互包含，需要前向引用声明防止报错
// 全局坐标系下的目标位姿
typedef struct {
  float world_x;
  float world_y;
  float world_angular_z;
} CAR_GOAL_POINT;
//比例系数和速度上限结构体
typedef struct {
  float kx;
  float ky;
  float kz;
  float MAX_x;          // m/s
  float MAX_y;         // m/s
  float MAX_z;  // rad/s
} CAR_KPS_MAX;
 //输出速度结构体
typedef struct 
{
  float vel_x;          // m/s
  float vel_y;         // m/s
  float angular_vel_z;  // rad/s
} VEL_OUTPUT;

class Car {
 public:
  Car(WheelOdometry *odometry);

  VEL_OUTPUT worldVel;
  VEL_OUTPUT botVel;
  

  // 获得世界坐标系下的速度
  void getWorldVel(CAR_GOAL_POINT goalPoint,CAR_KPS_MAX carKps);
  //获得附体坐标系下的速度
  void getBotVel(CAR_GOAL_POINT goalPoint,CAR_KPS_MAX carKps);

 private:
  WheelOdometry *_odometry;
};



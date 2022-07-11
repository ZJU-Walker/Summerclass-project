/**
 * @file SunKinematics.h
 * @author igcxl (igcxl@qq.com)
 * @brief 移动机器人运动学库
 * @version 0.2
 * @date 2020-07-24
 * 方向定义
 *            ^+X
 *            |
 *            |
 * +Y<--------|
 * kinematics

Arduino Kinematics library for differential drive(2WD, 4WD) and mecanum drive
robots.

The library requires the following robot's specification as an input:
•Robot's maximum RPM
•Distance between wheels (Front-Left and Front-Rear )
•Wheel's Diameter

Functions

1. Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter, float
base_width, int pwm_bits)

Object constructor which requires the robot's specification.
•MOTOR_MAX_RPM : Maximum RPM of the motor
•WHEEL_DIAMETER : Robot wheel's diameter expressed in meters
•FR_WHEEL_DISTANCE : Distance between front wheel and rear wheel
•LR_WHEEL_DISTANCE : Distance between left wheel and right wheel
•PWM_BITS : PWM resolution of the Microcontroller. Arduino Uno/Mega, Teensy is 8
bits by default

2. output getRPM(float linear_x, float linear_y, float angular_z)

Returns a Vector of Motor RPMs from a given linear velocity in x and y axis and
angular velocity in z axis using right hand rule. The returned values can be
used in a PID controller as "setpoint" vs a wheel encoder's feedback expressed
in RPM. •linear_x : target linear speed of the robot in x axis (forward or
reverse) expressed in m/s. •linear_y : target linear speed of the robot in y
axis (strafing left or strafing right for mecanum drive) expressed in m/s.
•angular_z : target angular speed of the robot in z axis (rotating CCW or CW)
rad/sec.

3. output getPWM(float linear_x, float linear_y, float angular_z)

The same output as getRPM() function converted to a PWM value. The returned
values can be used to drive motor drivers using the PWM signals.

4. velocities getVelocities(int motor1, int motor2)

This is the inverse of getRPM(). Returns linear velocities in x and y axis, and
angular velocity in z axis given two measured RPMs on each motor of a 2WD robot.
The returned values can be used to calculate the distance traveled in a specific
axis - where distance traveled is the product of the change in velocity and
change in time. •motor1: left motor's measured RPM •motor2: right motor's
measured RPM

*each motor's RPM value must be signed. + to signify forward direction and - to
signify reverse direction of the motor.

5. velocities getVelocities(int motor1, int motor2, int motor3, int motor4)

The same output as No.4 but requires 4 measured RPMs on each motor of a 4WD
robot. This can be used for both 4 wheeled differential drive and mecanum drive
robots. •motor1: front left motor's measured RPM •motor2: front right motor's
measured RPM •motor3: front left motor's measured RPM •motor4: front right
motor's measured RPM

*each motor's RPM value must be signed. + to signify forward direction and - to
signify reverse direction of the motor.

Data structures

1. output

Struct returned by getRPM() and getPWM used to store PWM or RPM values.
struct output{
  int motor1;
  int motor2;
  int motor3;
  int motor4;
};

•each returned motor RPM or PWM value is signed to signify the motor's
direction. + forward direction ; - reverse direction.

2. velocities

Struct returned by getVelocities() used to store linear velocities in x and y
axis, and angular velocity in z axis (right hand rule). struct velocities{ float
linear_x; float linear_y; float angular_z;
};

•linear_x and linear_y are expressed in m/s. angular_z is expressed in rad/s

 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include "Arduino.h"

//#define OMINKINE 0.8660254//sqrt(3)/2
#ifndef PWM_BITS
#define PWM_BITS 8  // PWM Resolution of the microcontroller
#endif
#ifndef PWM_MAX
#define PWM_MAX 255
#endif

#ifndef COUNTS_PER_REV
#define COUNTS_PER_REV 1560
#endif
#ifndef TIMER_PERIOD
#define TIMER_PERIOD 10
#endif

class Kinematics {
 public:
  enum Base {
    BALL = 1,
    BLANCE2,
    DIFFERENTIAL_DRIVE3,
    OMNI3,
    ACKERMAN,
    MECANUM_X,
    SKID4WD,
    OMNI_X,
    DEFAULT_2WD_4WD
  };
  Base base_type;
  struct output  //输出电机速度结构体
  {
    int motor1;
    int motor2;
    int motor3;
    int motor4;
  };

  struct velocities {
    float linear_x;   // x方向速度 m/s
    float linear_y;   // y方向速度 m/s
    float angular_z;  // z方向角速度 rad/s
  };

  Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist,
             float lr_wheels_dist);
  Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist,
             float lr_wheels_dist, Base base_type);
  Kinematics(int motor_max_rpm, float wheel_diameter, float center_wheels_dist,
             Base base_type);
  //正运动学 两轮
  velocities calculateVelocities(int motor1, int motor2);
  //正运动学 3轮全向
  velocities calculateVelocities(int motor1, int motor2, int motor3);
  ///正运动学 4WD 麦克纳姆轮正运动学 输入轮速rpm 输出小车速度
  velocities calculateVelocities(int motor1, int motor2, int motor3,
                                 int motor4);
  //正运动学 麦克纳姆轮正运动学 输入编码器脉冲数 输出小车速度
  velocities pulsesCalculateVelocities(int pulses1, int pulses2, int pulses3);
  //正运动学 麦克纳姆轮正运动学 输入编码器脉冲数 输出小车速度
  velocities pulsesCalculateVelocities(int pulses1, int pulses2, int pulses3,
                                       int pulses4);
  //逆运动学
  output getRPM(float linear_x, float linear_y, float angular_z);
  output getPWM(float linear_x, float linear_y, float angular_z);
  output getPulses(float linear_x, float linear_y, float angular_z);
  int rpmToPWM(int rpm);
  int rpmToPulses(int rpm);  //转速转换为电机编码器对应脉冲计数
  int pulsesToRpm(int pulses);  //电机编码器对应脉冲计数转换为转速
 private:
  float linear_vel_x_mins_;
  float linear_vel_y_mins_;
  float angular_vel_z_mins_;
  float circumference_;  //轮子周长
  float tangential_vel_;
  float x_rpm_;
  float y_rpm_;
  float tan_rpm_;
  int max_rpm_;
  float fr_wheels_dist_;     //轴距
  float lr_wheels_dist_;     //轮距
  float center_wheels_dist;  //中心距 三轮
  float pwm_res_;
};

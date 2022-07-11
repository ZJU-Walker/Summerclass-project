
#include "SunKinematics.h"

#include "math.h"

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter,
                       float fr_wheels_dist, float lr_wheels_dist)
    : circumference_(PI * wheel_diameter),
      max_rpm_(motor_max_rpm),
      fr_wheels_dist_(fr_wheels_dist),
      lr_wheels_dist_(lr_wheels_dist),
      base_type(DEFAULT_2WD_4WD) {}  //兼容性考虑，保留

Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter,
                       float fr_wheels_dist, float lr_wheels_dist,
                       Base base_type)
    : circumference_(PI * wheel_diameter),
      max_rpm_(motor_max_rpm),
      fr_wheels_dist_(fr_wheels_dist),
      lr_wheels_dist_(lr_wheels_dist),
      base_type(base_type) {}
//三轮全向
Kinematics::Kinematics(int motor_max_rpm, float wheel_diameter,
                       float center_wheels_dist, Base base_type)
    : circumference_(PI * wheel_diameter),
      max_rpm_(motor_max_rpm),
      lr_wheels_dist_(center_wheels_dist),
      base_type(base_type) {}

/**
 * @brief
 *
 * @param linear_x
 * @param linear_y
 * @param angular_z
 * @return Kinematics::output 逆运动学解算
 * 增加限幅
 */
Kinematics::output Kinematics::getRPM(float linear_x, float linear_y,
                                      float angular_z) {
  // convert m/s to m/min 转换为 米/分钟
  linear_vel_x_mins_ = linear_x * 60;
  linear_vel_y_mins_ = linear_y * 60;
  // convert rad/s to rad/min
  angular_vel_z_mins_ = angular_z * 60;
  Kinematics::output rpm;
  switch (base_type) {
    case MECANUM_X:
      // Vt = ω * radius 转换为车轮轮心处切向线速度？
      tangential_vel_ =
          angular_vel_z_mins_ * ((lr_wheels_dist_ / 2) + (fr_wheels_dist_ / 2));

      x_rpm_ = linear_vel_x_mins_ / circumference_;
      y_rpm_ = linear_vel_y_mins_ / circumference_;
      tan_rpm_ = tangential_vel_ / circumference_;

      // calculate for the target motor RPM and direction
      // front-left motor
      rpm.motor1 = x_rpm_ - y_rpm_ - tan_rpm_;
      rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);
      // rear-left motor
      rpm.motor3 = x_rpm_ + y_rpm_ - tan_rpm_;
      rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

      // front-right motor
      rpm.motor2 = x_rpm_ + y_rpm_ + tan_rpm_;
      rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

      // rear-right motor
      rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;
      rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

      break;
    case OMNI3:
      // Vt = ω * radius 转换为轮心处切向线速度
      tangential_vel_ = angular_vel_z_mins_ * lr_wheels_dist_;

      x_rpm_ = linear_vel_x_mins_ / circumference_;
      y_rpm_ = linear_vel_y_mins_ / circumference_;
      tan_rpm_ = tangential_vel_ / circumference_;
      // calculate for the target motor RPM and direction
      // front-A motor1
      rpm.motor1 = y_rpm_ + tan_rpm_;
      rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);
      // rear-left B motor2
      rpm.motor2 = -sqrt(3) * x_rpm_ / 2 - 0.5 * y_rpm_ + tan_rpm_;
      rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);
      // rear-right C motor3
      rpm.motor3 = sqrt(3) * x_rpm_ / 2 - 0.5 * y_rpm_ + tan_rpm_;
      rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);
      // out of use
      rpm.motor4 = 0;
      rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
      break;
    default:
      // Vt = ω * radius 转换为轮心的线速度 
      tangential_vel_ =
          angular_vel_z_mins_ * ((lr_wheels_dist_ / 2) + (fr_wheels_dist_ / 2));

      x_rpm_ = linear_vel_x_mins_ / circumference_;
      y_rpm_ = linear_vel_y_mins_ / circumference_;
      tan_rpm_ = tangential_vel_ / circumference_;

      // calculate for the target motor RPM and direction
      // front-left motor
      rpm.motor1 = x_rpm_ - y_rpm_ - tan_rpm_;
      rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);
      // rear-left motor
      rpm.motor3 = x_rpm_ + y_rpm_ - tan_rpm_;
      rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

      // front-right motor
      rpm.motor2 = x_rpm_ + y_rpm_ + tan_rpm_;
      rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

      // rear-right motor
      rpm.motor4 = x_rpm_ - y_rpm_ + tan_rpm_;
      rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

      break;
  }

  return rpm;
}

Kinematics::output Kinematics::getPWM(float linear_x, float linear_y,
                                      float angular_z) {
  Kinematics::output rpm;
  Kinematics::output pwm;

  rpm = getRPM(linear_x, linear_y, angular_z);

  // convert from RPM to PWM
  // front-left motor
  pwm.motor1 = rpmToPWM(rpm.motor1);
  // rear-left motor
  pwm.motor2 = rpmToPWM(rpm.motor2);

  // front-right motor
  pwm.motor3 = rpmToPWM(rpm.motor3);
  // rear-right motor
  pwm.motor4 = rpmToPWM(rpm.motor4);

  return pwm;
}
Kinematics::output Kinematics::getPulses(float linear_x, float linear_y,
                                         float angular_z) {
  Kinematics::output rpm;
  Kinematics::output pulses;

  rpm = getRPM(linear_x, linear_y, angular_z);

  // convert from RPM to PWM
  // front-left motor
  pulses.motor1 = rpmToPulses(rpm.motor1);
  // rear-left motor
  pulses.motor2 = rpmToPulses(rpm.motor2);

  // front-right motor
  pulses.motor3 = rpmToPulses(rpm.motor3);
  // rear-right motor
  pulses.motor4 = rpmToPulses(rpm.motor4);

  return pulses;
}
//两轮差速正运动学
Kinematics::velocities Kinematics::calculateVelocities(int motor1, int motor2) {
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2) / 2;  // RPM
  // convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60;         // RPS
  vel.linear_x = (average_rps_x * circumference_);  // m/s

  float average_rpm_a = (float)(motor2 - motor1) / 2;
  // convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z = (average_rps_a * circumference_) / (lr_wheels_dist_ / 2);

  return vel;
}

/**
 * @brief 三轮全向车正运动学
 * @param motor1
 * @param motor2
 * @param motor3
 * @return Kinematics::velocities
 */

Kinematics::velocities Kinematics::calculateVelocities(int motor1, int motor2,
                                                       int motor3) {
  Kinematics::velocities vel;

  float average_rpm_x =
      (float)(-sqrt(3) * motor2 + sqrt(3) * motor3) / 3;  // RPM
  // convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60;         // RPS
  vel.linear_x = (average_rps_x * circumference_);  // m/s

  float average_rpm_y = (float)(2 * motor1 - motor2 - motor3) / 3;  // RPM
  // convert revolutions per minute in y axis to revolutions per second
  float average_rps_y = average_rpm_y / 60;         // RPS
  vel.linear_y = (average_rps_y * circumference_);  // m/s

  float average_rpm_a = (float)(motor1 + motor2 + motor3) / 3;
  // convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z = (average_rps_a * circumference_) / lr_wheels_dist_;

  return vel;
}
//麦克纳姆轮正运动学 输入四轮转速RPM 输出小车速度
Kinematics::velocities Kinematics::calculateVelocities(int motor1, int motor2,
                                                       int motor3, int motor4) {
  Kinematics::velocities vel;

  float average_rpm_x = (float)(motor1 + motor2 + motor3 + motor4) / 4;  // RPM
  // convert revolutions per minute to revolutions per second
  float average_rps_x = average_rpm_x / 60;         // RPS
  vel.linear_x = (average_rps_x * circumference_);  // m/s

  float average_rpm_y = (float)(-motor1 + motor2 + motor3 - motor4) / 4;  // RPM
  // convert revolutions per minute in y axis to revolutions per second
  float average_rps_y = average_rpm_y / 60;         // RPS
  vel.linear_y = (average_rps_y * circumference_);  // m/s

  float average_rpm_a = (float)(-motor1 + motor2 - motor3 + motor4) / 4;
  // convert revolutions per minute to revolutions per second
  float average_rps_a = average_rpm_a / 60;
  vel.angular_z = (average_rps_a * circumference_) /
                  ((fr_wheels_dist_ / 2) + (lr_wheels_dist_ / 2));

  return vel;
}
Kinematics::velocities Kinematics::pulsesCalculateVelocities(int pulses1,
                                                             int pulses2,
                                                             int pulses3) {
  Kinematics::velocities velp;
  Kinematics::output rpm = {pulsesToRpm(pulses1), pulsesToRpm(pulses2),
                            pulsesToRpm(pulses3)};
  velp = calculateVelocities(rpm.motor1, rpm.motor2, rpm.motor3);

  return velp;
}
Kinematics::velocities Kinematics::pulsesCalculateVelocities(int pulses1,
                                                             int pulses2,
                                                             int pulses3,
                                                             int pulses4) {
  Kinematics::velocities velp;
  Kinematics::output rpm = {pulsesToRpm(pulses1), pulsesToRpm(pulses2),
                            pulsesToRpm(pulses3), pulsesToRpm(pulses4)};
  velp = calculateVelocities(rpm.motor1, rpm.motor2, rpm.motor3, rpm.motor4);

  return velp;
}
//rpm映射成PWM
int Kinematics::rpmToPWM(int rpm) {
  // remap scale of target RPM vs MAX_RPM to PWM
  return (((float)rpm / (float)max_rpm_) * (float)PWM_MAX);
}

/**
 * @brief rpm转速转变为每个定时中断周期TIMER_PERIOD内的编码器计数
 *       Pulses = rpm/60*COUNTS_PER_REV*TIMER_PERIOD/1000
 * @param rpm 转速
 * @return int 编码器计数
 */
int Kinematics::rpmToPulses(int rpm) {
  return ((((float)rpm * COUNTS_PER_REV) * TIMER_PERIOD) / 60000);
}

/**
 * @brief 每个定时中断周期TIMER_PERIOD内的编码器计数转变为转速rpm
 *        rpm=pulses/COUNTS_PER_REV/TIMER_PERIOD*60000
 * @param pulses  编码器计数
 * @return int 转速
 */

int Kinematics::pulsesToRpm(int pulses) {
  return (((float)pulses / COUNTS_PER_REV / TIMER_PERIOD) * 60000);
}
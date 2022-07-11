/**
 * @brief Source:
 *
 */

#include "SunCar.hpp"
#include "math.h"
//构造函数

Car::Car(WheelOdometry *odometry) : _odometry(odometry) {}

//输入目标位姿 得到世界坐标系下的速度
void Car::getWorldVel(CAR_GOAL_POINT goalPoint, CAR_KPS_MAX carKps) {
  //系统误差：被控量（输出量）目标值-实际值

  worldVel.vel_x =
      carKps.kx * (goalPoint.world_x - _odometry->botPosition.position_x);
  worldVel.vel_y =
      carKps.ky * (goalPoint.world_y - _odometry->botPosition.position_y);
  worldVel.angular_vel_z = carKps.kz * (goalPoint.world_angular_z -
                                        _odometry->botPosition.heading_theta);
}

void Car::getBotVel(CAR_GOAL_POINT goalPoint, CAR_KPS_MAX carKps) {
  getWorldVel(goalPoint, carKps);
  botVel.vel_x = cos(_odometry->botPosition.heading_theta) * worldVel.vel_x +
                 sin(_odometry->botPosition.heading_theta) * worldVel.vel_y;
  botVel.vel_y = -sin(_odometry->botPosition.heading_theta) * worldVel.vel_x +
                 cos(_odometry->botPosition.heading_theta) * worldVel.vel_y;
  botVel.angular_vel_z = worldVel.angular_vel_z;
  //限幅 0.3m/s 3rad/s
  botVel.vel_x = constrain(botVel.vel_x, -carKps.MAX_x, carKps.MAX_x);
  botVel.vel_y = constrain(botVel.vel_y, -carKps.MAX_y, carKps.MAX_y);
  botVel.angular_vel_z =
      constrain(botVel.angular_vel_z, -carKps.MAX_z, carKps.MAX_z);
}

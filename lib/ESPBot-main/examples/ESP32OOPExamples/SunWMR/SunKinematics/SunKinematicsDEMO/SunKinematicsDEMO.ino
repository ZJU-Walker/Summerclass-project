
#include "SunConfig.h"


/*
 Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist,
float lr_wheels_dist, int pwm_bits);
motor_max_rpm = motor's maximum rpm 电机最大转速
wheel_diameter = robot's wheel diameter expressed in meters 车轮直径
fr_wheels_dist FR_WHEELS_DISTANCE 轴距
lr_wheels_dist LR_WHEELS_DISTANCE = distance between two wheels expressed in
 meters 轮距 
 pwm_bits = microcontroller's PWM pin resolution. Arduino Uno/Mega Teensy is using 8 bits(0-255)
*/
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);

void setup() { Serial.begin(BAUDRATE); }

void loop() {
  Kinematics::output rpm;
  Kinematics::output pluses;

  // simulated required velocities
  float linear_vel_x = 0.5;  // 1 m/s
  float linear_vel_y = 0;    // 0 m/s
  float angular_vel_z = 0;   // 1 rad/s

  // given the required velocities for the robot, you can calculate the rpm
  // required for each motor
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
  Serial.print(" FRONT LEFT MOTOR: ");
  // Assuming you have an encoder for each wheel, you can pass this RPM value to
  // a PID controller as a setpoint and your encoder data as a feedback.
  Serial.print(rpm.motor1);
  Serial.print(",");
  Serial.print(pluses.motor1);
  Serial.print(" FRONT RIGHT MOTOR: ");
  Serial.print(rpm.motor2);
  Serial.print(",");
  Serial.print(pluses.motor2);
  Serial.print(" REAR LEFT MOTOR: ");
  Serial.print(rpm.motor3);
  Serial.print(",");
  Serial.print(pluses.motor3);
  Serial.print(" REAR RIGHT MOTOR: ");
  Serial.println(rpm.motor4);
  Serial.print(",");
  Serial.print(pluses.motor4);
  delay(4000);

  // This is a simulated feedback from each motor. We'll just pass the
  // calculated rpm above for demo's sake. In a live robot, these should be
  // replaced with real RPM values derived from encoder.
  int motor1_feedback = rpm.motor1;  // in rpm
  int motor2_feedback = rpm.motor2;  // in rpm
  int motor3_feedback = rpm.motor3;  // in rpm
  int motor4_feedback = rpm.motor4;  // in rpm

  Kinematics::velocities vel;

  // Now given the RPM from each wheel, you can calculate the linear and angular
  // velocity of the robot. This is useful if you want to create an odometry
  // data (dead reckoning)
  vel = kinematics.calculateVelocities(motor1_feedback, motor2_feedback,
                                       motor3_feedback, motor4_feedback);
  Serial.print(" VEL X: ");
  Serial.print(vel.linear_x, 4);

  Serial.print(" VEL_Y: ");
  Serial.print(vel.linear_y, 4);

  Serial.print(" ANGULAR_Z: ");
  Serial.println(vel.angular_z, 4);
  Serial.println("");
}


//走矩形线路测试
// 20210406更新 使用枚举类型 imu
#include "Wire.h"
#include <MPU6050_light.h>
#include <FlexiTimer2.h>  //定时中断
// 请先安装FlexiTimer2库,https://playground.arduino.cc/Main/FlexiTimer2/

#include "SunConfig.h"

//#define DEBUG
/*
 Kinematics(int motor_max_rpm, float wheel_diameter, float fr_wheels_dist,
float lr_wheels_dist, int pwm_bits);
motor_max_rpm = motor's maximum rpm 电机最大转速
wheel_diameter = robot's wheel diameter expressed in meters 车轮直径
fr_wheels_dist FR_WHEELS_DISTANCE 轴距
lr_wheels_dist LR_WHEELS_DISTANCE = distance between two wheels expressed in
 meters 轮距 pwm_bits = microcontroller's PWM pin resolution. Arduino Uno/Mega
 Teensy is using 8 bits(0-255)
*/
//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
//新建小车电机实例
A4950MotorShield motors;

long newPulses[4] = {0, 0, 0, 0};
//新建小车编码器实例
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
float targetPulses[4] = {0};
float Kp = 10, Ki = 0.1, Kd = 0;
double outPWM[4] = {0, 0, 0, 0};
//新建小车速度PID实例
PID VeloPID[4] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;            // m/s
float linear_vel_y = 0;            // m/s
float angular_vel_z = 0;           // rad/s
unsigned long previousMillis = 0;  // will store last time run
const long period = 5000;          // period at which to run in ms

//运行状态标记
enum CARMOTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD };
enum CARMOTION direction = PAUSE;

void control() {
  sei();  //全局中断开启

  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  //获取电机速度

#ifdef PINS_REVERSE
  newPulses[0] = -ENC[0].read();  // A
  newPulses[1] = ENC[1].read();   // B
  newPulses[2] = -ENC[2].read();  // C
  newPulses[3] = ENC[3].read();   // D
#else
  newPulses[0] = ENC[0].read();   // A
  newPulses[1] = -ENC[1].read();  // B
  newPulses[2] = ENC[2].read();   // C
  newPulses[3] = -ENC[3].read();  // D
#endif

  for (int i = 0; i < WHEEL_NUM; i++) {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
    ENC[i].write(0);  //复位
  }
  //设置电机速度
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
  // static int print_Count;
  //  if (++print_Count >= 10) //打印控制，控制周期100ms
  //  {
  // // Serial.println(millis());//显示
  //  Serial.print(Velocity_A);//显示
  //  Serial.print(",");
  // Serial.print(Velocity_B);//显示
  // Serial.print(",");
  //  Serial.print(Velocity_C);//显示
  //  Serial.print(",");
  //  Serial.println(Velocity_D);//显示
  //  //Serial.println(iConstrain);
  //    print_Count = 0;
  //  }
}

MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup() {
  motors.init();
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  delay(100);                               //延时等待初始化完成
  FlexiTimer2::set(TIMER_PERIOD, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();                     //中断使能
  delay(100);                               //延时等待初始化完成
  Serial.begin(BAUDRATE);
  Serial.println("Sunnybot Basic  Mecanum Rectangle Test:");
}
void loop() {
  unsigned long currentMillis = millis();  // store the current time
  //使用有限状态机方式走正方形
  // simulated required velocities
  // PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD
  switch (direction) {
    case PAUSE:           //停止
      linear_vel_x = 0;   // m/s
      linear_vel_y = 0;   // m/s
      angular_vel_z = 0;  // rad/s
      //使用millis函数进行定时控制，代替delay函数
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        direction = LEFTWARD;
      }
      break;
    case LEFTWARD:         //左进
      linear_vel_x = 0;    // m/s
      linear_vel_y = 0.2;  // m/s
      angular_vel_z = 0;   // rad/s
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        direction = FORWARD;
      }
      break;
    case FORWARD:          //前进
      linear_vel_x = 0.2;  // m/s
      linear_vel_y = 0;    // m/s
      angular_vel_z = 0;   // rad/s
      if (currentMillis - previousMillis >= (2 * period)) {
        previousMillis = currentMillis;
        direction = RIGHTWARD;
      }
      break;
    case RIGHTWARD:         //右进
      linear_vel_x = 0;     // m/s
      linear_vel_y = -0.2;  // m/s
      angular_vel_z = 0;    // rad/s
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        direction = BACKWARD;
      }
      break;
    case BACKWARD:          //后退
      linear_vel_x = -0.2;  // m/s
      linear_vel_y = 0;     // m/s
      angular_vel_z = 0;    // rad/s
      if (currentMillis - previousMillis >= (2 * period)) {
        previousMillis = currentMillis;
        direction = PAUSE;
      }
      break;
    default:              //停止
      linear_vel_x = 0;   // m/s
      linear_vel_y = 0;   // m/s
      angular_vel_z = 0;  // rad/s
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        direction = PAUSE;
      }
      break;
  }

  // given the required velocities for the robot, you can calculate
  // the rpm required for each motor
  //
//mpu.fetchData();
//angular_vel_z=-mpu.getGyroZ()*0.0174;
mpu.update();
angular_vel_z=-mpu.getAngleZ()*0.0174*5;
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
#ifndef DEBUG
  Serial.print(" FRONT LEFT MOTOR: ");
  // Assuming you have an encoder for each wheel, you can pass this
  // RPM value to a PID controller as a setpoint and your encoder
  // data as a feedback.
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
  Serial.println(pluses.motor4);
#else
  //串口绘图器输出
  Serial.print(" FL: ");
  Serial.print(pluses.motor1);
  Serial.print(",");
  Serial.print(" FR: ");
  Serial.print(pluses.motor2);
  Serial.print(",");
  Serial.print(" RL: ");
  Serial.print(pluses.motor3);
  Serial.print(",");
  Serial.print(" RR: ");
  Serial.print(pluses.motor4);
  Serial.print(",");
#endif

  Kinematics::velocities vel;
  // This is a simulated feedback from each motor. We'll just pass
  // the calculated rpm above for demo's sake. In a live robot,
  // these should be replaced with real RPM values derived from
  // encoder.
  int motor1_feedback = newPulses[0];  // in rpm
  int motor2_feedback = newPulses[1];  // in rpm
  int motor3_feedback = newPulses[2];  // in rpm
  int motor4_feedback = newPulses[3];  // in rpm

#ifndef DEBUG
  Serial.print(" FRONT LEFT MOTOR: ");
  // Assuming you have an encoder for each wheel, you can pass this
  // RPM value to a PID controller as a setpoint and your encoder
  // data as a feedback.
  Serial.print(motor1_feedback * 3.85);
  Serial.print(",");
  Serial.print(newPulses[0]);
  Serial.print(" FRONT RIGHT MOTOR: ");
  Serial.print(motor2_feedback * 3.85);
  Serial.print(",");
  Serial.print(newPulses[1]);
  Serial.print(" REAR LEFT MOTOR: ");
  Serial.print(motor3_feedback * 3.85);
  Serial.print(",");
  Serial.print(newPulses[2]);
  Serial.print(" REAR RIGHT MOTOR: ");
  Serial.println(motor4_feedback * 3.85);
  Serial.print(",");
  Serial.println(newPulses[3]);
#else
  Serial.print(" FLF: ");
  Serial.print(newPulses[0]);
  Serial.print(",");
  Serial.print(" FRF: ");
  Serial.print(newPulses[1]);
  Serial.print(",");
  Serial.print(" RLF: ");
  Serial.print(newPulses[2]);
  Serial.print(",");
  Serial.print(" RRF: ");
  Serial.print(newPulses[3]);
  Serial.print(",");
  Serial.print(" FLP: ");
  Serial.print(outPWM[0]);
  Serial.print(",");
  Serial.print(" FRP: ");
  Serial.print(outPWM[1]);
  Serial.print(",");
  Serial.print(" RLP: ");
  Serial.print(outPWM[2]);
  Serial.print(",");
  Serial.print(" RRP: ");
  Serial.println(outPWM[3]);

#endif


  // Now given the RPM from each wheel, you can calculate the linear
  // and angular velocity of the robot. This is useful if you want
  // to create an odometry data (dead reckoning)
#ifndef DEBUG
  vel = kinematics.pulsesCalculateVelocities(motor1_feedback, motor2_feedback,
                                             motor3_feedback, motor4_feedback);
  Serial.print(" VEL X: ");
  Serial.print(vel.linear_x, 4);

  Serial.print(" VEL_Y: ");
  Serial.print(vel.linear_y, 4);

  Serial.print(" ANGULAR_Z: ");
  Serial.println(vel.angular_z, 4);
  Serial.println("");
#endif
}

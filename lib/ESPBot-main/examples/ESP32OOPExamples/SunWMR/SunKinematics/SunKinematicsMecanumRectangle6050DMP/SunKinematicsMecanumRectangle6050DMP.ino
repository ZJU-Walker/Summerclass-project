//走矩形线路测试
//运行前先校准陀螺仪
// 20210406更新 使用枚举类型 imu
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <FlexiTimer2.h>  //定时中断
// 请先安装FlexiTimer2库,https://playground.arduino.cc/Main/FlexiTimer2/

#include "SunConfig.h"
MPU6050 mpu;
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
//建立4个编码器脉冲数变量
long newPulses[4] = {0, 0, 0, 0};
//新建小车编码器实例
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
float targetPulses[4] = {0};
float Kp = 10, Ki = 0.1, Kd = 0;
double outPWM[4] = {0, 0, 0, 0};
//新建4个电机速度PID实例
PID VeloPID[4] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;   //附体坐标系下x方向速度，单位 m/s
float linear_vel_y = 0;   //附体坐标系下y方向速度，单位 m/s
float angular_vel_z = 0;  //附体坐标系下z方向角速度，单位 rad/s
unsigned long previousMillis = 0;  // will store last time run
const long period = 5000;          // period at which to run in ms

//运行状态标记 c++ enum实际上并没有创建任何变量，只是定义数据类型。
enum CARMOTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD };
CARMOTION direction = PAUSE;

void control() {
  sei();  //全局中断开启
  //设置各电机目标脉冲数（速度）=逆运动学解算各电机脉冲数（速度）
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;

  //获取各电机实测脉冲数（速度），然后置零
#ifdef PINS_REVERSE
  newPulses[0] = -ENC[0].readAndReset();  // A
  newPulses[1] = ENC[1].readAndReset();   // B
  newPulses[2] = -ENC[2].readAndReset();  // C
  newPulses[3] = ENC[3].readAndReset();   // D
#else
  newPulses[0] = ENC[0].readAndReset();   // A
  newPulses[1] = -ENC[1].readAndReset();  // B
  newPulses[2] = ENC[2].readAndReset();   // C
  newPulses[3] = -ENC[3].readAndReset();  // D
#endif
  //通过pid控制器输出各电机PWM,并复位编码器计数器
  for (int i = 0; i < WHEEL_NUM; i++) {
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], (float)newPulses[i]);
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
//陀螺仪变量
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container 四元数容器
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16 gy;  // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
  //电机初始化
  motors.init();
  //陀螺仪IIC初始化
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having
                          // compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  /*
  mpu.setXAccelOffset(-1656);
  mpu.setYAccelOffset(-5074);
  mpu.setZAccelOffset(838);
  mpu.setXGyroOffset(-36);
  mpu.setYGyroOffset(15);
  mpu.setZGyroOffset(78);
   mpu.setXAccelOffset(-783);
  mpu.setYAccelOffset(-873);
  mpu.setZAccelOffset(1085);
  mpu.setXGyroOffset(-252);
  mpu.setYGyroOffset(-25);
  mpu.setZGyroOffset(-24);
  */
  mpu.setXAccelOffset(-550);
  mpu.setYAccelOffset(-2144);
  mpu.setZAccelOffset(1016);
  mpu.setXGyroOffset(-30);
  mpu.setYGyroOffset(49);
  mpu.setZGyroOffset(50);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  delay(100);                               //延时等待初始化完成
  FlexiTimer2::set(TIMER_PERIOD, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();                     //中断使能
  delay(100);                               //延时等待初始化完成

  Serial.println("Sunnybot Basic  Mecanum Rectangle Test:");
}
void loop() {
  unsigned long currentMillis = millis();  // store the current time
  //使用有限状态机方式走正方形
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
  // mpu.fetchData();
  // angular_vel_z=-mpu.getGyroZ()*0.0174;
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //ypr[0]弧度制
    angular_vel_z = ypr[0] * 5;
  }
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

// P2P点位运动/点镇定（Point-to-Point Motion/Pose Stabilization）
//走矩形线路测试 使用枚举类型
#include <Arduino.h>

#include "OOPConfig.h"  //请根据实际元器件修改配置文件，比如电机、车型
#ifdef USE_MPU6050_DMP
//使用带DMP的支持ESP32的MPU6050库
#include "I2Cdev.h"  //点击自动打开管理库页面并安装: http://librarymanager/All#MPU6050
#include "MPU6050_6Axis_MotionApps_V6_12.h"  //和I2Cdev.h同时安装了,原版I2Cdev无法使用
MPU6050 mpu;
#endif
#include <Ticker.h>  //定时中断
#include <Wire.h>

#define DEBUG  //是否开启串口调试

#ifdef USE_OLED
#include <U8g2lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping
#endif

//****************** MPU6050***************************//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16 gy;  // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//小车偏航角
float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad;
bool isFirst = 1;
float angle2rad(float angle) { return angle * M_PI / 180; }
float rad2angle(float rad) { return rad * 180 / M_PI; }
//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;
//新建小车里程计实例
WheelOdometry botOdometry(&kinematics);
//新建小车位置环P2P实例
Car mecanumXbot(&botOdometry);
//目标点结构体 X:mm Y:mm Z:弧度
CAR_GOAL_POINT p0{0, 0, 0};
CAR_GOAL_POINT p1{900, 900, 0};
CAR_GOAL_POINT p2{900, 900, 1.57};
//比例系数和速度最大值
CAR_KPS_MAX botKps{0.01, 0.01, 5, 0.3, 0.3, 3};
//运行状态标记
enum CARMOTION { P0, P1, P2 };
enum CARMOTION position = P0;

//小车附体坐标系（局部坐标系）下速度
float linear_vel_x = 0;   // m/s
float linear_vel_y = 0;   // m/s
float angular_vel_z = 0;  // rad/s
//运行周期
unsigned long previousMillis = 0;  // will store last time run
const long period = 5000;          // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1;  // 中断函数
bool timer_flag = 0; 
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

long targetPulses[WHEELS_NUM] = {0, 0, 0, 0};  //四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0,
                                   0};  //四个车轮的定时中断编码器四倍频计数
double outPWM[WHEELS_NUM] = {0, 0, 0, 0};

//*****************创建4个速度PID实例***************************//
float Kp = 10, Ki = 0.1, Kd = 0;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;
int print_Count = 0;

//定时器中断处理函数,核心函数
void timerISR() {
  //获取电机脉冲数（速度）
  timer_flag = 1;  //定时时间达到标志
  print_Count++;
  //  /获取电机目标速度计数=运动学解算理论数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  //复位
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  //里程计更新位置,融合陀螺仪偏转角数据
  botOdometry.getPositon_mm(feedbackPulses[0], feedbackPulses[1],
                            feedbackPulses[2], feedbackPulses[3], realYawRad);
  //电机更新速度
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup() {
  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]);  //根据实际转向进行调整false or true 黑色PCB电机
                       // false  绿色PCB电机true 翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++) {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);

// OLED初始化
#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("Initializing");
  u8g2.setCursor(0, 32);
  u8g2.print("DO NOT MOVE MPU");
  u8g2.sendBuffer();
#endif
  //陀螺仪初始化
  Wire.begin();
  // Wire.setClock(400000);
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
  //运行例程中的IMU_Zero获得，注意波特率一致问题
  mpu.setXAccelOffset(XAccelOffset);
  mpu.setYAccelOffset(YAccelOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setXGyroOffset(XGyroOffset);
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);
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

  delay(100);  //延时等待初始化完成

  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
#ifdef USE_OLED
  u8g2.clearBuffer();                          // clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  // choose a suitable font
  u8g2.setCursor(0, 32);
  u8g2.print("请按对应按键ON");  //汉字用print
  u8g2.sendBuffer();
#endif
  while (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }

  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR);  // 打开定时器中断
  interrupts();
}
void loop() {
  unsigned long currentMillis = millis();  // store the current time
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  newYawRad =
      -ypr[0];  //传感器库返回方向和定义方向相反，所以取负号，返回值为弧度
  if (isFirst) {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = newYawRad - initialYawRad;
  realYaw = rad2angle(realYawRad);
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);

  // P2P走点位
  switch (position) {
    case P0:  //停止
      mecanumXbot.getBotVel(p0, botKps);
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      //使用millis函数进行定时控制，代替delay函数
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        position = P1;
      }
      break;
    case P1:  //右转
      mecanumXbot.getBotVel(p1, botKps);
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        position = P2;
      }
      break;
    case P2:  //前进
      // mecanumXbot.getBotVel(p2, botKps);
      mecanumXbot.getBotVel(p0, botKps);
      linear_vel_x = mecanumXbot.botVel.vel_x;  // m/s
      linear_vel_y = mecanumXbot.botVel.vel_y;  // m/s
      // angular_vel_z = 0;  // rad/s
      angular_vel_z = mecanumXbot.botVel.angular_vel_z;
      if (currentMillis - previousMillis >= (period)) {
        previousMillis = currentMillis;
        position = P0;
      }
      break;

    default:             //停止
      linear_vel_x = 0;  // m/s
      linear_vel_y = 0;  // m/s
      angular_vel_z = 0;
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        position = P0;
      }
      break;
  }

  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
  if (print_Count >= 100)  //打印和显示控制，控制周期1000ms
  {
#ifdef USE_OLED
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("i:");
    u8g2.print(initialYaw);
    u8g2.print("n:");
    u8g2.print(newYaw);
    u8g2.setCursor(0, 32);
    u8g2.print("X:");
    u8g2.print(botOdometry.botPosition.position_x);
    u8g2.setCursor(0, 48);
    u8g2.print("Y:");
    u8g2.print(botOdometry.botPosition.position_y);
    u8g2.setCursor(0, 64);
    u8g2.print("Z:");
    u8g2.print(realYaw);
    u8g2.sendBuffer();
#endif
#ifdef DEBUG
    //串口输出目标值
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
    Serial.println(pluses.motor4);
    //串口输出反馈值
    Serial.print(" FLF: ");
    Serial.print(feedbackPulses[0]);
    Serial.print(",");
    Serial.print(" FRF: ");
    Serial.print(feedbackPulses[1]);
    Serial.print(",");
    Serial.print(" RLF: ");
    Serial.print(feedbackPulses[2]);
    Serial.print(",");
    Serial.print(" RRF: ");
    Serial.println(feedbackPulses[3]);
    //串口输出 IO输出PWM值
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
    //串口输出 里程计值
    Serial.print(" X: ");
    Serial.print(botOdometry.botPosition.position_x);
    Serial.print(",");
    Serial.print("Y: ");
    Serial.print(botOdometry.botPosition.position_y);
    Serial.print(",");
    Serial.print(" Z: ");
    Serial.println(botOdometry.botPosition.heading_theta);
#endif
    print_Count = 0;
  }
}

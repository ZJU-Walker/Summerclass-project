
//走矩形线路测试 使用枚举类型
#include <Arduino.h>
#include <MPU6050_tockn.h> //点击自动打开管理库页面并安装: http://librarymanager/All#MPU6050_tockn
#include <Wire.h>
#include <Ticker.h> //定时中断
#include "OOPConfig.h"
#ifdef USE_OLED
#include <U8g2lib.h>                                                                                        //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
#endif

#define DEBUG

MPU6050 mpu6050(Wire);
float initialYaw, newYaw;
bool isFirst = 1;
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
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;           // m/s
float linear_vel_y = 0;           // m/s
float angular_vel_z = 0;          // rad/s
unsigned long previousMillis = 0; // will store last time run
const long period = 5000;         // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1; // 中断函数
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

long targetPulses[WHEELS_NUM] = {0, 0, 0, 0}; //四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0,
                                   0}; //四个车轮的定时中断编码器四倍频计数
double outPWM[WHEELS_NUM] = {0, 0, 0, 0};

//*****************创建4个速度PID实例***************************//
/*PID(float min_val, float max_val, float kp, float ki, float kd)
 * float min_val = min output PID value
 * float max_val = max output PID value
 * float kp = PID - P constant PID控制的比例、积分、微分系数
 * float ki = PID - I constant
 * float di = PID - D constant
 * Input	(double)输入参数feedbackPulses，待控制的量
 * Output	(double)输出参数outPWM，指经过PID控制系统的输出量
 * Setpoint	(double)目标值targetPulses，希望达到的数值
 */
float Kp = 10, Ki = 0.1, Kd = 0;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态标记**************************//
enum CARMOTION
{
  PAUSE,
  LEFTWARD,
  FORWARD,
  RIGHTWARD,
  BACKWARD
};
enum CARMOTION direction = PAUSE;

int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR()
{
  //获取电机脉冲数（速度）
  timer_flag = 1; //定时时间达到标志
  print_Count++;
  //  /获取电机目标速度计数=数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++)
  {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0); //复位
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup()
{

  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]); //根据实际转向进行调整false or true 黑色PCB电机
                      // false  绿色PCB电机true 翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++)
  {
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
  u8g2.clearBuffer();             // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("calculating Offsets"); //汉字用print
  u8g2.setCursor(0, 32);
  u8g2.print("DO NOT MOVE MPU"); //汉字用print
  u8g2.sendBuffer();
#endif
  //陀螺仪初始化
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); //首次使用请记录串口补偿值填入下方
                                 // mpu6050.setGyroOffsets(-1.36, -0.49, 0.96);//已有补偿值时使用
  Serial.println("Sunnybot 麦轮走长方形测试，请按下对应按键开始测试");
#ifdef USE_OLED
  u8g2.clearBuffer();                         // clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2); // choose a suitable font
  u8g2.setCursor(0, 32);
  u8g2.print("请按对应按键ON"); //汉字用print
  u8g2.sendBuffer();
#endif
  while (digitalRead(BUTTON_PIN) == HIGH)
  {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
  //打印起始偏航角度
#ifdef USE_OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB12_tf); // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("rYaw:");
  mpu6050.update();
  u8g2.print(mpu6050.getAngleZ());
  u8g2.sendBuffer();
#endif
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR); // 打开定时器中断
  interrupts();
}
void loop()
{
  unsigned long currentMillis = millis(); // store the current time
  //使用有限状态机方式走正方形
  // PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD
  switch (direction)
  {
  case PAUSE:          //停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    //使用millis函数进行定时控制，代替delay函数
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = LEFTWARD;
    }
    break;
  case LEFTWARD:        //左进
    linear_vel_x = 0;   // m/s
    linear_vel_y = 0.2; // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = FORWARD;
    }
    break;
  case FORWARD:         //前进
    linear_vel_x = 0.2; // m/s
    linear_vel_y = 0;   // m/s
    angular_vel_z = 0;  // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      direction = RIGHTWARD;
    }
    break;
  case RIGHTWARD:        //右进
    linear_vel_x = 0;    // m/s
    linear_vel_y = -0.2; // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = BACKWARD;
    }
    break;
  case BACKWARD:         //后退
    linear_vel_x = -0.2; // m/s
    linear_vel_y = 0;    // m/s
    angular_vel_z = 0;   // rad/s
    if (currentMillis - previousMillis >= (period))
    {
      previousMillis = currentMillis;
      direction = PAUSE;
    }
    break;
  default:             //停止
    linear_vel_x = 0;  // m/s
    linear_vel_y = 0;  // m/s
    angular_vel_z = 0; // rad/s
    if (currentMillis - previousMillis >= period)
    {
      previousMillis = currentMillis;
      direction = PAUSE;
    }
    break;
  }
  mpu6050.update();
  newYaw = mpu6050.getAngleZ();
  if (isFirst)
  {
    initialYaw = newYaw;
    isFirst = 0;
  }
  //角速度比例环 vz=k*errZ; errZ=期望-实际
  angular_vel_z = (initialYaw - newYaw) * 0.4;

  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
  if (print_Count >= 50) //打印控制，控制周期500ms
  {
#ifdef USE_OLED
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf); // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("rYaw:"); //汉字要用print
    u8g2.print(initialYaw);
    u8g2.setCursor(0, 32);
    u8g2.print("nYaw:"); //汉字要用print
    u8g2.print(newYaw);
    u8g2.sendBuffer();
#endif
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
    print_Count = 0;
  }
}

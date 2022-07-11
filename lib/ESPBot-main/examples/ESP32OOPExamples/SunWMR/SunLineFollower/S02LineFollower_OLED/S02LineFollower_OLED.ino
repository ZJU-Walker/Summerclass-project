
//前后巡线测试
// 20220705更新 巡线测试

#include <Ticker.h>  //定时中断

#include "OOPConfig.h"

#ifdef USE_OLED
#include <U8g2lib.h>  //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL,
    /* data=*/SDA);  // ESP32 Thing, HW I2C with pin remapping
#endif
#ifndef DEBUG
//#define DEBUG
#endif
#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif
// 新建Grayscale 实例 默认参数
Grayscale GraySensors;
// GraySensors(/*Num*/7,/**uartPort*/&Serial2,/*isDarkHigh*/true,/*isOffset*/false);
// //7路灰度使用串口2 uart输出量的结构体
Grayscale::strOutput GrayUartOutputIO;

//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;            // m/s
float linear_vel_y = 0;            // m/s
float angular_vel_z = 0;           // rad/s
unsigned long previousMillis = 0;  // will store last time run
const long period = 3000;          // period at which to run in ms
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
float Kp = 15, Ki = 0.04, Kd = 0.01;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态枚举**************************//
enum CARMOTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD, STOP };
enum CARMOTION direction = PAUSE;
bool carDirection = 0;
unsigned long readTime, readPeriod, ISRPeriod;
int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据并刷新电机输出
void timerISR() {
  // ISRPeriod = micros();
  //获取电机脉冲数（速度）
  timer_flag = 1;  //定时时间达到标志
  print_Count++;
  //  /获取电机目标速度计数=数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  //复位
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
  // ISRPeriod = micros() - ISRPeriod;
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

  Serial.println("Sunnybot 麦轮寻线测试，请按下对应按键开始测试");
  // OLED初始化
#ifdef USE_OLED
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  u8g2.setCursor(0, 16);
  u8g2.print("Press the BTN");
  u8g2.sendBuffer();
#endif
  while (digitalRead(BUTTON_PIN) == HIGH) {
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
  }
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR);  // 打开定时器中断
  interrupts();

  readPeriod = micros();
}
void loop() {
  /*
 //方式1
 if (timer_flag)  // 10ms更新一次
 {
   readTime = micros();
   GrayUartOutputIO = GraySensors.readUart();  //读取串口数字量数据
   readTime = micros() - readTime;
   timer_flag = 0;
   // delay(1);
 }
  */

  //方式2
  if (micros() - readPeriod > 5000)  //每5ms读取一次
  {
    readPeriod = micros();
    readTime = micros();
    GrayUartOutputIO = GraySensors.readUart();  //读取串口数字量数据
    readTime = micros() - readTime;
  }
  //方式2

  unsigned long currentMillis = millis();  // store the current time
  float kp = 0.003;
#ifndef DEBUG
  kp = 0.002;  //
#endif

  //使用有限状态机方式前后走
  switch (direction) {
    case PAUSE:  //校正

      linear_vel_x = 0;  // m/s
      if (GrayUartOutputIO.ioCount) {
        linear_vel_y = -0.005 * GrayUartOutputIO.offset;  // m/s
      }
      angular_vel_z = 0;  // rad/s
      //使用millis函数进行定时控制，代替delay函数
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        if (carDirection) {
          direction = BACKWARD;

        } else
          direction = FORWARD;
      }
      break;

    case STOP:
      //急停
      motors.motorsBrake();
      if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis;
        direction = PAUSE;
      }
      break;
    case FORWARD:          //前进
      linear_vel_x = 0.3;  // m/s
      if (GrayUartOutputIO.ioCount) {
        linear_vel_y = -kp * GrayUartOutputIO.offset;
      } else {
        linear_vel_y = -linear_vel_y;
      }                   // m/s
      angular_vel_z = 0;  // rad/s
      carDirection = 1;
      if (currentMillis - previousMillis >= (2 * period)) {
        previousMillis = currentMillis;
        direction = STOP;
      }
      break;
    case BACKWARD:          //后退
      linear_vel_x = -0.3;  // m/s
      if (GrayUartOutputIO.ioCount) {
        linear_vel_y = -kp * GrayUartOutputIO.offset;  // m/s
      }
      carDirection = 0;
      angular_vel_z = 0;  // rad/s
      if (currentMillis - previousMillis >= (2 * period)) {
        previousMillis = currentMillis;
        direction = STOP;
      }
      break;
    default:             //停止
      linear_vel_x = 0;  // m/s
      if (GrayUartOutputIO.ioCount) {
        linear_vel_y = -kp * GrayUartOutputIO.offset;  // m/s
      }
      angular_vel_z = 0;  // rad/s
      if (currentMillis - previousMillis >= period) {
        previousMillis = currentMillis;
        direction = PAUSE;
      }
      break;
  }

  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
#ifdef DEBUG_INFO
  Serial.print((String) "黑色点数：" + GrayUartOutputIO.ioCount +
               "点变化数:" + GrayUartOutputIO.ioChangeCount + "偏移量：" +
               GrayUartOutputIO.offset);
  Serial.print("二进制IO值:");
  Serial.print(GrayUartOutputIO.ioDigital, BIN);
  Serial.print("ReadTime:");
  Serial.print(readTime);
  Serial.println("微秒");
  Serial.print("Time:");
  Serial.print(millis());
  // Serial.print("ISRPeriod:");
  // Serial.println(ISRPeriod);
#endif
  if (print_Count >= 50)  //打印控制，控制周期500ms
  {
#ifdef DEBUG
#ifdef USE_OLED
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
    u8g2.setCursor(0, 16);
    u8g2.print("offset:");
    u8g2.print(GrayUartOutputIO.offset);
    u8g2.setCursor(0, 32);
    u8g2.print("IOCnt:");
    u8g2.print(GrayUartOutputIO.ioCount);
    u8g2.setCursor(0, 48);
    u8g2.print("vel_x:");
    u8g2.print(linear_vel_x);
    u8g2.setCursor(0, 64);
    u8g2.print("vel_y:");
    u8g2.print(linear_vel_y);
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

#endif
    print_Count = 0;
  }
}

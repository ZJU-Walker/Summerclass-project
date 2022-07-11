#include <MPU6050_tockn.h>
#include <Ticker.h>
#include <Wire.h>

#include "analogWrite.h"  //模拟量读写库

MPU6050 mpu6050(Wire);
float angleY, angleZ, gyroY, gyroZ;

/***************** 定时中断参数 *****************/
Ticker timer1;               // 中断函数
int interruptTime = 10;      // 中断时间
int timer_flag = 0;          //定时器标志；
#define COUNTS_PER_REV 1560  // wheel encoder's num of ticks per rev 390*4=1560

volatile long M1EncoderCount, M2EncoderCount;  //编码器计数
float M1Velocity, M2Velocity;                  //电机速度

unsigned long lastTime;

//******************电机驱动PWM引脚***************************//

#define LeftMotorAIN1 \
  32  //左轮电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2 \
  33  //左轮机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1 \
  26  //右轮电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2 \
  25  //  右轮电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚

//******************编码器引脚***************************//

#define Motor1EncoderA 35  // A路电机编码器引脚AA，外部中断，中断号0
#define Motor1EncoderB 34  // A路电机编码器引脚AB

#define Motor2EncoderA 39  // B路电机编码器引脚BA，外部中断，中断号1
#define Motor2EncoderB 36  // B路电机编码器引脚BB

//******************变量***************************//
int M1_Speed, M2_Speed;
int FBValue, LRValue, commaIndex;

//******************电机死区值 **********************//
int motorDeadZone =
    0;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-255,255]；motorb  b电机驱动PWM[-255,255]
**************************************************************************/
void RunMotors(int motora, int motorb) {
  if (motora > 0)
    analogWrite(LeftMotorAIN2, motora + motorDeadZone),
        analogWrite(LeftMotorAIN1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(LeftMotorAIN2, 0), analogWrite(LeftMotorAIN1, 0);
  else if (motora < 0)
    analogWrite(LeftMotorAIN1, -motora + motorDeadZone),
        analogWrite(
            LeftMotorAIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(RightMotorBIN2, motorb + motorDeadZone),
        analogWrite(RightMotorBIN1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(RightMotorBIN2, 0), analogWrite(RightMotorBIN1, 0);
  else if (motorb < 0)
    analogWrite(RightMotorBIN1, -motorb + motorDeadZone),
        analogWrite(
            RightMotorBIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

/*****函数功能：外部中断读取M1编码器数据，注意外部中断是跳变沿触发********/
void readMotor1EncoderA() {
  if (digitalRead(Motor1EncoderA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(Motor1EncoderB) == LOW)
      M1EncoderCount--;  //根据另外一相电平判定方向
    else
      M1EncoderCount++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(Motor1EncoderB) == LOW)
      M1EncoderCount++;  //根据另外一相电平判定方向
    else
      M1EncoderCount--;
  }
}

void readMotor1EncoderB() {
  if (digitalRead(Motor1EncoderB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(Motor1EncoderA) == LOW)
      M1EncoderCount++;  //根据另外一相电平判定方向
    else
      M1EncoderCount--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(Motor1EncoderA) == LOW)
      M1EncoderCount--;  //根据另外一相电平判定方向
    else
      M1EncoderCount++;
  }
}
/*****函数功能：外部中断读取M2编码器数据，注意外部中断是跳变沿触发********/
void readMotor2EncoderA() {
  if (digitalRead(Motor2EncoderA) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(Motor2EncoderB) == LOW)
      M2EncoderCount--;  //根据另外一相电平判定方向
    else
      M2EncoderCount++;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(Motor2EncoderB) == LOW)
      M2EncoderCount++;  //根据另外一相电平判定方向
    else
      M2EncoderCount--;
  }
}

void readMotor2EncoderB() {
  if (digitalRead(Motor2EncoderB) == LOW) {  //如果是下降沿触发的中断
    if (digitalRead(Motor2EncoderA) == LOW)
      M2EncoderCount++;  //根据另外一相电平判定方向
    else
      M2EncoderCount--;
  } else {  //如果是上升沿触发的中断
    if (digitalRead(Motor2EncoderA) == LOW)
      M2EncoderCount--;  //根据另外一相电平判定方向
    else
      M2EncoderCount++;
  }
}

//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR() {
  timer_flag = 1;  //定时时间达到标志
  M1Velocity = -M1EncoderCount;
  M2Velocity = M2EncoderCount;
  M1EncoderCount = 0;
  M2EncoderCount = 0;
}

void setup() {
  // IO引脚初始化
  pinMode(LeftMotorAIN1, OUTPUT);  //将电机控制引脚配置为输出模式
  pinMode(LeftMotorAIN2, OUTPUT);  //将电机控制引脚配置为输出模式

  pinMode(RightMotorBIN1, OUTPUT);  //将电机控制引脚配置为输出模式
  pinMode(RightMotorBIN2, OUTPUT);  //将电机控制引脚配置为输出模式

  pinMode(Motor1EncoderA, INPUT_PULLUP);
  pinMode(Motor1EncoderB, INPUT_PULLUP);
  pinMode(Motor2EncoderA, INPUT_PULLUP);
  pinMode(Motor2EncoderB, INPUT_PULLUP);

  attachInterrupt(Motor1EncoderA, readMotor1EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(Motor1EncoderB, readMotor1EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(Motor2EncoderA, readMotor2EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(Motor2EncoderB, readMotor2EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发

  /***************** 定时中断 *****************/
  timer1.attach_ms(interruptTime, timerISR);  // 打开定时器中断
  interrupts();                               //打开外部中断

  //串口初始化
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(2.08, 0.92, -0.08);
  delay(1000);

  lastTime = millis();
}

void loop() {
  mpu6050.update();
  angleY = mpu6050.getAngleY();  // get angle and gyro value
  angleZ = mpu6050.getAngleZ();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  if (timer_flag == 1)

  {  //判断是否发生定时器中断，这里是 interrupt_time ms发生一次
    timer_flag = 0;  //清除标记
  }
  if (millis() - lastTime < 5000) {
    RunMotors(100, 100);
  } else {
    RunMotors(255, 255);
  }

  Serial.print("M1速度： ");
  Serial.print(M1Velocity);
  Serial.print("M2速度： ");
  Serial.print(M2Velocity);
  Serial.print("俯仰角： ");
  Serial.print(angleY);
  Serial.print("偏航角： ");
  Serial.print(angleZ);
  Serial.print("俯仰角速度： ");
  Serial.print(gyroY);
  Serial.print("偏航角速度： ");
  Serial.println(gyroZ);
  delay(100);
}
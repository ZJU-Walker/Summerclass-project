//任务要求：从出发区出发，巡线一圈，停在返回区
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，三路循迹传感器
#ifndef ESP_ARDUINO_VERSION_MAJOR
#include "analogWrite.h" //预处理文件包含命令，用来引入对应的analogWrite库头文件
// C:\Users\Sun\AppData\Local\Arduino15\packages\esp32\hardware\esp32\2.0.3-RC1\cores\esp32
// esp32-hal-ledc.c 中已经定义了analogWrite函数，pwm频率1000hz,所以不再需要第三方库，第三方库频率5000hz
#endif

//******************电机驱动PWM引脚***************************//
#define LeftMotorAIN1 \
  32 //左轮电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2 \
  33 //左轮机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1 \
  26 //右轮电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2 \
  25 //  右轮电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚

//********************循迹传感器***************************//
#define LeftGrayscale 17   //左循迹传感器引脚，根据实际接线修改引脚
#define MiddleGrayscale 16 //中循迹传感器引脚，根据实际接线修改引脚
#define RightGrayscale 4   //左循迹传感器引脚，根据实际接线修改引脚
#define BLACK HIGH         //循迹传感器检测到黑色为“HIGH”,灯灭（黑），输出高电平1
#define WHITE LOW          //循迹传感器检测到白色为“LOW”，灯亮（白），输出低电平0

//********************启动按钮***************************//
#define ButtonPin 0

//******************电机启动初始值 **********************//
int motorDeadZone = 0;
//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM；motorb  b电机驱动PWM；取值范围[-255,255]
**************************************************************************/
void RunMotors(int motora, int motorb)
{
  if (motora > 0)
    analogWrite(LeftMotorAIN2, motora + motorDeadZone),
        analogWrite(LeftMotorAIN1,
                    0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(LeftMotorAIN2, 0), analogWrite(LeftMotorAIN1, 0);
  else if (motora < 0)
    analogWrite(LeftMotorAIN1, -motora + motorDeadZone),
        analogWrite(
            LeftMotorAIN2,
            0); //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(RightMotorBIN2, motorb + motorDeadZone),
        analogWrite(RightMotorBIN1,
                    0); //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(RightMotorBIN2, 0), analogWrite(RightMotorBIN1, 0);
  else if (motorb < 0)
    analogWrite(RightMotorBIN1, -motorb + motorDeadZone),
        analogWrite(
            RightMotorBIN2,
            0); //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}
// xVelocity 小车x方向速度 单位：cm/s  angularVelocity小车角速度 单位：度/s
void RunCar(int xVelocity, int angularVelocity)
{
  int M1Speed, M2Speed;
  M1Speed = (xVelocity * 1 - angularVelocity * 1);
  M2Speed = (xVelocity * 1 + angularVelocity * 1);
  RunMotors(M1Speed, M2Speed);
}

void stop() //普通刹车，停车，惯性滑行
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

unsigned long lastTime;
bool timeflag = 1;
//========================灰度传感器读取==================================
// byte 一个字节存储一个8位无符号数，从0到255。
byte sensorValue[4] = {0}; //巡线传感器值 sensorValue[3] 传感器组复合值 左传感器高位模式
byte lastSensorValue = 10;

//                  以中心点为坐标系原点,右为正值，左为负值
const float ioDistance[3] = {-1, 0, 1};
float offsetDistance = 0; //小车偏移量
void readSensors()
{
  sensorValue[0] = digitalRead(LeftGrayscale);                                     //左
  sensorValue[1] = digitalRead(MiddleGrayscale);                                   //中
  sensorValue[2] = digitalRead(RightGrayscale);                                    //右
  sensorValue[3] = (sensorValue[0] << 2) + (sensorValue[1] << 1) + sensorValue[2]; //传感器组复合值 左传感器高位模式 俯视
  if (sensorValue[3] == 0)
  {
    offsetDistance = -100.0;
  }
  else
  {
    offsetDistance = (sensorValue[0] * ioDistance[0] + sensorValue[1] * ioDistance[1] + sensorValue[2] * ioDistance[2]) / (sensorValue[0] + sensorValue[1] + sensorValue[2]);
  }
}

void setup()
{
  pinMode(LeftMotorAIN1, OUTPUT); //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);
  pinMode(LeftGrayscale, INPUT_PULLUP);   //定义左循迹传感器为上拉输入
  pinMode(MiddleGrayscale, INPUT_PULLUP); //定义中循迹传感器为上拉输入
  pinMode(RightGrayscale, INPUT_PULLUP);  //定义右循迹传感器为上拉输入

  pinMode(ButtonPin, INPUT);
  Serial.begin(115200);
  while (digitalRead(ButtonPin) == HIGH)
  {
    Serial.print("isRobotOn:");
    Serial.println(!digitalRead(ButtonPin));
  }
  // delay(2000);
  lastTime = millis(); // millis函数可获取单片机开机后运行的时间长度，单位ms
}

void loop()
{
  readSensors();
  //   Serial.print("All Sensors Value:");
  //   Serial.println( sensorValue[3],BIN);
  //   Serial.print("Offset Distance Value:");
  //   Serial.println( offsetDistance);
  if (millis() - lastTime < 20000)
  {
    //请断开小车总电源烧录，请拿小车底部
    switch (sensorValue[3])
    {
    // 111十字路口黑色，前进
    case 7:
      RunCar(120, 0);
      break;
    // 010正常 前进
    case 2:
      RunCar(120, 0); // moveForward
      break;
    // 110小偏右，左转纠正
    case 6:
       RunCar(120, 20);// turnLeft
      break;
    // 100大偏右，左转纠正
    case 4:
       RunCar(50, 50);// turnLeft
      break;
    // 011小偏左，小右转纠正
    case 3:
      RunCar(120, -20); // turnRight()
      break;
    // 001偏左，右转纠正
    case 1:
     RunCar(50, -50);; // turnRight()
      break;
    // 101
    case 5:
      // do something
      break;
    // 000
    case 0:
      //基于时间 基于特征
      // 500ms内盲走
      if (millis() - lastTime <= 500)
      {
         RunCar(120, 0);// moveForward
      }
      // 500ms-25000ms巡线
      else if ((millis() - lastTime) > 500 && (millis() - lastTime) < 25000)
      {
        //当上一次是111或011时，认为识别到直角弯，原地右转
        if (lastSensorValue == 7)
        {
          RunCar(0, -130);// spin
        }
        else
        {
         RunCar(-120,0); // moveBackward
        }
      }
      // 25000ms后，返回区停止
      else
      {
        //循环中，函数只执行一次的方法
        //方法1:标志位
        if (timeflag)
        {
          RunCar(120, 0); // moveForward
          delay(500);
          RunCar(0, 0);
          timeflag = 0;
        }
        else
        {
           RunCar(0, 0);
        }
      }
      break;

    default:
       RunCar(0, 0);
      break;
    }
  }
  else
  {
     RunCar(0, 0);
  }
  //保存本次的传感器值
  lastSensorValue = sensorValue[3];
}

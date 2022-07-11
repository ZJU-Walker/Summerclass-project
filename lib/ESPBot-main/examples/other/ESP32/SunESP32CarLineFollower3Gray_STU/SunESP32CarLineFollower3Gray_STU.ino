//任务要求：从出发区出发，巡线一圈，停在返回区
//******************电机驱动规格***************************//
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，三路循迹传感器
#include "analogWrite.h"  //预处理文件包含命令，用来引入对应的analogWrite库头文件

//******************电机驱动PWM引脚***************************//
#define LeftMotorAIN1 \
  32  //左轮电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2 \
  33  //左轮机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1 \
  26  //右轮电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2 \
  25  //  右轮电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚

//********************循迹传感器***************************//
#define LeftGrayscale 17  //左循迹传感器引脚，根据实际接线修改引脚
#define MiddleGrayscale 16  //中循迹传感器引脚，根据实际接线修改引脚
#define RightGrayscale 4  //左循迹传感器引脚，根据实际接线修改引脚
#define BLACK HIGH  //循迹传感器检测到黑色为“HIGH”,灯灭（黑），输出高电平1
#define WHITE LOW  //循迹传感器检测到白色为“LOW”，灯亮（白），输出低电平0

//********************启动按钮***************************//
#define ButtonPin 0

//******************电机启动初始值 **********************//
int motorDeadZone = 0;
//高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM；motorb  b电机驱动PWM；取值范围[-255,255]
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

void brake()  //急刹车，停车
{
  digitalWrite(RightMotorBIN1, HIGH);
  digitalWrite(RightMotorBIN2, HIGH);
  digitalWrite(LeftMotorAIN1, HIGH);
  digitalWrite(LeftMotorAIN2, HIGH);
}
void stop()  //普通刹车，停车，惯性滑行
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

unsigned long lastTime;
bool timeflag = 1;
//==========================================================

void setup() {
  pinMode(LeftMotorAIN1, OUTPUT);  //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);
  pinMode(LeftGrayscale, INPUT_PULLUP);  //定义左循迹传感器为上拉输入
  pinMode(MiddleGrayscale, INPUT_PULLUP);  //定义中循迹传感器为上拉输入
  pinMode(RightGrayscale, INPUT_PULLUP);  //定义右循迹传感器为上拉输入

  pinMode(ButtonPin, INPUT);
  Serial.begin(115200);
  while (digitalRead(ButtonPin) == HIGH) {
    Serial.print("isRobotOn:");
    Serial.println(!digitalRead(ButtonPin));
  }
  // Serial.begin(9600);
  // delay(2000);
  lastTime = millis();  // millis函数可获取单片机开机后运行的时间长度，单位ms
}

void loop() {
  bool LeftGrayscaleValue = digitalRead(LeftGrayscale);
  bool MiddleGrayscaleValue = digitalRead(MiddleGrayscale);
  bool RightGrayscaleValue = digitalRead(RightGrayscale);
  // Serial.print("LeftGrayscaleValue:");
  // Serial.print(LeftGrayscaleValue);
  // Serial.print(",");
  // Serial.print("MiddleGrayscale:");
  // Serial.println(MiddleGrayscaleValue);
  // Serial.print(",");
  // Serial.print("RightGrayscaleValue:");
  // Serial.println(RightGrayscaleValue);
  if (millis() - lastTime < 20000) {
    // 111都是黑色，前进
    if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == BLACK &&
        RightGrayscaleValue == BLACK) {
      RunMotors(170, 170);  // moveForward
    }

    // 010 前进
    else if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == BLACK &&
             RightGrayscaleValue == WHITE) {
      RunMotors(170, 170);  //  moveForward
    }
    // 110偏右，左转纠正
    else if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == BLACK &&
             RightGrayscaleValue == WHITE) {
      RunMotors(0, 150);  // turnLeft
    }
    // 100偏右，左转纠正
    else if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == WHITE &&
             RightGrayscaleValue == WHITE) {
      RunMotors(0, 150);  // turnLeft
    }
    // 011偏左，右转纠正
    else if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == BLACK &&
             RightGrayscaleValue == BLACK) {
      //请断开小车总电源烧录，请拿小车底部
      RunMotors(150, 0);  // turnRight()
    }

    // 001偏左，右转纠正
    else if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == WHITE &&
             RightGrayscaleValue == BLACK) {
      //请断开小车总电源烧录，请拿小车底部
      RunMotors(150, 0);  // turnRight()
    }

    // 101，后退
    else if (LeftGrayscaleValue == BLACK && MiddleGrayscaleValue == WHITE &&
             RightGrayscaleValue == BLACK) {
      RunMotors(-150, -150);  // moveBackward
    }

    // 000，后退
    else if (LeftGrayscaleValue == WHITE && MiddleGrayscaleValue == WHITE &&
             RightGrayscaleValue == WHITE) {
      //参考方案1
      // 500ms内盲走
      if (millis() - lastTime <= 500) {
        RunMotors(150, 150);  // moveForward
      }
      // 500ms-25000ms巡线
      else if ((millis() - lastTime) > 500 && (millis() - lastTime) < 25000) {
        RunMotors(-150, -150);  // moveBackward
      }
      // 25000ms后，返回区停止
      else {
        //循环中，函数只执行一次的方法
        //方法1:标志位
        if (timeflag) {
          RunMotors(150, 150);  // moveForward
          delay(500);
          RunMotors(0, 0);
          timeflag = 0;
        } else {
          RunMotors(0, 0);
        }
        //方法2:退出程序
        // do something
        // exit(0);
      }
    }

  }

  else
    RunMotors(0, 0);
}

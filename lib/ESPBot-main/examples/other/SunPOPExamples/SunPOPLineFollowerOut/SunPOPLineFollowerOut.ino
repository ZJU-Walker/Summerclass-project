//******************电机驱动规格***************************//
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，两路循迹传感器均在黑线外
#define LeftMotorAIN1 6  // A电机控制PWM波,左电机控制引脚1
#define LeftMotorAIN2 5  // A电机控制PWM波,左电机控制引脚2

#define RightMotorBIN1 8  // B电机控制PWM波,右电机控制引脚1
#define RightMotorBIN2 7  // B电机控制PWM波,右电机控制引脚2
//********************循迹传感器***************************//
#define LeftGrayscale 42   //左循迹传感器引脚
#define RightGrayscale 48  //左循迹传感器引脚
#define BLACK HIGH         //循迹传感器检测到黑色为“HIGH”,灯灭
#define WHITE LOW          //循迹传感器检测到白色为“LOW”，灯亮

//=======================智能小车的基本动作=========================
void moveForward()  // 前进
{
  // 左电机前进，PWM比例0~255调速，左右轮差异略增减
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机前进，PWM比例0~255调速，左右轮差异略增减
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void moveBackward()  //后退
{
  //左轮后退，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);  
  //右轮后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);  
}

void turnLeft()  //左转(左轮不动，右轮前进)
{
  //右电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);  
  analogWrite(LeftMotorAIN2, 0);
   //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100); 
  analogWrite(RightMotorBIN2, 0);
}

void spinLeft()  //左转(左轮后退，右轮前进)
{
  //左轮后退PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);   
  //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100);  
  analogWrite(RightMotorBIN2, 0);
}

void turnRight()  //右转(右轮不动，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);  
//右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 0);  
}

void spinRight()  //右转(右轮后退，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);  
  analogWrite(LeftMotorAIN2, 0);
//右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);  
}

void brake()  //急刹车，停车
{
  digitalWrite(RightMotorBIN1, HIGH);
  digitalWrite(RightMotorBIN2, HIGH);
  digitalWrite(LeftMotorAIN1, HIGH);
  digitalWrite(LeftMotorAIN2, HIGH);
}
void stop()  //普通刹车，停车
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

//==========================================================

void setup() {
  pinMode(LeftMotorAIN1, OUTPUT);  //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);
  pinMode(LeftGrayscale, INPUT);   //定义左循迹传感器为输入
  pinMode(RightGrayscale, INPUT);  //定义右循迹传感器为输入
  //Serial.begin(9600);
  //moveForward();
  //delay(2000);
}

void loop() {
  bool LeftGrayscaleValue = digitalRead(LeftGrayscale);
  bool RightGrayscaleValue = digitalRead(RightGrayscale);
  //Serial.print("LeftGrayscaleValue:");
  //Serial.print(LeftGrayscaleValue);
  //Serial.print(",");
  //Serial.print("RightGrayscaleValue:");
  //Serial.println(RightGrayscaleValue);
  if (LeftGrayscaleValue == BLACK &&
      RightGrayscaleValue == BLACK)  //都是黑色，前进
  {
    moveForward();
  }
  if (LeftGrayscaleValue == WHITE &&
      RightGrayscaleValue == WHITE)  //都是白色，前进
  {
     moveForward();
  }
  if (LeftGrayscaleValue == BLACK &&
      RightGrayscaleValue == WHITE)  //偏右，左转纠正
  {
    turnLeft();
  }
  if (LeftGrayscaleValue == WHITE &&
      RightGrayscaleValue == BLACK)  //偏左，右转纠正
  {
    turnRight();
  }
}

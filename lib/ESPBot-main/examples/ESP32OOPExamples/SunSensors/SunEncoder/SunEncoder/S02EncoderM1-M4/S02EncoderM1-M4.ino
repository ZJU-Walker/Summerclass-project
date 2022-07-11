#include <Ticker.h>  //定时中断

#include "OOPConfig.h"  //包含配置库

const int ButtonPin = 0;
/***************** 定时中断参数 *****************/
Ticker timer1;                 // 中断函数
const int interruptTime = 10;  // 中断时间
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

bool flipEncoder[WHEELS_NUM] = {
    true, false, true,
    false};  //编码器计数方向翻转变量数组，false不修正,true需要修正
long feedbackPulses[WHEELS_NUM] = {0, 0, 0,
                                   0};  //四个车轮的定时中断编码器四倍频计数
//*****************创建1个4路电机对象***************************//
BDCMotor motors;

int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR() {
  //获取电机脉冲数（速度）
  timer_flag = 1;  //定时时间达到标志
  print_Count++;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  //复位
    motors.setSpeeds(100, 100, 100, 100);
  }
}

void setup() {
  motors.init();
  motors.flipMotors(false, false, false,
                    false);  //根据实际转向进行调整false or true 黑色PCB电机
                             // false  绿色PCB电机true
  for (int i = 0; i < WHEELS_NUM; i++) {
    ENC[i].init();
    ENC[i].flipEncoder(flipEncoder[i]);
  }
  delay(100);
  pinMode(ButtonPin, INPUT_PULLUP);
  Serial.begin(BAUDRATE);

  Serial.println("Sunnybot ENCOUDER Test，请悬空车轮后按下BOOT按键开始测试");
  while (digitalRead(ButtonPin) == HIGH) {
    Serial.print("请按BOOT按键:");
    Serial.println(!digitalRead(ButtonPin));
  }
  /***************** 定时中断 *****************/
  timer1.attach_ms(interruptTime, timerISR);  // 打开定时器中断
  interrupts();                               //打开外部中断
}

void loop() {
  if (print_Count >= 100)  //打印控制，控制周期1000ms
  {
    Serial.println(millis());  //显示时间
    for (int i = 0; i < WHEELS_NUM; i++) {
      Serial.print((String) "M" + (i + 1) + ":");  //显示
      Serial.print("反馈数");
      Serial.println(feedbackPulses[i]);  //显示     
    }
     print_Count = 0;
  }
}

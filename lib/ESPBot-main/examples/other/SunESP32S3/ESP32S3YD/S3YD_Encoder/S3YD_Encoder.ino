//#include "analogWrite.h"  //模拟量读写库
#include <Ticker.h>

#include "ESP32Car.h"  //ESP32小车库文件

/***************** 定时中断参数 *****************/
#define WHEELS_NUM 4
Ticker timer1;                          // 中断函数
const unsigned int interruptTime = 10;  // 中断时间
unsigned int printCount = 0;            // 中断时间
int timer_flag = 0;                     //定时器标志；
#define COUNTS_PER_REV 1560  // wheel encoder's num of ticks per rev 390*4=1560

float Velocity[WHEELS_NUM];  //电机速度
int SpeedPWM[WHEELS_NUM];
bool fixEncoder[WHEELS_NUM] = {1, 0, 1,
                               0};  //编码器计数方向修正，0不需要修正,1需要修正
unsigned long lastTime;
//定时器中断处理函数,得到编码器采样周期内的计数并清零
void timerIsr() {
  timer_flag = 1;  //定时时间达到标志
  for (int i = 0; i < WHEELS_NUM; i++) {
    if (fixEncoder[i] == 0) {
      Velocity[i] = EncoderCount[i];
    } else {
      Velocity[i] = -EncoderCount[i];
    }

    EncoderCount[i] = 0;
  }
}

void setup() {
  // IO引脚初始化
  pinMode(M1PWM1, OUTPUT);
  pinMode(M2PWM1, OUTPUT);
  pinMode(M3PWM1, OUTPUT);
  pinMode(M4PWM1, OUTPUT);
  pinMode(M1PWM2, OUTPUT);
  pinMode(M2PWM2, OUTPUT);
  pinMode(M3PWM2, OUTPUT);
  pinMode(M4PWM2, OUTPUT);

  pinMode(M1ENA, INPUT_PULLUP);
  pinMode(M1ENB, INPUT_PULLUP);
  pinMode(M2ENA, INPUT_PULLUP);
  pinMode(M2ENB, INPUT_PULLUP);
  pinMode(M3ENA, INPUT_PULLUP);
  pinMode(M3ENB, INPUT_PULLUP);
  pinMode(M4ENA, INPUT_PULLUP);
  pinMode(M4ENB, INPUT_PULLUP);

  attachInterrupt(M1ENA, readM1EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M1ENB, readM1EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M2ENA, readM2EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M2ENB, readM2EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M3ENA, readM3EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M3ENB, readM3EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M4ENA, readM4EncoderA,
                  CHANGE);  //中断处理函数，电平变化触发
  attachInterrupt(M4ENB, readM4EncoderB,
                  CHANGE);  //中断处理函数，电平变化触发

  /***************** 定时中断 *****************/
  timer1.attach_ms(interruptTime, timerIsr);  // 打开定时器中断
  interrupts();                               //打开外部中断

  //串口初始化
  Serial.begin(115200);
  lastTime = millis();
}

void loop() {
  if (millis() - lastTime < 8000) {
    setSpeeds(250, 250, 250, 250);
    delay(2000);
    brake();
     Serial.println("停了吗");
    delay(2000);
    setSpeeds(250, 250, 250, 250);
    delay(2000);
    setSpeeds(0, 0, 0, 0);
    delay(2000);

  } else {
    if (timer_flag == 1)  //判断是否发生定时器中断，这里是 10ms发生一次
    {
      timer_flag = 0;  //清除标记
      printCount++;
      //控制加速和打印频率
      if (printCount == 20) {
        for (int i = 0; i < WHEELS_NUM; i++) {
          if (abs(SpeedPWM[i]) < 256) {
            SpeedPWM[i]++;
          } else if (abs(SpeedPWM[i]) >= 256) {
            SpeedPWM[i] = -200;
          }

          Serial.println((String) "电机M" + (i + 1) + "脉冲计数为：" +
                         Velocity[i] + "PWM:" + SpeedPWM[i]);
          printCount = 0;
        }
      }
    }

    setSpeeds(SpeedPWM[0], SpeedPWM[1], SpeedPWM[2], SpeedPWM[3]);
  }
}
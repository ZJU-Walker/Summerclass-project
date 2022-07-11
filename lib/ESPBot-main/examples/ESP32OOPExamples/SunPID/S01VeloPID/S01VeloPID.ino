#include <Ticker.h>  //定时中断

#include "OOPConfig.h"  //包含配置库
#ifndef PWM_MAX
#define PWM_MAX 255
#define PWM_MIN -PWM_MAX
#endif

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

long targetPulses[WHEELS_NUM] = {50, 50, 50, 50};  //四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0,
                                   0};  //四个车轮的定时中断编码器四倍频计数
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

int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据
void timerISR() {
  //获取电机脉冲数（速度）
  timer_flag = 1;  //定时时间达到标志
  print_Count++;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  //复位
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
}

void setup() {
  motors.init();
  motors.map2model(1, 2, 3,
                   4);  //映射实际接线电机编号到理论建模编号，默认顺序1，2，3，4
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

  Serial.println("Sunnybot PID Test，请悬空车轮后按下BOOT按键开始测试");
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
    Serial.println(millis());  //显示
    for (int i = 0; i < WHEELS_NUM; i++) {
      Serial.print((String) "M" + (i + 1) + ":");  //显示
      Serial.print("目标数：");                    //显示
      Serial.print(targetPulses[i]);               //显示
      Serial.print("反馈数");
      Serial.print(feedbackPulses[i]);  //显示
      Serial.print("输出值");
      Serial.println(outPWM[i]);  //显示
    }
    print_Count = 0;
  }
}
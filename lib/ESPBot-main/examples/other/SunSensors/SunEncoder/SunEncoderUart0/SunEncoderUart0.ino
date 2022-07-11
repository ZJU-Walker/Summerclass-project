
/* Encoder Library - 阶跃输入 串口0输出 Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * 使用SunEncoder库检测4路电机速度.
 */

#include <FlexiTimer2.h>  //定时中断
// 请先安装FlexiTimer2库,https://playground.arduino.cc/Main/FlexiTimer2/


#include "SunConfig.h"       //包含配置库

A4950MotorShield motors;
Battery Battery3s;
long oldPulses[4] = {-999, -999, -999, -999};
long newPulses[4] = {0, 0, 0, 0};
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
//   avoid using pins with LEDs attached

unsigned long previousMillis = 0;  // will store last time run
const long period = 2000;          // period at which to run in ms
bool runflag = 1;
void control() {
  static int print_Count;
  sei();  //全局中断开启

  //获取电机速度
  newPulses[0] = ENC[0].read();   // A
  newPulses[1] = -ENC[1].read();  // B
  newPulses[2] = ENC[2].read();   // C
  newPulses[3] = -ENC[3].read();  // D

  for (int i = 0; i < WHEEL_NUM; i++) {
    oldPulses[i] = newPulses[i];
    ENC[i].write(0);  //复位
  }

    if (++print_Count < 200)  //打印控制
    {
      Serial.print(millis());//显示
      Serial.print(",");
      Serial.println(newPulses[0]);  //显示
      // Serial.print(",");
      // Serial.print(newPulses[1]);  //显示
      // Serial.print(",");
      // Serial.print(newPulses[2]);  //显示
      // Serial.print(",");
      // Serial.println(newPulses[3]);  //显示
      //                                 // Serial.println(iConstrain);
      //                                 //    print_Count = 0;
    }
   }

  void setup() {
    motors.init();

    delay(100);                     //延时等待初始化完成
    FlexiTimer2::set(10, control);  // 10毫秒定时中断函数
    FlexiTimer2::start();           //中断使能
    delay(100);                     //延时等待初始化完成
    Serial.begin(115200);
    Serial.println("Sunnybot Basic Encoder Test:");
    Serial.println(Battery3s.read());
    previousMillis = millis();
  }

  void loop() {
    unsigned long currentMillis = millis();  // store the current time
    if (currentMillis - previousMillis < period) {
      motors.setSpeeds(255, 255, 255, 255);
      // Serial.print(newPulses[0]);  //显示
      // Serial.print(",");
      // Serial.print(newPulses[1]);  //显示
      // Serial.print(",");
      // Serial.print(newPulses[2]);  //显示
      // Serial.print(",");
      // Serial.println(newPulses[3]);  //显示
    } else {
      if (runflag == 1) {
        motors.motorsBrake();
        Serial.println("end");
        runflag = 0;
      } else {
        motors.motorsBrake();
      }
    }
  }

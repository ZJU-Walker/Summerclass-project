
/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * 使用SunEncoder库检测4路电机定时中断周期的脉冲数（速度）.
 */

#include <FlexiTimer2.h>  //定时中断
// 请先安装FlexiTimer2库,https://playground.arduino.cc/Main/FlexiTimer2/
#include "SunConfig.h"  //包含配置库

A4950MotorShield motors;
Battery Battery3s;
long oldPulses[4] = {0, 0, 0, 0};
long newPulses[4] = {0, 0, 0, 0};
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder ENC[4] = {
    Encoder(ENCODER_A, DIRECTION_A), Encoder(ENCODER_B, DIRECTION_B),
    Encoder(ENCODER_C, DIRECTION_C), Encoder(ENCODER_D, DIRECTION_D)};
//   avoid using pins with LEDs attached
void control() {
  static int print_Count;
  sei();  //全局中断开启

//获取电机脉冲数（速度）
#ifdef PINS_REVERSE
  newPulses[0] = -ENC[0].read();  // A
  newPulses[1] = ENC[1].read();   // B
  newPulses[2] = -ENC[2].read();  // C
  newPulses[3] = ENC[3].read();   // D
#else
  newPulses[0] = ENC[0].read();   // A
  newPulses[1] = -ENC[1].read();  // B
  newPulses[2] = ENC[2].read();   // C
  newPulses[3] = -ENC[3].read();  // D
#endif

  for (int i = 0; i < WHEEL_NUM; i++) {
    oldPulses[i] = newPulses[i];
    ENC[i].write(0);  //复位
  }

  //  if (++print_Count >= 10) //打印控制，控制周期100ms
  //  {
  // // Serial.println(millis());//显示
  //  Serial.print(Velocity_A);//显示
  //  Serial.print(",");
  // Serial.print(Velocity_B);//显示
  // Serial.print(",");
  //  Serial.print(Velocity_C);//显示
  //  Serial.print(",");
  //  Serial.println(Velocity_D);//显示
  //  //Serial.println(iConstrain);
  //    print_Count = 0;
  //  }
}

void setup() {
  motors.init();

  delay(100);                     //延时等待初始化完成
  FlexiTimer2::set(10, control);  // 10毫秒定时中断函数
  FlexiTimer2::start();           //中断使能
  delay(100);                     //延时等待初始化完成
  Serial.begin(BAUDRATE);
  Serial.println("Sunnybot Basic Encoder Test:");
}

void loop() {
  if (Battery3s.is_Volt_Low() == false) {
    for (int i = 0; i < 256; i++) {
      motors.setSpeeds(100, 100, 255, i);

      Serial.println(i);
      Serial.println(Battery3s.read());

      for (int j = 0; j < WHEEL_NUM; j++) {
        Serial.print("W");
        Serial.print(newPulses[j]);
      }
      Serial.println("end");
      delay(1000);
    }
  } else {
    Serial.println("Battery voltage is low!");
  }
}

/**
 * @file SR04NewPingExample.ino 基于NewPing库的超声波测距例程
 * @author igcxl (igcxl@qq.com)
 * @brief 基于NewPing库的超声波测距例程
 * @version 0.2
 * @date 2021-06-11
 * @copyright Copyright © Sunnybot.cn 2021 All right reserved.
 */
#include <NewPing.h>    //请先安装NewPing库
#define TRIGGER_PIN 29  // 宏定义29号数字端口为触发
#define ECHO_PIN 28     // 宏定义28号数字端口为接收模块反馈信号
#define MAX_DISTANCE \
  200  //宏定义模块的最大测量距离，规格标400cm，但实测一般为200cm
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  //声明模块的参数
void setup() {
  Serial.begin(115200);  // 设置串口的波特率，可以改为9600
}
void loop() {
  delay(
      50);  // 每次测量的时间间隔，模块规格为40Hz，因此最小为25ms，但一般不少于30ms
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());  // 调用库里面的 ping_cm() 方法，直接输出距离
  Serial.println("cm");
}

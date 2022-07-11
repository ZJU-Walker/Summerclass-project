/**
 * @file VoltageRead.ino 电压检测
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 使用基于电阻分压原理的模块进行电压检测，可完成对锂电池电压大小的检测，在低电压时关闭电机。
 * @note
 * 电压检测模块能使输入的电压缩小11倍。由于Arduino模拟输入电压最大为5V，故电压检测模块的输入电压不能大于5Vx11=55V。
 * Arduino的模拟量分辨率最小为5/1024=0.0049V，所以电压检测模块分辨率为0.0049Vx11=0.05371V。
 * 电压检测输入引脚A0
 * @version 0.5
 * @date 20201-09-11
 * @copyright Copyright © sunnybot.cn 2021
 *
 */

void setup() {
   // 初始化串行通信速率为9600bit/s：
   Serial.begin(9600);
}

void loop() {
   // 读取模拟引脚A8的输入数据
   int sensorValue = analogRead(A8);
   // 将模拟信号转换成电压
   float voltage = sensorValue * (5.0 / 1024.0)*11;
   // 打印到串口监视器
   Serial.println(voltage);
}
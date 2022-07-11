/**
 * @file BatteryVoltageDetector.ino 电压检测
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 使用基于电阻分压原理的模块进行电压检测，可完成对锂电池电压大小的检测，在低电压时关闭电机。
 * @note
 * 电压检测模块能使输入的电压缩小11倍。由于Arduino模拟输入电压最大为5V，故电压检测模块的输入电压不能大于5Vx11=55V。
 * Arduino的模拟量分辨率最小为5/1024=0.0049V，所以电压检测模块分辨率为0.0049Vx11=0.05371V。
 * 电压检测输入引脚A0
 * @version 0.5
 * @date 2020-12-11
 * @copyright Copyright © igcxl.com 2020
 *
 */


#include "SunSensors/SunBattery/SunBattery.h"         //电池电压检测库

Battery battery3S;

/**
 * @brief 初始化函数
 * Sun
 */
void setup() {
#ifdef DEBUG_INFO
  Serial.begin(BAUDRATE);  ///初始化串口
#endif
  delay(300);  ///<延时等待初始化完成
}

/**
 * @brief 主函数
 *
 */
void loop() {
#ifdef DEBUG_INFO
  if (battery3S.is_Volt_Low() == true) {
    Serial.println(millis());
  }

  Serial.println(float(battery3S.read()) * 0.01);  ///<打印电压

#endif
  delay(1000);
}
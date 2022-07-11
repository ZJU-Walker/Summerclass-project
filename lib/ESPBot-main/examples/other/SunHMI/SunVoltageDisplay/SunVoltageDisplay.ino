/**
 * @file SunVoltageDisplay.ino 电压显示
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 使用基于电阻分压原理的模块进行电压检测，可完成对锂电池电压大小的检测，在低电压时关闭电机。
 * @note
 * 电压检测模块能使输入的电压缩小11倍。由于Arduino模拟输入电压最大为5V，故电压检测模块的输入电压不能大于5Vx11=55V。
 * Arduino的模拟量分辨率最小为5/1024=0.0049V，所以电压检测模块分辨率为0.0049Vx11=0.05371V。
 * 电压检测输入引脚A0
 * @version 0.5
 * @date 2020-10-30
 * @copyright Copyright © igcxl.com 2020
 *
 */

#include <U8x8lib.h>
// 请先在库管理器中安装U8G2库，U8G2库源代码地址：https://github.com/olikraus/u8g2/

#include "SunConfig.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

Battery battery3S;
U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(/* clock=*/28, /* data=*/26,
                                          /* cs=*/0, /* dc=*/22, /* reset=*/24);
/**
 * @brief 初始化函数
 * Sun
 */
void setup() {
  u8x8.begin();
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
  u8x8.setFont(u8x8_font_inb46_4x8_n);
  u8x8.print(battery3S.read());
#ifdef DEBUG_INFO
  if (battery3S.is_Volt_Low() == true) {
    Serial.println(millis());
  }

  Serial.println(float(battery3S.read()) * 0.01);  ///<打印电压

#endif
  delay(1000);
}
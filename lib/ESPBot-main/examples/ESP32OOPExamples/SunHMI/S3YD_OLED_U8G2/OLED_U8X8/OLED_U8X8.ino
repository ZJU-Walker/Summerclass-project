// https://blog.51cto.com/dpjcn1990/2978430#32219_u8g2print___726
#include <Arduino.h>
// U8G2库源代码地址：https://github.com/olikraus/u8g2/
#include <U8x8lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
void setup(void)
{
  u8x8.begin();
  u8x8.setPowerSave(0);
}

void loop(void)
{
  u8x8.clearDisplay();
  u8x8.setFont(u8x8_font_chroma48medium8_r); // choose a suitable font
  u8x8.drawString(0, 0, "Hello World!");
  u8x8.setCursor(0, 1);
  u8x8.print("angle"); //汉字要用print

  delay(1000);
}

/*
U8x8 字符模式
优缺点
快速
不需要内存（RAM）
不能绘制图形
不支持所有显示控制器
https://blog.csdn.net/qq_17351161/article/details/105177112
*/
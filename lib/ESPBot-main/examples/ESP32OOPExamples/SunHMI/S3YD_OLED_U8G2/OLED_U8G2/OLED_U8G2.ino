// https://blog.51cto.com/dpjcn1990/2978430#32219_u8g2print___726
#include <Arduino.h>
// U8G2库源代码地址：https://github.com/olikraus/u8g2/
#include <U8g2lib.h> //点击自动打开管理库页面并安装: http://librarymanager/All#U8g2

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/SCL, /* data=*/SDA); // ESP32 Thing, HW I2C with pin remapping

void setup(void)
{
  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function 支持中文
}

void loop(void)
{
  u8g2.clearBuffer();                         // clear the internal memory
  u8g2.setFont(u8g2_font_unifont_t_chinese2); // choose a suitable font
  u8g2.setCursor(0, 15);
  u8g2.print("Hello World!");
  u8g2.setCursor(0, 30);
  u8g2.print("想输入的字"); //汉字要用print
  u8g2.sendBuffer();        // transfer internal memory to the display
  delay(1000);
}

/*
https://zhuanlan.zhihu.com/p/517423020
有些语言依旧无法在Arduino中容纳所有的字体，所以它有不同的尺寸的子集来满足不同的要求，比如，中文字体有3个子集。

• u8g2_font_unifont_t_chinese1 - 大小为14,178 字节
• u8g2_font_unifont_t_chinese2 - 大小为20,225 字节
• u8g2_font_unifont_t_chinese3 - 大小为37,502 字节
你可以参考U8g2 Github Wiki以了解更多细节：

https://github.com/olikraus/u8g2/wiki/fntgrpunifont
适合 u8g2 的中文字体，采用文泉驿点阵宋体作为源本，提供 12x12、13x13、14x14、15x15 和 16x16 点阵字库。
https://github.com/larryli/u8g2_wqy
*/
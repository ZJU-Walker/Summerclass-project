#include <FastLED.h>  // 此示例程序需要使用FastLED库

// How many leds in your strip?
#define NUM_LEDS 1  // LED灯珠数量

// For led chips like WS2812, which have a data line, ground, and power, you
// just need to define DATA_PIN.  For led chipsets that are SPI based (four
// wires - data, clock, ground, and power), like the LPD8806 define both
// DATA_PIN and CLOCK_PIN Clock pin only needed for SPI based chipsets when not
// using hardware SPI
#define DATA_PIN 33  //输出控制信号引脚
#define CLOCK_PIN 100  //不需要

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
  // Uncomment/edit one of the following lines for your leds arrangement.
  // ## Clockless types ##

  FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds,
                                         NUM_LEDS);  // GRB ordering is typical
}

void loop() {
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;  // 将光带上第1个LED灯珠设置为红色
  FastLED.show();       // 点亮/更新LED
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;  // 将光带上第1个LED灯珠设置为黑色
  FastLED.show();         // 点亮/更新LED
  delay(500);
  // Turn the LED on, then pause
  leds[0] = CRGB::Green;  // 将光带上第1个LED灯珠设置为绿色
  FastLED.show();         // 点亮/更新LED
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;  // 将光带上第1个LED灯珠设置为黑色
  FastLED.show();         // 点亮/更新LED
  delay(500);
  // Turn the LED on, then pause
  leds[0] = CRGB::Blue;  // 将光带上第1个LED灯珠设置为蓝色
  FastLED.show();        // 点亮/更新LED
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;  // 将光带上第1个LED灯珠设置为黑色
  FastLED.show();         // 点亮/更新LED
  delay(500);
  // Turn the LED on, then pause
  leds[0] = CRGB::White;  // 将光带上第1个LED灯珠设置为白色
  FastLED.show();         // 点亮/更新LED
  delay(500);
}
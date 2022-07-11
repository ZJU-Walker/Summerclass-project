#include "FastLED.h"  //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
#define NUM_LEDS 1   // LED灯珠数量
#define LED_PIN 48  // Arduino输出控制信号引脚
#define LED_TYPE WS2812B  //LED灯带型号ESP32-S3-DevKitC-1使用SK6822LED芯片 YD:WS2812B
#define COLOR_ORDER GRB  // RGB灯珠中红色、绿色、蓝色LED的排列顺序
uint8_t MaxBright = 50;
// LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS];
// 建立光带leds
void setup() {
  Serial.begin(9600);
  // 启动串行通讯
  delay(1000);
  // 稳定性等待
  LEDS.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);//初始化LED灯
  FastLED.setBrightness(MaxBright);  //设置光带亮度
}

void loop() {
  // Turn the LED on, then pause
  // www.taichi-maker.com/homepage/reference-index/arduino-library-index/fastled-library/rgb-color/#rgb
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
  leds[0] = CRGB::SkyBlue;
  FastLED.show();
  delay(500);
  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(500);
}
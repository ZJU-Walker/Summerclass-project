#include "FastLED.h" //点击这里会自动打开管理库页面: http://librarymanager/All#FastLED
// How many leds in your strip?
#define NUM_LEDS 1

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
// Clock pin only needed for SPI based chipsets when not using hardware SPI
#define DATA_PIN 48
#define CLOCK_PIN 48

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
    // Uncomment/edit one of the following lines for your leds arrangement.
    // ## Clockless types ##

    //ESP32-S3-DevKitC-1使用LED芯片
    FastLED.addLeds<SK6822, DATA_PIN, GRB>(leds, NUM_LEDS);


}

void loop() { 
  // Turn the LED on, then pause
  leds[0] = CRGB::Red;
  delay(500);
  // Now turn the LED off, then pause
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
    leds[0] = CRGB::SkyBlue;
  FastLED.show();
  delay(500);
  leds[0] = CRGB::White;
  FastLED.show();
  delay(500); 
  leds[0] = CRGB::Purple;
  FastLED.show();
  delay(500);
}

/**
 * @file SunButtonBounce.ino 按键抖动引脚更改中断测试
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 通过按键控制串口0输出
 * @note
 * 按键输入引脚-需要是支持引脚更改中断引脚
 * 1.测试按键的抖动，使用EnableInterrupt库，引脚更改中断
 * See https://github.com/GreyGnome/EnableInterrupt and the README.md for more information.
 * @version 0.5
 * @date 2021-03-18
 * @copyright Copyright © Sunnybot.cn 2021
 *
 */

#include <EnableInterrupt.h>

// Modify this at your leisure. Refer to https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
#if defined __AVR_ATmega640__ || defined __AVR_ATmega2560__ || defined __AVR_ATmega1280__ || \
	      defined __AVR_ATmega1281__ || defined __AVR_ATmega2561__
#define ARDUINOPIN 10
#else
// Pin 7 is useful on Arduino Uno, Leonardo, Mighty1284, ATtiny84...
#define ARDUINOPIN 7
#endif

volatile uint16_t interruptCount=0; // The count will go back to 0 after hitting 65535.

void interruptFunction() {
  interruptCount++;
}

void setup() {
  Serial.begin(115200);
#ifdef MIGHTY1284
  DDRA=0x0;    DDRB=0x0;   DDRC=0x0;   DDRD=0x0; // set all pins as inputs
  PORTA=0xFF; PORTB=0xFF; PORTC=0xFF; PORTD=0xFF; // turn on all pullup resistors.
#else
  pinMode(ARDUINOPIN, INPUT_PULLUP);  // See http://arduino.cc/en/Tutorial/DigitalPins
#endif
  enableInterrupt(ARDUINOPIN, interruptFunction, CHANGE);
}

// In the loop we just display interruptCount. The value is updated by the interrupt routine.
void loop() {
  Serial.println("---------------------------------------");
  delay(1000);
  Serial.print("Pin was interrupted: ");
  Serial.print(interruptCount, DEC);
  Serial.println(" times so far.");
}

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// GORY DETAILS //////////////////////////////////////
// This has only been tested on an Arduino Duemilanove and Mega ADK.
// It is designed to work with the Arduino Duemilanove/Uno, Arduino Mega2560/ADK, the Arduino
// Leonardo, and the Arduino Due. Please let me know how you fare on the Leonardo or Due.

// To use:

// 1. You must be using a fairly recent version of the Arduino IDE software on your PC/Mac,
// that is, version 1.0.1 or later. Check Help->About Arduino in the IDE.

// 2. Wire a simple switch to any Analog or Digital pin (known as ARDUINOPIN, defined below)
// that supports interrupts. See https://github.com/GreyGnome/EnableInterrupt/wiki/Usage#Summary
// Attach the other end to a GND pin. A "single pole single throw momentary contact normally
// open" // pushbutton switch is best for the most interrupting fun.
// See https://www.sparkfun.com/products/97 and https://octopart.com/b3f-1000-omron-3117

// 3. When pressed, the switch will connect the pin to ground ("low", or "0") voltage, and interrupt the
// processor. See http://arduino.cc/en/Tutorial/DigitalPins

// 4. The interrupt is serviced immediately, and the ISR (Interrupt SubRoutine) modifies the value of
// the global variable interruptCount. Open Tools->Serial Monitor in the IDE to see the results of your
// interrupts.

// 5. Peruse the Examples directory for more elaborate examples.

// 6. Create your own sketch using the EnableInterrupt library!

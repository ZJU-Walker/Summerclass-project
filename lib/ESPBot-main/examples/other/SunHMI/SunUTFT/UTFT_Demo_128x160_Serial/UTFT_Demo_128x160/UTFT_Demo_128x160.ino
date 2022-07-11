// UTFT_Demo_128x128_Serial (C)2012 Henning Karlsen
//屏幕购买连接：https://shop73023976.taobao.com/?spm=2013.1.1000126.d21.f3HieB
// 晶美达：https://item.taobao.com/item.htm?spm=a1z09.2.0.0.105a2e8dMPVrft&id=620277170307&_u=jvnjr0ca27
// UTFT库：www.rinkydinkelectronics.com/library.php?id=51
// This program is a demo of how to use most of the functions
// of the library with a supported display modules.
//
// This demo was made to work on the 128x160 modules.
// Any other size displays may cause strange behaviour.
//
// This program requires the UTFT library.
//

#include <UTFT.h>
#ifndef USE_TFT_LCD
//#define LS18TFT  //绿深1.8 TFT 3.3V 8针
//#define JMD18TFT       //晶美达1.8 TFT 7针 3.3V  ST7735S
//#define YUROBOT18TFT       //YUROBOT深1.8 TFT 5V 8针
#define YUROBOT18TFT_QIYI
#ifdef LS18TFT  // 绿深1.8 TFT
#define TFT_Modle ST7735
#define TFT_SDA 24
#define TFT_SCL 28
#define TFT_CS 22
#define TFT_RST 26
#define TFT_RS 25
#endif
#ifdef JMD18TFT  //晶美达1.8 TFT 7针 3.3V
#define TFT_Modle ST7735
#define TFT_SDA 26
#define TFT_SCL 28
#define TFT_CS 25
#define TFT_RST 24
#define TFT_RS 22
#endif

#ifdef YUROBOT18TFT  // YUROBOT深1.8 TFT 5V
#define TFT_Modle ST7735
#define TFT_SDA 26
#define TFT_SCL 28  //对应屏幕的SCK引脚
#define TFT_CS 24
#define TFT_RST 25
#define TFT_RS 22
#endif

#ifdef YUROBOT18TFT_QIYI  // YUROBOT深1.8 TFT 5V
#define TFT_Modle ST7735
#define TFT_SCL 27 //sck
#define TFT_SDA 26
#define TFT_CS 25
#define TFT_RS 24 //DC
#define TFT_RST 23
#endif

#endif
// Declare which fonts we will be using
extern uint8_t BigFont[];

// Declare an instance of the class
// UTFT myGLCD(Modle,SDA,SCL,CS,RST,RS);   // Remember to change the model
// parameter to suit your display module! UTFT myGLCD(ST7735,24,28,22,26,25); //
// Remember to change the model parameter to suit your display module! UTFT
// myGLCD(ST7735,24,28,22,26,25);   // 绿深1.8 TFT UTFT
// myGLCD(ST7735,26,28,25,24,22);   // 晶美达1.8 TFT 7针 UTFT myGLCD(ST7735, 26,
// 28, 24, 25, 22); // yUROBOT ARDUINO 屏幕
UTFT myGLCD(TFT_Modle, TFT_SDA, TFT_SCL, TFT_CS, TFT_RST, TFT_RS);
void setup() {
  randomSeed(analogRead(0));

  // Setup the LCD
  myGLCD.InitLCD(PORTRAIT);
  myGLCD.setFont(BigFont);
  myGLCD.setContrast(0);  //设置对比度
  myGLCD.fillScr(0, 0, 255);
  myGLCD.setColor(255, 255, 0);
  myGLCD.fillRoundRect(2, 40, 125, 158);
  myGLCD.setColor(255, 255, 255);
  myGLCD.setBackColor(255, 0, 0);
}

void loop() {


  // Clear the screen and draw the frame
 // myGLCD.clrScr();  //清除屏幕背景黑色

  // Set up the "Finished"-screen

  myGLCD.printNumI(myGLCD.getDisplayXSize(), CENTER, 46);
  myGLCD.printNumI(myGLCD.getDisplayYSize(), CENTER, 66);
  myGLCD.printNumF(3.1415926, 5, CENTER, 86);
  myGLCD.printNumF(3.14159261, 8, CENTER,106);
   myGLCD.printNumF(3.14159261, 7, CENTER,120);
  myGLCD.printNumI(millis(), CENTER, 140);
  

}
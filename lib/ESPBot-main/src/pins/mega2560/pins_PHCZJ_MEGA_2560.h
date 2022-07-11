//引脚编号

#pragma once

//******************电压检测引脚***************************//
#ifndef VOLT_PIN
#define VOLT_PIN A0  ///<电压检测引脚A0
#endif

//********************循迹传感器***************************//
#define BLACK HIGH  //循迹传感器检测到黑色为“HIGH”,灯灭
#define WHITE LOW   //循迹传感器检测到白色为“LOW”，灯亮
#define DARK HIGH   //循迹传感器检测到深色为“HIGH”,灯灭
#define LIGHT LOW   //循迹传感器检测到浅色为“LOW”，灯亮

#ifndef GRAYSCALE_DEFAULT_NUM
#define GRAYSCALE_DEFAULT_NUM 7  //循迹传感器默认数量
#endif

/*
传感器串口线（默认使用串口2）
5V	→	5V
GND	→	GND
TX	→	RX2
RX	→	TX2
7路传感器普通数字接口
5V	→	5V
GND	→	GND
OUT1	→	49
OUT2	→	47
OUT3	→	43
OUT4	→	41
OUT5	→	39
OUT6	→	37
OUT7	→	35
*/


#ifdef USE_GRAY_IO
#define GRAY_PIN1 49
#define GRAY_PIN2 47
#define GRAY_PIN3 43
#define GRAY_PIN4 41
#define GRAY_PIN5 39
#define GRAY_PIN6 37
#define GRAY_PIN7 35
//#define GRAY_PIN8  27
#endif

//******************手柄定义***************************//
#ifdef USE_PS2
#define PS2_DAT        49//  
#define PS2_CMD        47//
#define PS2_SEL        43//
#define PS2_CLK        41//
#endif
//******************轮子引脚定义***************************//
/**
 * 轮子定义Z字型定义
 * *               |               |
 *    MotorA (1)-->| +-- FRONT ---+|<-- MotorB (2)
 *       [5,6]     |       |       |     [8,7]
 *Encoder[2,51]    L |     |     | R     [3,53]
 *                 E |     |     | I
 *                 F |     |     | G
 *                 T |     |     | H
 *                   |     |     | T
 *                   |     |     |
 *                 |       |       |
 *    MotorC (3)-->| +-- BACK  ---+|<-- MotorD (4)
 *       [11,12]   |               |     [44,46]
 *Encoder[18,52]                           [19,50]
----------------------

ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/
//******************PWM引脚和电机驱动引脚***************************//
#ifdef USE_A4950_DRIVER
#define AIN1 5  // A电机控制PWM波
#define AIN2 6  // A电机控制PWM波

#define BIN1 8  // B电机控制PWM波
#define BIN2 7  // B电机控制PWM波

#define CIN1 11  // C电机控制PWM波
#define CIN2 12  // C电机控制PWM波

#define DIN1 44  // D电机控制PWM波
#define DIN2 46  // D电机控制PWM波
#endif

#define PWM_BITS 8  // PWM Resolution of the microcontroller
#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX

//******************编码器引脚***************************//

#define ENCODER_A 2  // A路电机编码器引脚AA，外部中断，中断号0
#define DIRECTION_A 51  // A路电机编码器引脚AB

#define ENCODER_B 3  // B路电机编码器引脚BA，外部中断，中断号1
#define DIRECTION_B 53  // B路电机编码器引脚BB

#define ENCODER_C 18  // C路电机编码器引脚CA，外部中断，中断号5
#define DIRECTION_C 52  // C路电机编码器引脚CB

#define ENCODER_D 19  // D路电机编码器引脚DA，外部中断，中断号4
#define DIRECTION_D 50  // D路电机编码器引脚DB

//******************OLED显示屏引脚SPI相关设置***************************//
/*
-Master In Slave Out（MISO）- Slave line，用于Slave向Master发送数据
-Master Out Slave In（MOSI）- Master line，用于Master向Slave发送数据
-Serial Clock（SCK）- 时钟脉冲，主设备用于同步数据传输
-Slave Select pin-
分配给所有的设备，用于enable/disable指定的设备，同时用于避免由于线路忙导致的错误传输。
OLED SPI接口定义：
1，  GND（电源负极)
2，  VCC（电源正极）
3，  SCL/CLK（时钟线）
4，  SDA/MOSI（数据线）
5， RES
6， DC （数据/命令）
7.  CS （片选）
七针模块有cs脚；直接连接就好了；如果是用六针的CS默认已经接地；不接就可以了
20201031
*/
#define OLED_DC 22
#define OLED_CLK 28
#define OLED_MOSI 26
#define OLED_RESET 24
//******************TFT显示屏引脚SPI相关设置***************************//
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
#define TFT_SCL 28
#define TFT_CS 24
#define TFT_RST 25
#define TFT_RS 22
#endif

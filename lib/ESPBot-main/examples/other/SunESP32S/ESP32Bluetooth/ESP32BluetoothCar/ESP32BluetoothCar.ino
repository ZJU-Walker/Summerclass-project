/**
 * @file ESP32_WiFiCar.ino 连接蓝牙开环控制小车
 * @author igcxl (igcxl@qq.com)
 * @brief 连接蓝牙开环控制小车
 * @note 连接蓝牙开环控制小车
 * @version 0.9
 * @date 2021-11-08
 * @copyright Copyright © Sunnybot.cn 2021
 *
 */

#include "BluetoothSerial.h"
#include "analogWrite.h"
//摇杆数据
int Lpad_x, Lpad_y, Rpad_x, Rpad_y;
int Vx,Vy,Wz;
int M1_Speed, M2_Speed, M3_Speed, M4_Speed;
int speed_max;
char BluetoothData;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;



//******************PWM引脚和电机驱动引脚***************************//

#define AIN1 32  // A电机控制PWM波
#define AIN2 33  // A电机控制PWM波

#define BIN1 25  // B电机控制PWM波
#define BIN2 26  // B电机控制PWM波

#define CIN1 27  // C电机控制PWM波
#define CIN2 14  // C电机控制PWM波//该端口重启时会输出高电平 需更换

#define DIN1 13  // D电机控制PWM波
#define DIN2 12  // D电机控制PWM波
//******************电机启动初始值 **********************//
int motorDeadZone =
    30;  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM 还需要添加限幅
**************************************************************************/
void Set_PWM(int motora, int motorb, int motorc, int motord) {
  if (motora > 0)
    analogWrite(AIN2, motora + motorDeadZone),
        analogWrite(AIN1, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(AIN2, 0), analogWrite(AIN1, 0);
  else if (motora < 0)
    analogWrite(AIN1, -motora + motorDeadZone),
        analogWrite(
            AIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(BIN2, motorb + motorDeadZone),
        analogWrite(BIN1, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(BIN2, 0), analogWrite(BIN1, 0);
  else if (motorb < 0)
    analogWrite(BIN1, -motorb + motorDeadZone),
        analogWrite(
            BIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorc > 0)
    analogWrite(CIN1, motorc + motorDeadZone),
        analogWrite(CIN2, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorc == 0)
    analogWrite(CIN2, 0), analogWrite(CIN1, 0);
  else if (motorc < 0)
    analogWrite(CIN2, -motorc + motorDeadZone),
        analogWrite(
            CIN1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motord > 0)
    analogWrite(DIN1, motord + motorDeadZone),
        analogWrite(DIN2, 0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motord == 0)
    analogWrite(DIN1, 0), analogWrite(DIN2, 0);
  else if (motord < 0)
    analogWrite(DIN2, -motord + motorDeadZone),
        analogWrite(
            DIN1,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(CIN1, OUTPUT);
  pinMode(DIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(CIN2, OUTPUT);
  pinMode(DIN2, OUTPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32BT"); //蓝牙模块名称
  Serial.println("蓝牙已启动，你可以配对了！");

}

void loop() {

  if(SerialBT.available()) 
  {
      /*--------------------------L_pad -----------------------*/
    BluetoothData = SerialBT.read();
    if(BluetoothData=='L')
    {
      while (BluetoothData!='#')
      {
        if(SerialBT.available())
        {
          BluetoothData=SerialBT.read();   //Get next character from bluetooth
          if(BluetoothData=='X')
            Lpad_x=SerialBT.parseInt();
          if(BluetoothData=='Y')
            Lpad_y=SerialBT.parseInt();
        }
      }
    }
 
      /*--------------------------R_pad -----------------------*/
    if(BluetoothData=='R')
    {
      while (BluetoothData!='#')
      {
        if(SerialBT.available())
        {
          BluetoothData=SerialBT.read();   //Get next character from bluetooth
          if(BluetoothData=='X')
            Rpad_x=SerialBT.parseInt();
          if(BluetoothData=='Y')
            Rpad_y=SerialBT.parseInt();
        }
      }
    }
 
    Serial.print("Lpad_x:");
    Serial.println(Lpad_x);
    Serial.print("Lpad_y:");
    Serial.println(Lpad_y);
    Serial.print("Rpad_x:");
    Serial.println(Rpad_x);
    Serial.print("Rpad_y:");
    Serial.println(Rpad_y);
/**
////////////摇杆坐标////////////
//    -----> Lpad_x
//    |
//    V Lpad_y+
////////////附体坐标系//////////
//         ^+Vx
//         |
// +Vy<----- 
//////////////转换/////////////
//vx=-Lpad_y
//vy=-Lpad_x
//W=-Rpad_x
**/
Vx=-Lpad_y;
Vy=-Lpad_x;
Wz=-Rpad_x;

    M1_Speed = (Vx-Vy-Wz) ;
    M2_Speed = (Vx+Vy+Wz) ;
    M3_Speed = (Vx+Vy-Wz) ;
    M4_Speed = (Vx-Vy+Wz) ;
    Set_PWM(M1_Speed, M2_Speed, M3_Speed, M4_Speed);
  }

}
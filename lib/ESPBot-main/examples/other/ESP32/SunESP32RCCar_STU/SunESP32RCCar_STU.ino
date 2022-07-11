
/*****************************************************************************
Copyright: 2022,Sunnybot.
File name: SunESP32Car_STU.ino
Description: wifi遥控小车
Author: Sunny
Version: 0.8
Date: 20220201
History: 修改历史记录列表， 每条修改记录应包括修改日期、修改者及修改内容简述。
20220301 Sunny 规范注释
*****************************************************************************/

//******************预编译包含使用库的头文件***************************//
#include <ArduinoWebsockets.h>  //Websocket库 双向通信协议
#include <ESPAsyncWebServer.h>  //异步服务器库
#include <WiFi.h>               //WiFi库
#include "analogWrite.h"  //模拟量读写库
#include "config.h"       //服务器实例配置
#include "web.h"          //网页文件

//******************WiFi热点配置***************************//
const char* ssid = "ESP32_Web";     // WiFi热点名称，请修改为"ESP32_学号"
const char* password = "12345678";  // WiFi密码
//启动后使用组内一台手机连接以下名称的Wifi
//连接后使用浏览器打开192.168.4.1
//使用网页摇杆进行控制

//******************电机驱动PWM引脚***************************//
#define LeftMotorAIN1   32  //左轮电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2   33  //左轮机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1   26  //右轮电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2   25  //  右轮电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚

//******************变量***************************//
int M1_Speed, M2_Speed;
int FBValue, LRValue, commaIndex;

//******************电机死区值 **********************//
int motorDeadZone =    0; 
 //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30,和电池电压、电机特性、PWM频率等有关，需要单独测试

/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-255,255]；motorb  b电机驱动PWM[-255,255]
**************************************************************************/
void RunMotors(int motora, int motorb) {
  if (motora > 0)
    analogWrite(LeftMotorAIN2, motora + motorDeadZone),
        analogWrite(LeftMotorAIN1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motora == 0)
    analogWrite(LeftMotorAIN2, 0), analogWrite(LeftMotorAIN1, 0);
  else if (motora < 0)
    analogWrite(LeftMotorAIN1, -motora + motorDeadZone),
        analogWrite(
            LeftMotorAIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30

  if (motorb > 0)
    analogWrite(RightMotorBIN2, motorb + motorDeadZone),
        analogWrite(RightMotorBIN1,
                    0);  //赋值给PWM寄存器根据电机响应速度与机械误差微调,
  else if (motorb == 0)
    analogWrite(RightMotorBIN2, 0), analogWrite(RightMotorBIN1, 0);
  else if (motorb < 0)
    analogWrite(RightMotorBIN1, -motorb + motorDeadZone),
        analogWrite(
            RightMotorBIN2,
            0);  //高频时电机启动初始值高约为130，低频时电机启动初始值低约为30
}

//******************处理http消息并控制电机 **********************//

void handle_message(WebsocketsMessage msg) {
  commaIndex = msg.data().indexOf(',');
  LRValue = msg.data().substring(0, commaIndex).toInt();  //获得摇杆左右方向返回值
  FBValue = msg.data().substring(commaIndex + 1).toInt();  //获得摇杆上下方向返回值

  //控制器 轮速PWM和摇杆值映射
  //参考映射规则 ：摇杆前后FBValue映射刚体Vx（油门），LRValue映射为刚体角速度w（方向）
  //摇杆左右LRValue代表映射w，该方向与小车坐标系相反故取反
  //对小车进行运动学分析并结合现场调试结果设置合理的参数。

  if(LRValue==100||LRValue==-100&&FBValue<=10&&FBValue>=-10)
  {  
    M1_Speed = (FBValue * 0 + LRValue * 2);
    M2_Speed = (FBValue * 0 - LRValue * 2);
  }
  else
  {
    M1_Speed = (FBValue * 1.8 + LRValue * 0.4);
    M2_Speed = (FBValue * 1.8 - LRValue * 0.4);
}
  //驱动车轮转动
  RunMotors(M1_Speed, M2_Speed);
}

void setup() {
  //IO引脚初始化
  pinMode(LeftMotorAIN1, OUTPUT);//将电机控制引脚配置为输出模式
  pinMode(LeftMotorAIN2, OUTPUT);//将电机控制引脚配置为输出模式

  pinMode(RightMotorBIN1, OUTPUT);//将电机控制引脚配置为输出模式
  pinMode(RightMotorBIN2, OUTPUT);//将电机控制引脚配置为输出模式

  //串口初始化
  Serial.begin(115200);

  // 建立WiFi热点 Create AP
  WiFi.softAP(ssid, password);
  //打印热点IP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // HTTP handler assignment
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse_P(
        200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  // 启动服务start server
  webserver.begin();
  server.listen(82);
  Serial.print("Is server live? ");
  Serial.println(server.available());
}

void loop() {
  //接受客户端连接
  auto client = server.accept();
  //等待消息
  client.onMessage(handle_message);

  while (client.available()) {
    client.poll();
  }
}
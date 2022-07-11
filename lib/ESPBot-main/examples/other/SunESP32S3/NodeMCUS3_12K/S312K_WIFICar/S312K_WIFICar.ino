#include <ArduinoWebsockets.h>  //Websocket库 双向通信协议
#include <ESPAsyncWebServer.h>  //异步服务器库
#include <WiFi.h>               //WiFi库
#include "config.h"       //服务器实例配置
#include "web.h"          //网页文件
#include "ESP32Car.h"          //ESP32小车库文件
//启动后连接以下Wifi,并使用浏览器打开192.168.4.1
//******************WiFi热点配置***************************//
const char* ssid = "ESP32_Sunnybot";     // Enter SSID 请修改为"ESP32_学号"
const char* password = "12345678";  // Enter Password


//******************变量***************************//
int M1Speed, M2Speed;
int FBValue, LRValue, commaIndex;


//处理http消息并控制电机 handle http messages 
void handle_message(WebsocketsMessage msg) {
  commaIndex = msg.data().indexOf(',');
  LRValue = msg.data().substring(0, commaIndex).toInt();  //摇杆左右方向返回值
  FBValue = msg.data().substring(commaIndex + 1).toInt();  //摇杆上下方向返回值

  //控制器 轮速PWM和摇杆值映射
  //参考映射规则 摇杆前后FBValue映射Vx
  //摇杆左右LRValue代表映射w，该方向与小车坐标系相反故取反
  //对小车进行运动学分析并结合现场调试结果设置合理的参数。

  //if(LRValue==100||LRValue==-100&&FBValue<=10&&FBValue>=-10)
  if(abs(LRValue)==100&&abs(FBValue)<=10)
  {  
    M1Speed = (FBValue * 0 + LRValue * 1.0);
    M2Speed = (FBValue * 0 - LRValue * 1.0);
  }
  else
  {
    M1Speed = (FBValue * 1.4 + LRValue * 0.3);
    M2Speed = (FBValue * 1.4 - LRValue * 0.3);
}
  //驱动车轮转动
  setSpeeds(M1Speed, M2Speed);
}

void setup() {
  //IO引脚初始化
  pinMode(M1PWM1, OUTPUT);//将电机控制引脚配置为输出模式
  pinMode(M1PWM2, OUTPUT);//将电机控制引脚配置为输出模式

  pinMode(M2PWM1, OUTPUT);//将电机控制引脚配置为输出模式
  pinMode(M2PWM2, OUTPUT);//将电机控制引脚配置为输出模式

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
#include <WiFi.h>
#include <WiFiUdp.h> //引用以使用UDP

const char *ssid = "FAST_201";
const char *password = "1234qwer";

WiFiUDP Udp;                      //创建UDP对象
unsigned int localUdpPort = 4321; //本地端口号

void setup()
{
  Serial.begin(115200);
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort); //启用UDP监听以接收数据
}

void loop()
{
  int packetSize = Udp.parsePacket(); //获取当前队首数据包长度
  if (packetSize)                     //如果有数据可用
  {
    char buf[packetSize];
    Udp.read(buf, packetSize); //读取当前包数据

    Serial.println();
    Serial.print("Received: ");
    Serial.println(buf);
    Serial.print("From IP: ");
    Serial.println(Udp.remoteIP());
    Serial.print("From Port: ");
    Serial.println(Udp.remotePort());

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //准备发送数据
    Udp.print("Received: ");    //复制数据到发送缓存
    Udp.write((const uint8_t*)buf, packetSize); //复制数据到发送缓存
    Udp.endPacket();            //发送数据
  }
}

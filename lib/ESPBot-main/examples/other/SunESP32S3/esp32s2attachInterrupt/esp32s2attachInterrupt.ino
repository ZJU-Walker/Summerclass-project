int port = 0;
void setup()
{
  // 初始化日志打印串口
  Serial.begin(115200);  
  // 配置中断引脚
  pinMode(port, INPUT|PULLUP );
  // 检测到引脚 26 下降沿，触发中断函数 blink
  attachInterrupt(port, blink, FALLING);
  Serial.println("\nstart irq test");
}

void loop()
{
}

// 中断函数
void blink()
{
  Serial.println("IRQ");
}

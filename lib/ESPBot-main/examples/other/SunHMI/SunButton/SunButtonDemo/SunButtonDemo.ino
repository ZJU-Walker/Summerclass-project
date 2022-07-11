/**
 * @file SunButtonDemo.ino 按键上拉模式测试
 * @author igcxl (igcxl@qq.com)
 * @brief
 * 通过按键控制串口0输出
 * @note
 * 按键输入引脚
 * 1.由于Arduino上电后，数字I/O管脚处于悬空状态，此时通过digitalRead()读到的是一个不稳定的值(可能是高，也可能是低)。
 * 所以通过pinMode()函数设置按键引脚为上拉输入模式。即使用内部上拉电阻，按键未按下时，引脚将为高电平，按键按下为低电平。
 * 2.按键在按下的过程中，有一段接触未接触的不稳定过程，即按键的抖动。简单粗暴的方法：在判断按键状态时，通过delay()延时跳过抖动的不稳定状态。
 * 3.可以不使用内部上拉电阻，在电路上添加按键的上拉电阻或下拉电阻，可达到相同效果。
 * @version 0.5
 * @date 2021-03-18
 * @copyright Copyright © Sunnybot.cn 2021
 *
 */
const int buttonPin = 30;  // 按键的管脚定义
int state = 0;  //打印到串口的数据，按钮按下的情况下state每次循环自增1

void setup() {
  pinMode(buttonPin, INPUT);  //设置按键管脚普通输入模式
  //pinMode(buttonPin, INPUT_PULLUP);  //设置按键管脚上拉输入模式
  Serial.begin(9600);                //用于串口输出
  Serial.println("按键测试开始");
}

void loop() {
  if (digitalRead(buttonPin) == LOW)  // 若按键被按下
  {
    printState();
  } else {
    state = 0;
    // printState();
  }
}

void printState()

{
  state++;

  Serial.println(state);
}
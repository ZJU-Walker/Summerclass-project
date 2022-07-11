#include <PS2X_lib.h>

//任务要求：遥控跑一圈
//******************电机驱动规格***************************//
//假设坐在车上驾驶，驾驶员左手为左，右手为右侧。
//差速小车，三路循迹传感器
#define LeftMotorAIN1 6  // A电机控制PWM波,左电机控制引脚1，根据实际接线修改引脚
#define LeftMotorAIN2 5  // A电机控制PWM波,左电机控制引脚2，根据实际接线修改引脚

#define RightMotorBIN1 8  // B电机控制PWM波,右电机控制引脚1，根据实际接线修改引脚
#define RightMotorBIN2 7  // B电机控制PWM波,右电机控制引脚2，根据实际接线修改引脚
//********************循迹传感器***************************//
#define LeftGrayscale A1  //左循迹传感器引脚，根据实际接线修改引脚
#define MiddleGrayscale A2  //中循迹传感器引脚，根据实际接线修改引脚
#define RightGrayscale A3  //左循迹传感器引脚，根据实际接线修改引脚
#define BLACK HIGH  //循迹传感器检测到黑色为“HIGH”,灯灭（黑）
#define WHITE LOW  //循迹传感器检测到白色为“LOW”，灯亮（白）

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT 49
#define PS2_CMD 47
#define PS2_SEL 43
#define PS2_CLK 41

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures false
//#define rumble      true
#define rumble false

PS2X ps2x;  // create PS2 Controller Class

// right now, the library does NOT support hot pluggable controllers, meaning
// you must always either restart your Arduino after you connect the controller,
// or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

// Reset func
// Arduino有一个名为resetFunc的内置函数，我们声明函数地址为0，当我们执行此功能时，Arduino将自动重置。
void (*resetFunc)(void) = 0;
//=======================智能小车的基本动作=========================
void moveForward()  // 前进
{
  // 左电机前进，PWM比例，0~255调速，左右轮差异略增减，电机空载死区0-30
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机前进，PWM比例，0~255调速，左右轮差异略增减
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void moveBackward()  //后退
{
  //左轮后退，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);
  //右轮后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);
}

void turnLeft()  //左转(左轮不动，右轮前进)
{
  //右电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 0);
  //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void spinLeft()  //原地左转(左轮后退，右轮前进)
{
  //左轮后退PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 0);
  analogWrite(LeftMotorAIN2, 100);
  //右电机前进，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 100);
  analogWrite(RightMotorBIN2, 0);
}

void turnRight()  //右转(右轮不动，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 0);
}

void spinRight()  //原地右转(右轮后退，左轮前进)
{
  //左电机前进，PWM比例0~255调速
  analogWrite(LeftMotorAIN1, 100);
  analogWrite(LeftMotorAIN2, 0);
  //右电机后退，PWM比例0~255调速
  analogWrite(RightMotorBIN1, 0);
  analogWrite(RightMotorBIN2, 100);
}

void brake()  //急刹车，停车
{
  digitalWrite(RightMotorBIN1, HIGH);
  digitalWrite(RightMotorBIN2, HIGH);
  digitalWrite(LeftMotorAIN1, HIGH);
  digitalWrite(LeftMotorAIN2, HIGH);
}
void stop()  //普通刹车，停车，惯性滑行
{
  digitalWrite(RightMotorBIN1, LOW);
  digitalWrite(RightMotorBIN2, LOW);
  digitalWrite(LeftMotorAIN1, LOW);
  digitalWrite(LeftMotorAIN2, LOW);
}

void setup() {
  pinMode(LeftMotorAIN1, OUTPUT);  //初始化电机驱动IO为输出方式
  pinMode(LeftMotorAIN2, OUTPUT);
  pinMode(RightMotorBIN1, OUTPUT);
  pinMode(RightMotorBIN2, OUTPUT);
  pinMode(LeftGrayscale, INPUT);    //定义左循迹传感器为输入
  pinMode(MiddleGrayscale, INPUT);  //定义中循迹传感器为输入
  pinMode(RightGrayscale, INPUT);   //定义右循迹传感器为输入
  Serial.begin(115200);

  delay(1000);  // added delay to give wireless ps2 module some time to startup,
               // before configuring it

  // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  // setup pins and settings: GamePad(clock, command, attention, data,
  // Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures,
                              rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println(
        "Try out all the buttons, X will vibrate the controller, faster as you "
        "press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println(
        "Note: Go to www.billporter.info for updates and to report bugs.");
  } else if (error == 1)
    Serial.println(
        "No controller found, check wiring, see readme.txt to enable debug. "
        "visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println(
        "Controller found but not accepting commands. see readme.txt to enable "
        "debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println(
        "Controller refusing to enter Pressures mode, may not support it. ");

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.println("Unknown Controller type found ");
      break;
    case 1:
      Serial.println("DualShock Controller found ");
      break;
    case 2:
      Serial.println("GuitarHero Controller found ");
      break;
    case 3:
      Serial.println("Wireless Sony DualShock Controller found ");
      break;
  }
}

void loop() {
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */

  // DualShock Controller
  ps2x.read_gamepad(false, vibrate);  // read controller and set large motor
                                      // to spin at 'vibrate' speed

  if (ps2x.Button(PSB_START))  // will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if (ps2x.Button(PSB_SELECT)) Serial.println("Select is being held");
// 思考题1 同时按下上和左，小车产生什么动作？
//参考答案：if else if
// 思考题2 前进时如果无法走直线如何处理？

// 思考题3 如何让小车后退时和向前运行一样灵活？
  if (ps2x.Button(PSB_PAD_UP)) {  // will be TRUE as long as button is pressed

    Serial.print("Up held this hard: ");
    // Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    moveForward();  //前进
  } else if (ps2x.Button(PSB_PAD_RIGHT)) {
    Serial.print("Right held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    //Do something
  } else if (ps2x.Button(PSB_PAD_LEFT)) {
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    //Do something
  } else if (ps2x.Button(PSB_PAD_DOWN)) {
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
   //Do something
  } else {
    stop();
  }

  vibrate = ps2x.Analog(
      PSAB_CROSS);  // this will set the large motor vibrate speed based on
                    // how hard you press the blue (X) button
  if (ps2x.NewButtonState()) {  // will be TRUE if any button changes state
                                // (on to off, or off to on)
    if (ps2x.Button(PSB_L3)) Serial.println("L3 pressed");
    if (ps2x.Button(PSB_R3)) Serial.println("R3 pressed");
    if (ps2x.Button(PSB_L2)) Serial.println("L2 pressed");
    if (ps2x.Button(PSB_R2)) Serial.println("R2 pressed");
    if (ps2x.Button(PSB_TRIANGLE)) Serial.println("Triangle pressed");
  }

  if (ps2x.ButtonPressed(
          PSB_CIRCLE))  // will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if (ps2x.NewButtonState(
          PSB_CROSS))  // will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if (ps2x.ButtonReleased(
          PSB_SQUARE))  // will be TRUE if button was JUST released
    Serial.println("Square just released");

  if (ps2x.Button(PSB_L1) ||
      ps2x.Button(PSB_R1)) {  // print stick values if either is TRUE
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY),
                 DEC);  // Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);
  }

  delay(20);
}

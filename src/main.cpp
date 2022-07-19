#include <Ticker.h>  //定时中断
#include "OOPConfig.h"
#include <esp_now.h>
#include <WiFi.h>


#ifdef USE_MPU6050_DMP
//使用带DMP的支持ESP32的MPU6050库
#include "I2Cdev.h"  //点击自动打开管理库页面并安装: http://librarymanager/All#MPU6050
#include "MPU6050_6Axis_MotionApps_V6_12.h"  //和I2Cdev.h同时安装了,原版I2Cdev无法使用
MPU6050 mpu;
//****************** MPU6050***************************//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;    // return status after each device operation (0 = success,
                      // !0 = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;    // [w, x, y, z]         quaternion container
VectorInt16 aa;  // [x, y, z]            accel sensor measurements
VectorInt16 gy;  // [x, y, z]            gyro sensor measurements
VectorInt16
    aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16
    aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float
    ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//小车偏航角
float initialYaw, initialYawRad, newYaw, newYawRad, realYaw, realYawRad;
bool isFirst = 1;
float angle2rad(float angle) { return angle * M_PI / 180; }
float rad2angle(float rad) { return rad * 180 / M_PI; }


//其他变量——运动状态全局变量
#define SINGLEGRAY_PIN 12  //12引脚的宏定义
int generalstate = 1;//用于记录小车总的运动状态

int motionstate = 1;//用于切换小车循迹的运行状态
int motionstatepre = 1;

//其他变量——黑线读取
int linenember=0;//用于记录通过的黑线数量
int ColorP12=0;//用于读取P12的值，黑为1
int KANGGANRAO=0;//用于抗干扰
int Record=0;//用于记录上一次黑线的值

//****************** MPU6050***************************//
#endif

#ifndef DEBUG
//#define DEBUG
#endif
#ifndef DEBUG_INFO
#define DEBUG_INFO
#endif
// 新建Grayscale 实例 默认参数
Grayscale GraySensors;
// GraySensors(/*Num*/7,/**uartPort*/&Serial2,/*isDarkHigh*/true,/*isOffset*/false);
// //7路灰度使用串口2 uart输出量的结构体
Grayscale::strOutput GrayUartOutputIO;

//新建小车底盘运动学实例
Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE,
                      LR_WHEELS_DISTANCE);
Kinematics::output rpm;
Kinematics::output pluses;

float linear_vel_x = 0;            // m/s
float linear_vel_y = 0;            // m/s
float angular_vel_z = 0;           // rad/s
unsigned long previousMillis = 0;  // will store last time run
const long period = 3000;          // period at which to run in ms
/***************** 定时中断参数 *****************/
Ticker timer1;  // 中断函数
bool timer_flag = 0;
//******************创建4个编码器实例***************************//
SunEncoder ENC[WHEELS_NUM] = {
    SunEncoder(M1ENA, M1ENB), SunEncoder(M2ENA, M2ENB),
    SunEncoder(M3ENA, M3ENB), SunEncoder(M4ENA, M4ENB)};

long targetPulses[WHEELS_NUM] = {0, 0, 0, 0};  //四个车轮的目标计数
long feedbackPulses[WHEELS_NUM] = {0, 0, 0,
                                   0};  //四个车轮的定时中断编码器四倍频计数
double outPWM[WHEELS_NUM] = {0, 0, 0, 0};

//*****************创建4个速度PID实例***************************//
float Kp = 15, Ki = 0.04, Kd = 0.01;
PID VeloPID[WHEELS_NUM] = {
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd),
    PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd), PID(PWM_MIN, PWM_MAX, Kp, Ki, Kd)};

//*****************创建1个4路电机对象***************************//
BDCMotor motors;

//*****************运行状态枚举**************************//
// enum CARMOTION { PAUSE, LEFTWARD, FORWARD, RIGHTWARD, BACKWARD, STOP, RIGHT, TURNLEFT, TURNRIGHT };
//enum CARMOTION direction = PAUSE;
bool carDirection = 0;
unsigned long readTime, readPeriod, ISRPeriod;
int print_Count = 0;
//定时器中断处理函数,其功能主要为了输出编码器得到的数据并刷新电机输出
void timerISR() {
  // ISRPeriod = micros();
  //获取电机脉冲数（速度）
  timer_flag = 1;  //定时时间达到标志
  print_Count++;
  //  /获取电机目标速度计数=数
  targetPulses[0] = pluses.motor1;
  targetPulses[1] = pluses.motor2;
  targetPulses[2] = pluses.motor3;
  targetPulses[3] = pluses.motor4;
  for (int i = 0; i < WHEELS_NUM; i++) {
    feedbackPulses[i] = ENC[i].read();

    ENC[i].write(0);  //复位
    outPWM[i] = VeloPID[i].Compute(targetPulses[i], feedbackPulses[i]);
  }
  motors.setSpeeds(outPWM[0], outPWM[1], outPWM[2], outPWM[3]);
  // ISRPeriod = micros() - ISRPeriod;
}

//*********************************通信*******************************//
uint8_t broadcastAddress[] = {0x94,0x3C,0xC6,0x11,0x07,0x08};//机械臂板子MAC地址
// // 信息结构体类型
typedef struct struct_message {
  int b;
} struct_message;
// 创建一个结构体变量
struct_message Datasendarm;//发送结构体
struct_message Datarecvarm;//接收结构体
// 回调函数,函数将在发送消息时执行。此函数告诉我们信息是否成功发送;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// 回调函数,当收到消息时会调佣该函数
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Datarecvarm, incomingData, sizeof(Datarecvarm));
}
//发送函数，小车向机械臂发送int 类型表示小车运动结束，到达一个地点，机械臂开始运动
//发送int state 1               2              3             4
//        到达二维码区     到达原料区       到达半成品区    到达放置区
void sendfunction(int state){
  //发送信息到指定ESP32上
  Datasendarm.b = state;
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Datasendarm, sizeof(Datasendarm));
   
 //判断是否发送成功
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}
/***************************************set up*************************************************/
void setup() {
  pinMode(SINGLEGRAY_PIN, INPUT_PULLUP); //12引脚初始化
  Datarecvarm.b=0;
  Datasendarm.b=0;
  motors.init();
  motors.flipMotors(
      FLIP_MOTOR[0], FLIP_MOTOR[1], FLIP_MOTOR[2],
      FLIP_MOTOR[3]);  //根据实际转向进行调整false or true 黑色PCB电机
                       // false  绿色PCB电机true 翻转信息包含在OOPConfig
  for (int i = 0; i < WHEELS_NUM; i++) {
    ENC[i].init();
    ENC[i].flipEncoder(FLIP_ENCODER[i]);
  }
  delay(100);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(BAUDRATE);
  //******************ESP32_now setup******************//
  // // 设置WIFI模式为STA模式，即无线终端
  WiFi.mode(WIFI_STA);
  // //  初始化ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  //注册回调函数
  esp_now_register_send_cb(OnDataSent);
  // 注册通信频道
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  //通道
  peerInfo.encrypt = false;//是否加密 False
    peerInfo.ifidx = WIFI_IF_STA;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  sendfunction(0);
  //注册接收信息的回调函数
  esp_now_register_recv_cb(OnDataRecv);

  //陀螺仪初始化
  Wire.begin();
  // Wire.setClock(400000);
  while (!Serial)
    ;
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  //运行例程中的IMU_Zero获得，注意波特率一致问题
  mpu.setXAccelOffset(XAccelOffset);
  mpu.setYAccelOffset(YAccelOffset);
  mpu.setZAccelOffset(ZAccelOffset);
  mpu.setXGyroOffset(XGyroOffset);
  mpu.setYGyroOffset(YGyroOffset);
  mpu.setZGyroOffset(ZGyroOffset);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(20);
    mpu.CalibrateGyro(20);
    Serial.println("打印补偿值：");
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  delay(100);  //延时等待初始化完成

  // Serial.println("Sunnybot 麦轮寻线测试，请按下对应按键开始测试");
  //   u8g2.clearBuffer();               // clear the internal memory
  // u8g2.setFont(u8g2_font_7x14_tf);  // choose a suitable font
  // u8g2.setCursor(0, 16);
  // u8g2.print("Press the BTN");
  // u8g2.sendBuffer();
  bool pflag=1;
  while (digitalRead(BUTTON_PIN) == HIGH) {
    if(pflag){
    Serial.print("请按对应按键:");
    Serial.println(!digitalRead(BUTTON_PIN));
    pflag=0;
    }
  }
  /***************** 定时中断 *****************/
  timer1.attach_ms(TIMER_PERIOD, timerISR);  // 打开定时器中断
  interrupts();

  readPeriod = micros();
}

/**********************************************小车运行分函数定义*********************************************/

void PAUSE(unsigned long currentMillis, int PAUSETIME){
  linear_vel_x = 0;  // m/sz
  if (GrayUartOutputIO.ioCount) {
    linear_vel_y = -0.005 * GrayUartOutputIO.offset;  // m/s
  }
  angular_vel_z = -realYawRad * 2;   //*原始代码为2
  //使用millis函数进行定时控制，代替delay函数
  if (currentMillis - previousMillis >= PAUSETIME) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    isFirst=1;
  }
}

void UNZPAUSE(unsigned long currentMillis, int PAUSETIME){
  linear_vel_x = 0;  // m/sz
  if (GrayUartOutputIO.ioCount) {
    linear_vel_y = -0.004 * GrayUartOutputIO.offset;  // m/s
  }
  angular_vel_z = -realYawRad * 0.0001;   //*原始代码为2
  //使用millis函数进行定时控制，代替delay函数
  if (currentMillis - previousMillis >= PAUSETIME) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    isFirst=1;
  }
}


void moveForward(unsigned long currentMillis,int BlackNumber,float kp,float kpz,float speed){
  linear_vel_x = speed;  // m/s
  if (GrayUartOutputIO.ioCount) {
     linear_vel_y = -kp * GrayUartOutputIO.offset;
    } else {
  linear_vel_y = -linear_vel_y;
    }                   // m/s
  angular_vel_z = -realYawRad * 25 * kpz;
  if (linenember==BlackNumber) {
    previousMillis = currentMillis;
    linenember=0;
    motionstate = motionstate+1;//进入下一阶段
    }
}
void moveBackward(unsigned long currentMillis,int BlackNumber,float kp){
  linear_vel_x = -0.18;  // m/s
  if (GrayUartOutputIO.ioCount) {
     linear_vel_y = -kp * GrayUartOutputIO.offset;
    } else {
  linear_vel_y = -linear_vel_y;
    }                   // m/s
  angular_vel_z = -realYawRad ;
  if (linenember==BlackNumber) {
    previousMillis = currentMillis;
    linenember=0;
    motionstate = motionstate+1;//进入下一阶段
    }
}
void turnLeft(unsigned long currentMillis,float kp,float turntime){
  linear_vel_x = 0;  // m/s
  linear_vel_y = 0;  // m/s
  angular_vel_z = 2;  // rad/s
  if (currentMillis - previousMillis >= (0.34 * turntime)) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    linenember = 0;
    isFirst = 1;
  }
}

void turnRight(unsigned long currentMillis, float kp,float turntime){
  linear_vel_x = 0;  // m/s
  linear_vel_y = 0;  // m/s
  angular_vel_z = -2;  // rad/s
  if (currentMillis - previousMillis >= (0.34 * turntime)) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    linenember = 0;
    isFirst = 1;
  }
}

void Moveforwardtime(unsigned long currentMillis,float kp,float kpz,float movetime){
  linear_vel_x = 0.3;  // m/s
  if (GrayUartOutputIO.ioCount) {
    linear_vel_y = -kp * GrayUartOutputIO.offset;
  } else {
  linear_vel_y = -linear_vel_y;
  }                   // m/s
  angular_vel_z = -realYawRad * kpz;
  if (currentMillis - previousMillis >= movetime) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    linenember = 0;
    isFirst = 1;
  }
}

void Moveleftforwardtime(unsigned long currentMillis,float kp,float kpz,float movetime){
  linear_vel_y = 0.3; 
  linear_vel_x = 0; // m/s
  angular_vel_z = -realYawRad * kpz;
  if (currentMillis - previousMillis >= movetime) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    linenember = 0;
    isFirst = 1;
  }
}

void STOP(unsigned long currentMillis, int STOPTIME){
//急停
  linear_vel_x = 0;  // m/s
  linear_vel_y = 0;  // m/s
  angular_vel_z = 0;
  motors.motorsBrake();
  if (currentMillis - previousMillis >= STOPTIME) {
    previousMillis = currentMillis;
    motionstate = motionstate+1;//进入下一阶段
    linenember = 0;
    }
}


/************************************************小车运行阶段函数定义***********************************************/
void generalstateplus(){
    if(motionstatepre==1){
      motionstatepre=motionstate;
    }
    if(motionstate>motionstatepre){
        generalstate++;
        motionstate=1;
        motionstatepre=1;
    }
    else{
        motionstatepre=motionstate;
    }
}
//起步右转到达二维码处
void start(unsigned long currentMillis,float kp,float kpz){
    switch(motionstate){
      case 1: //第一阶段 出发//
        moveForward(currentMillis,2,0,0,0.2);
        break;
      case 2:
        turnRight(currentMillis,kp,3050);
        break;
      case 3:
        STOP(currentMillis,100);
        break;
      case 4:
        PAUSE(currentMillis, 3000);
        break;
      case 5:
        moveForward(currentMillis,2,kp,0.02,0.25);
        break;
      case 6:
        STOP(currentMillis,100);
        break;
      case 7:
        PAUSE(currentMillis,2000);
        break;
      case 8:
        moveBackward(currentMillis,1,kp);//二维码停
        break;
      case 9:
        STOP(currentMillis,100);
        break;
      case 10:
        PAUSE(currentMillis,2000);
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        sendfunction(1);
        if(Datarecvarm.b == 1){
          currentMillis = millis();
          previousMillis = currentMillis;
          sendfunction(0);
          Datarecvarm.b=0;
          motionstate++;
        }
        break;
      case 13:
        generalstate++;
        motionstate=1;
        break;
    }
}

//二维码到达原料区
void rawmaterial1(unsigned long currentMillis,float kp,float kpz){
    switch(motionstate){
      case 1: 
        moveForward(currentMillis,5,kp,0.03,0.2);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      case 4:
        moveBackward(currentMillis,1,kp);
        break;
      case 5:
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,3000);
        break;
      case 7:
        turnRight(currentMillis,kp,3030);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        UNZPAUSE(currentMillis,3000);
        break;
      case 10:
        Moveforwardtime(currentMillis,0.001,0.005,830);
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        UNZPAUSE(currentMillis,3000);
        break;
      case 13:
        STOP(currentMillis,100);
        break;
      case 14:
        sendfunction(2);
        if(Datarecvarm.b == 1){
          currentMillis = millis();
          previousMillis = currentMillis;
          sendfunction(0);
          Datarecvarm.b = 0;
          motionstate++;
        }
        break;
      case 15:
        generalstate++;
        motionstate=1;
        break; 
    }
}

//原料区到达粗加工区
void roughmachining1(unsigned long currentMillis,float kp,float kpz){
    switch(motionstate){
      case 1:
        moveBackward(currentMillis,3,kp);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      case 4:
        moveForward(currentMillis,1,kp,0.052,0.15);
        break;
      case 5:
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,2000);
        break;
      case 7:
        turnLeft(currentMillis,kp,3000);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        PAUSE(currentMillis,3000);
        break;
      case 10:
        Moveforwardtime(currentMillis,0.0008,0.002,830);
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        UNZPAUSE(currentMillis,3000);
        break;
      case 13:
        STOP(currentMillis,100);
        break;
      case 14:
        sendfunction(3);
        if(Datarecvarm.b == 1){
          currentMillis = millis();
          previousMillis = currentMillis;
          sendfunction(0);
          Datarecvarm.b = 0;
          motionstate++;
        }
        break;
      case 15:
        generalstate++;
        motionstate=1;
        break;
    }
}

//粗加工区到达半成品区
void semifinishedproduct1(unsigned long currentMillis,float kp,float kpz){
    switch(motionstate){
    case 1:
      moveForward(currentMillis,3,kp,kpz,0.3);
      break;
    case 2:
      STOP(currentMillis,100);
      break;
    case 3:
      PAUSE(currentMillis,2000);
      break;
    case 4:
      turnLeft(currentMillis,kp,3100);
      break;
    case 5:
      STOP(currentMillis,100);
      break;  
    case 6:
      PAUSE(currentMillis,3000);
      break;
    case 7:
      moveForward(currentMillis,4,kp,kpz,0.3);
      break;
    case 8:
      STOP(currentMillis,100);
      break;
    case 9: 
      PAUSE(currentMillis,1000);
      break;
    case 10:
      moveBackward(currentMillis,1,kp);//半成品区停
      break;
    case 11:
      STOP(currentMillis,100);
      break;
    case 12:
      PAUSE(currentMillis,3000);
      generalstateplus();
      break;
    }
}

//半成品区到达原料区
void rawmaterial1(unsigned long currentMillis,float kp,float kp2,float kpz){
    switch(motionstate){
      case 1:
        turnLeft(currentMillis,kp2,3050);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      case 4:
        moveForward(currentMillis,6,kp,0.05,0.3);
        break;
      case 5:
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,2000);
        break;
      //important
      case 7:
        turnLeft(currentMillis,kp2,3130);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        PAUSE(currentMillis,3000);
        break;
      case 10:
        moveForward(currentMillis,3,kp,kpz,0.3);
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        PAUSE(currentMillis,2000);
        break;
      case 13:
        moveBackward(currentMillis,1,kp);//原料区停
        break;
      case 14:
        STOP(currentMillis,100);
        break;
      case 15:
        PAUSE(currentMillis,3000);
        generalstateplus();
        break;
    }
}
    
//原料区到达粗加工区
void roughmachining2(unsigned long currentMillis,float kp,float kp2,float kpz){
    switch(motionstate){
      case 1:
        moveForward(currentMillis,2,kp,kpz,0.3);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      //important
      case 4:
        turnLeft(currentMillis,kp2,3150);
        break;
      case 5: 
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,3000);
        break;
      case 7:
        moveForward(currentMillis,4,kp,kpz,0.3);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        PAUSE(currentMillis,2000);
        break;
      case 10:
        moveBackward(currentMillis,1,kp);//粗加工区停
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        PAUSE(currentMillis,3000);
        generalstateplus();
        break;
    }
}

//粗加工区到达半成品区
void semifinishedproduct1(unsigned long currentMillis,float kp,float kp2,float kpz){
    switch(motionstate){
      case 1:
        moveForward(currentMillis,3,kp,kpz,0.3);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      case 4:
        turnLeft(currentMillis,kp2,3200);
        break;
      case 5:
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,3000);
        break;
      case 7:
        moveForward(currentMillis,4,kp,kpz,0.3);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        PAUSE(currentMillis,2000);
        break;
      case 10:
        moveBackward(currentMillis,1,kp);//半成品区2停
        break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        PAUSE(currentMillis,3000);
        generalstateplus();
        break;
    }
}    

//半成品区到达终点
void destination(unsigned long currentMillis,float kp,float kp2,float kpz){
    switch(motionstate){
      case 1:
        moveForward(currentMillis,4,kp,kpz,0.3);
        break;
      case 2:
        STOP(currentMillis,100);
        break;
      case 3:
        PAUSE(currentMillis,2000);
        break;
      case 4:
        turnRight(currentMillis,kp2,3100);
        break;
      case 5:
        STOP(currentMillis,100);
        break;
      case 6:
        PAUSE(currentMillis,3000);
        break;
      case 7:
        moveForward(currentMillis,2,kp,kpz,0.3);
        break;
      case 8:
        STOP(currentMillis,100);
        break;
      case 9:
        PAUSE(currentMillis,2000);
        break;
      case 10:
        Moveforwardtime(currentMillis,kp,kpz,600);
       break;
      case 11:
        STOP(currentMillis,100);
        break;
      case 12:
        PAUSE(currentMillis,2000);
        break;
      case 13:
        Moveleftforwardtime(currentMillis,kp,kpz,650);
        break;
      case 14:
        STOP(currentMillis,100);
        break;
      case 15:
        PAUSE(currentMillis,3000);
        generalstateplus();
        break;
    }
}


/**********************************************loop**************************************************/
void loop() {

  //黑线数计数
  ColorP12=digitalRead(SINGLEGRAY_PIN);
  if (ColorP12==1){
    KANGGANRAO=KANGGANRAO+1;//抗干扰达到3次证明确实是黑线
  }
  if (ColorP12==1&&KANGGANRAO==4){
    Record=ColorP12;
  }
  if(ColorP12==0&&Record==1){//出黑色区域
    linenember=linenember+1;
    KANGGANRAO=0;
    Record=0;
  }

  if (micros() - readPeriod > 5000)  //每5ms读取一次
  {
    readPeriod = micros();
    readTime = micros();
    GrayUartOutputIO = GraySensors.readUart();  //读取串口数字量数据
    readTime = micros() - readTime;
  }

  //****************** MPU6050***************************//
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  newYawRad =
      -ypr[0];  //传感器库返回方向和定义方向相反，所以取负号，返回值为弧度
  if (isFirst) {
    initialYawRad = newYawRad;
    isFirst = 0;
  }
  realYawRad = (newYawRad - initialYawRad);
  realYaw = rad2angle(realYawRad);
  initialYaw = rad2angle(initialYawRad);
  newYaw = rad2angle(newYawRad);

  //****************** MPU6050***************************//
  // realYaw=57.29578*realYawRad;
  // realYaw = newYaw - initialYaw;
  // realYawRad=PI/180*realYaw
  // realYawRad = 0.017453 * realYaw;
  //角速度比例环 vz=k*errZ; errZ=期望-实际 (0 - realYaw)
  //angular_vel_z = -realYaw * 0.1;
  //****************** MPU6050***************************//

  unsigned long currentMillis = millis();  // store the current time
    float kp = 0.01;
    float kp2 = 0.0095;
#ifndef DEBUG
  kp = 0.002;  //
#endif
    float kpz = 0.05;

  //使用有限状态机方式前后走
 
  switch (generalstate) {
    case 1: //出发到达二维码处
      start(currentMillis,kp,kpz);
      break;
    case 2:
      rawmaterial1(currentMillis,kp, kpz);
      break;
    case 3:
      roughmachining1(currentMillis,kp, kpz);
      break;
    case 4:
      semifinishedproduct1(currentMillis,kp, kpz);
      break;
    case 5:
      rawmaterial1(currentMillis,kp,kp2, kpz);
      break;
    case 6:
      roughmachining2(currentMillis,kp,kp2, kpz);
      break;
    case 7:
      semifinishedproduct1(currentMillis,kp,kp2, kpz);
      break;
    case 8:
      destination(currentMillis,kp,kp2, kpz);
      break;
      
    default:      //停止
      linear_vel_x = 0;  // m/s
      if (GrayUartOutputIO.ioCount) {
        linear_vel_y = -kp * GrayUartOutputIO.offset;  // m/s
      }
      angular_vel_z = 0;  // rad/s
      break;
  }


//角度闭环
// angular_vel_z = -realYaw * 0.1;
  // given the required velocities for the robot, you can calculate
  // the rpm or pulses required for each motor 逆运动学
  // rpm = kinematics.getRPM(linear_vel_x, linear_vel_y, angular_vel_z);
  pluses = kinematics.getPulses(linear_vel_x, linear_vel_y, angular_vel_z);
// #ifdef DEBUG_INFO
//   Serial.print((String) "黑色点数：" + GrayUartOutputIO.ioCount +
//                "点变化数:" + GrayUartOutputIO.ioChangeCount + "偏移量：" +
//                GrayUartOutputIO.offset);
//   Serial.print("二进制IO值:");
//   Serial.print(GrayUartOutputIO.ioDigital, BIN);
//   Serial.print("ReadTime:");
//   Serial.print(readTime);
//   Serial.println("微秒");
//   Serial.print("Time:");
//   Serial.print(millis());
//     Serial.print("realYaw(度):");
//   Serial.println(realYaw);
//   // Serial.print("ISRPeriod:");
//   // Serial.println(ISRPeriod);
// #endif
  if (print_Count >= 50)  //打印控制，控制周期500ms
  {
#ifdef DEBUG
// #ifdef USE_OLED
//     u8g2.clearBuffer();                  // clear the internal memory
//     u8g2.setFont(u8g2_font_ncenB12_tf);  // choose a suitable font
//     u8g2.setCursor(0, 16);
//     u8g2.print("offset:");
//     u8g2.print(GrayUartOutputIO.offset);
//     u8g2.setCursor(0, 32);
//     u8g2.print("IOCnt:");
//     u8g2.print(GrayUartOutputIO.ioCount);
//     u8g2.setCursor(0, 48);
//     u8g2.print("vel_x:");
//     u8g2.print(linear_vel_x);
//     u8g2.setCursor(0, 64);
//     u8g2.print("vel_y:");
//     u8g2.print(linear_vel_y);
//     u8g2.sendBuffer();
// #endif

//     //串口输出目标值
//     Serial.print(" FL: ");
//     Serial.print(pluses.motor1);
//     Serial.print(",");
//     Serial.print(" FR: ");
//     Serial.print(pluses.motor2);
//     Serial.print(",");
//     Serial.print(" RL: ");
//     Serial.print(pluses.motor3);
//     Serial.print(",");
//     Serial.print(" RR: ");
//     Serial.println(pluses.motor4);
//     //串口输出反馈值
//     Serial.print(" FLF: ");
//     Serial.print(feedbackPulses[0]);
//     Serial.print(",");
//     Serial.print(" FRF: ");
//     Serial.print(feedbackPulses[1]);
//     Serial.print(",");
//     Serial.print(" RLF: ");
//     Serial.print(feedbackPulses[2]);
//     Serial.print(",");
//     Serial.print(" RRF: ");
//     Serial.println(feedbackPulses[3]);
//     //串口输出 IO输出PWM值
//     Serial.print(" FLP: ");
//     Serial.print(outPWM[0]);
//     Serial.print(",");
//     Serial.print(" FRP: ");
//     Serial.print(outPWM[1]);
//     Serial.print(",");
//     Serial.print(" RLP: ");
//     Serial.print(outPWM[2]);
//     Serial.print(",");
//     Serial.print(" RRP: ");
//     Serial.println(outPWM[3]);

#endif
    print_Count = 0;
  }
}


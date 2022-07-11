// ---------------------------------------------------------------------------
// This Arduino sketch accompanies the OpenBot Android application. 
// By Matthias Mueller, Intelligent Systems Lab, 2020
//
// The sketch has the following functinonalities:
//  - receive control commands from Android application (USB serial)从安卓app接收控制指令
//. - produce low-level controls (PWM) for the vehicle
//  - toggle left and right indicator signals 
//  - wheel odometry based on optical speed sensors
//  - estimate battery voltage via voltage divider
//  - estimate distance based on sonar sensor 
//  - send sensor readings to Android application (USB serial)
//  - display vehicle status on OLED
//
// Dependencies: Install via "Tools --> Manage Libraries" (type library name in the search field)
//  - Interrupts: PinChangeInterrupt by Nico Hood (read speed sensors and sonar)
//  - OLED: Adafruit_SSD1306 & Adafruit_GFX (display vehicle status)
// Contributors:
//  - October 2020: OLED display support by Ingmar Stapel
// ---------------------------------------------------------------------------

// PIN_PWM_L1,PIN_PWM_L2,PIN_PWM_R1,PIN_PWM_R2  Low-level control of left DC motors via PWM 
// PIN_SPEED_L, PIN_SPEED_R                     Measure left and right wheel speed
// PIN_VIN                                      Measure battery voltage via voltage divider
// PIN_TRIGGER                                  Arduino pin tied to trigger pin on ultrasonic sensor.
// PIN_ECHO                                     Arduino pin tied to echo pin on ultrasonic sensor.
// PIN_LED_LB, PIN_LED_RB                       Toggle left and right rear LEDs (indicator signals) 
//修改
//使用newping库，引脚不支持 PinChangeInterrupt 
//使用外部中断
//针对SFM车进行修改
//已做：1.spi屏幕
//待做：1.速度闭环 开环无法低速行驶 2.超声波小角度前面 距离不准确
//------------------------------------------------------//
// DEFINITIONS
//------------------------------------------------------//

// DO NOT CHANGE!
#define DIY 0
#define PCB_V1 1
#define PCB_V2 2

//------------------------------------------------------//
//SETTINGS
//------------------------------------------------------//

// Setup the OpenBot version (DIY,PCB_V1,PCB_V2)
#define OPENBOT DIY

// Enable/Disable voltage divider (1,0)
#define HAS_VOLTAGE_DIVIDER 1

// Enable/Disable indicators (1,0)
#define HAS_INDICATORS 0

// Enable/Disable speed sensors (1,0)
#define HAS_SPEED_SENSORS 1

// Enable/Disable sonar (1,0)
#define HAS_SONAR 1

// Enable/Disable median filter for sonar measurements (1,0)
#define USE_MEDIAN 1

// Enable/Disable OLED (1,0)
#define HAS_OLED 1

// Enable/Disable SPI OLED (1,0)新增SPI屏幕
#define HAS_SPI_OLED 1

// Enable/Disable no phone mode (1,0)
// In no phone mode:
// - the motors will turn at 50% speed
// - the speed will be reduced if an obstacle is detected by the sonar sensor
// - the car will turn, if an obstacle is detected within STOP_THRESHOLD
// WARNING: If the sonar sensor is not setup, the car will go full speed forward!
#define NO_PHONE_MODE 0

//------------------------------------------------------//
// PINOUT
//------------------------------------------------------//

//Setup the pin definitions
#if (OPENBOT == DIY)
  #define PIN_PWM_L1 5
  #define PIN_PWM_L2 6
  #define PIN_PWM_R1 7
  #define PIN_PWM_R2 8
  
  //四驱独立电机后轮
  #define PIN_PWM_L1_REAR 11
  #define PIN_PWM_L2_REAR 12
  #define PIN_PWM_R1_REAR 46
  #define PIN_PWM_R2_REAR 44

  #define PIN_SPEED_L 2//左轮测速
  #define PIN_SPEED_R 3//右轮测速
  #define PIN_VIN A0//测电压
  #define PIN_TRIGGER A11//测距离
  #define PIN_ECHO A12//测距离
  #define PIN_LED_LB 23//指示灯
  #define PIN_LED_RB 25//指示灯
  // SPI OLED
  
#elif (OPENBOT == PCB_V1)
  #define PIN_PWM_L1 9
  #define PIN_PWM_L2 10
  #define PIN_PWM_R1 5
  #define PIN_PWM_R2 6
  #define PIN_SPEED_L 2
  #define PIN_SPEED_R 4
  #define PIN_VIN A7
  #define PIN_TRIGGER 3
  #define PIN_ECHO 3
  #define PIN_LED_LB 7
  #define PIN_LED_RB 8
#elif (OPENBOT == PCB_V2)
  #define PIN_PWM_L1 9
  #define PIN_PWM_L2 10
  #define PIN_PWM_R1 5
  #define PIN_PWM_R2 6
  #define PIN_SPEED_L 2
  #define PIN_SPEED_R 3
  #define PIN_VIN A7
  #define PIN_TRIGGER 4
  #define PIN_ECHO 4
  #define PIN_LED_LB 7
  #define PIN_LED_RB 8
#endif

//------------------------------------------------------//
// INITIALIZATION
//------------------------------------------------------//

#include <limits.h>//头文件决定了各种变量类型的各种属性。
const unsigned int STOP_THRESHOLD = 32; //单位cm 停止距离阈值
const unsigned int SPEED_THRESHOLD = 160; //单位cm 最大速度阈值 新添加
#if NO_PHONE_MODE
  int turn_direction = 0; // right
  const unsigned long TURN_DIRECTION_INTERVAL = 2000; // How frequently to change turn direction (ms).随机换向间隔
  unsigned long turn_direction_timeout = 0;   // After timeout (ms), random turn direction is updated.
#endif


#if HAS_SONAR
//Sonar sensor
  #include <NewPing.h>
  const unsigned int MAX_DISTANCE = 260; //cm
  NewPing sonar(PIN_TRIGGER, PIN_ECHO, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
  const unsigned int PING_INTERVAL = 100; // How frequently to send out a ping (ms).
  unsigned long ping_timeout = 0;   // After timeout (ms), distance is set to maximum.
  boolean ping_success = false;
  unsigned int distance = MAX_DISTANCE; //cm
  unsigned int distance_estimate = MAX_DISTANCE; //cm
  #if USE_MEDIAN
    const unsigned int distance_array_sz = 3;
    unsigned int distance_array[distance_array_sz]={};
    unsigned int distance_counter = 0;
  #endif
#else
  unsigned int distance_estimate = UINT_MAX; //cm
#endif

#if HAS_OLED
  #include <SPI.h>
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#if HAS_SPI_OLED
  #define OLED_MOSI   26
  #define OLED_CLK   28
  #define OLED_DC    22
  #define OLED_CS    0 //虚拟
  #define OLED_RESET 24
   // OLED Display SSD1306 SPI OLED
  const unsigned int SCREEN_WIDTH = 128; // OLED display width, in pixels
  const unsigned int SCREEN_HEIGHT = 64; // OLED display height, in pixels
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#else  
  const int OLED_RESET = -1; // not used
  Adafruit_SSD1306 display(OLED_RESET);
  
  // OLED Display SSD1306 IIC OLED
  const unsigned int SCREEN_WIDTH = 128; // OLED display width, in pixels
  const unsigned int SCREEN_HEIGHT = 32; // OLED display height, in pixels
#endif

#endif

#if HAS_VOLTAGE_DIVIDER
  const unsigned int ADC_MAX = 1023;
  const unsigned int VREF = 5;
  //The voltage divider factor is computed as (R1+R2)/R2
  #if (OPENBOT == PCB_V1)
    const float VOLTAGE_DIVIDER_FACTOR = (100+33)/33;
  #else //DIY and PCB_V2
    const float VOLTAGE_DIVIDER_FACTOR = (10+1)/1;
  #endif
#endif

//Vehicle Control
int ctrl_left = 0;
int ctrl_right = 0;

//Voltage measurement
const unsigned int VIN_ARR_SZ = 10;
unsigned int vin_counter = 0;
unsigned int vin_array[VIN_ARR_SZ]={0};

//Speed sensor
const unsigned int DISK_HOLES = 390;
volatile int counter_left = 0;
volatile int counter_right = 0;

//Indicator Signal
const unsigned long INDICATOR_INTERVAL = 500; //Blinking rate of the indicator signal (ms).
unsigned long indicator_timeout = 0;
int indicator_val = 0;

//Serial communication
const unsigned long SEND_INTERVAL = 500; // How frequently vehicle data is sent (ms).
unsigned long send_timeout = 0;
String inString = "";

//------------------------------------------------------//
// SETUP
//------------------------------------------------------//

void setup()
{
  //Outputs
  pinMode(PIN_PWM_L1,OUTPUT);
  pinMode(PIN_PWM_L2,OUTPUT);
  pinMode(PIN_PWM_R1,OUTPUT);
  pinMode(PIN_PWM_R2,OUTPUT);
  pinMode(PIN_PWM_L1_REAR,OUTPUT);
  pinMode(PIN_PWM_L2_REAR,OUTPUT);
  pinMode(PIN_PWM_R1_REAR,OUTPUT);
  pinMode(PIN_PWM_R2_REAR,OUTPUT);
  pinMode(PIN_LED_LB,OUTPUT);
  pinMode(PIN_LED_RB,OUTPUT);

  //Inputs
  pinMode(PIN_VIN,INPUT);       
  pinMode(PIN_SPEED_L,INPUT);
  pinMode(PIN_SPEED_R,INPUT);
  
  Serial.begin(115200,SERIAL_8N1); //8 data bits, no parity, 1 stop bit
  send_timeout = millis() + SEND_INTERVAL; //wait for one interval to get readings

  //直接使用外部中断
  #if HAS_SPEED_SENSORS
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_L), update_speed_left, CHANGE);//双倍
    attachInterrupt(digitalPinToInterrupt(PIN_SPEED_R), update_speed_right, CHANGE);//双倍
  #endif
  
  //Initialize with the I2C addr 0x3C
  #if HAS_OLED
   #if HAS_SPI_OLED
  
    display.begin(SSD1306_SWITCHCAPVCC);
  #else
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  #endif
   display.display();
  delay(2000); // Pause for 2 seconds

  #endif
  
  //Test sequence for indicator LEDs
  #if HAS_INDICATORS
    digitalWrite(PIN_LED_LB,LOW);
    digitalWrite(PIN_LED_RB,LOW);
    delay(500);
    digitalWrite(PIN_LED_LB,HIGH);
    delay(500);
    digitalWrite(PIN_LED_LB,LOW);
    digitalWrite(PIN_LED_RB,HIGH);
    delay(500);
    digitalWrite(PIN_LED_RB,LOW);
  #endif
}

//------------------------------------------------------//
// MAIN LOOP
//------------------------------------------------------//

void loop() {
  
  #if HAS_VOLTAGE_DIVIDER
    //Measure voltage
    vin_array[vin_counter%VIN_ARR_SZ] = analogRead(PIN_VIN);
    vin_counter++;
  #endif
  
  #if HAS_SONAR
    //Measure distance every PING_INTERVAL
    if (millis() >= ping_timeout) {
      if (!ping_success) { // Check if last ping was returned
        distance = MAX_DISTANCE;
              }
      #if USE_MEDIAN
        distance_array[distance_counter%distance_array_sz] = distance;
        distance_counter++;
        distance_estimate = get_median(distance_array, distance_array_sz);
      #else
        distance_estimate = distance;
      #endif
      send_ping();
    }
  #endif
  
  // Send vehicle measurments to serial every SEND_INTERVAL
  if (millis() >= send_timeout) {
    send_vehicle_data();
    send_timeout = millis() + SEND_INTERVAL;
  }

  #if HAS_INDICATORS
    // Check indicator signal every INDICATOR_INTERVAL
    if (millis() >= indicator_timeout) {
      update_indicators();
      indicator_timeout = millis() + INDICATOR_INTERVAL;
    }
  #endif
  
  #if (NO_PHONE_MODE)
    if (millis() > turn_direction_timeout)
    {
      turn_direction_timeout = millis() + TURN_DIRECTION_INTERVAL;
      turn_direction = random(2); //Generate random number in the range [0,1]
    }
    // drive forward 128
    if (distance_estimate > 4*STOP_THRESHOLD) {
      ctrl_left = distance_estimate;
      ctrl_right = ctrl_left;
      digitalWrite(PIN_LED_LB, LOW);
      digitalWrite(PIN_LED_RB, LOW);
    }
    // turn slightly
    else if (distance_estimate > 2*STOP_THRESHOLD) {
      ctrl_left = 96;
      ctrl_right = 0;
    }
    // turn strongly 修改为128
    else if (distance_estimate > STOP_THRESHOLD) {
      ctrl_left = 128;
      ctrl_right = - 128;
    }
    // drive backward slowly
    else {
        ctrl_left = -64;
        ctrl_right = -64;
        digitalWrite(PIN_LED_LB, HIGH);
        digitalWrite(PIN_LED_RB, HIGH);
    }
    // flip controls if needed and set indicator light
    if (ctrl_left != ctrl_right) {
      if (turn_direction > 0) {
        int temp = ctrl_left;
        ctrl_left = ctrl_right;
        ctrl_right = temp;
        digitalWrite(PIN_LED_LB, HIGH);
        digitalWrite(PIN_LED_RB, LOW);
      }
      else {
        digitalWrite(PIN_LED_LB, LOW);
        digitalWrite(PIN_LED_RB, HIGH);
      }
    }
    // enforce limits
    ctrl_left = ctrl_left > 0 ? max(64, min(ctrl_left, SPEED_THRESHOLD)) : min(-64, max(ctrl_left, -SPEED_THRESHOLD));
    ctrl_right = ctrl_right > 0 ? max(64, min(ctrl_right, SPEED_THRESHOLD)) : min(-64, max(ctrl_right, -SPEED_THRESHOLD));

  #else // Wait for messages from the phone
    if (Serial.available() > 0) {
      read_msg();
    }
    if (distance_estimate < STOP_THRESHOLD) {
      if (ctrl_left > 0) ctrl_left = 0;
      if (ctrl_right > 0) ctrl_right = 0;
    }
  #endif
  
  update_left_motors();
  update_right_motors();
}


//------------------------------------------------------//
// FUNCTIONS
//------------------------------------------------------//

void update_left_motors() {
    if (ctrl_left < 0) {
      analogWrite(PIN_PWM_L1,-ctrl_left);
      analogWrite(PIN_PWM_L2,0);
      //添加后轮电机控制
      analogWrite(PIN_PWM_L1_REAR,-ctrl_left);
      analogWrite(PIN_PWM_L2_REAR,0);
    }
    else if (ctrl_left > 0) {
      analogWrite(PIN_PWM_L1,0);
      analogWrite(PIN_PWM_L2,ctrl_left);
      //添加后轮电机控制
       analogWrite(PIN_PWM_L1_REAR,0);
      analogWrite(PIN_PWM_L2_REAR,ctrl_left);
    }
    else { //Motor brake
      analogWrite(PIN_PWM_L1,255);
      analogWrite(PIN_PWM_L2,255);
      //添加后轮电机控制
      analogWrite(PIN_PWM_L1_REAR,255);
      analogWrite(PIN_PWM_L2_REAR,255);
    }
}

void update_right_motors() {
    if (ctrl_right < 0) {
      analogWrite(PIN_PWM_R1,-ctrl_right);
      analogWrite(PIN_PWM_R2,0);
      //添加后轮电机控制
      analogWrite(PIN_PWM_R1_REAR,-ctrl_right);
      analogWrite(PIN_PWM_R2_REAR,0);
    }
    else if (ctrl_right > 0) {
      analogWrite(PIN_PWM_R1,0);
      analogWrite(PIN_PWM_R2,ctrl_right);
      //添加后轮电机控制
      analogWrite(PIN_PWM_R1_REAR,0);
      analogWrite(PIN_PWM_R2_REAR,ctrl_right);
    }
    else { //Motor brake
      analogWrite(PIN_PWM_R1,255);
      analogWrite(PIN_PWM_R2,255);
      //添加后轮电机控制
      analogWrite(PIN_PWM_R1_REAR,255);
      analogWrite(PIN_PWM_R2_REAR,255);
    }
}

void read_msg() {
  if (Serial.available()) {
    char inChar = Serial.read();
    switch (inChar) {
      case 'c':
        ctrl_left = Serial.readStringUntil(',').toInt();
        ctrl_right = Serial.readStringUntil('\n').toInt();
        break;
      case 'i':
        indicator_val = Serial.readStringUntil('\n').toInt();
        break;
      default:
        break;
    }
  }
}

void send_vehicle_data() {
  float voltage_value = get_voltage();
  int ticks_left = counter_left;
  counter_left = 0;
  int ticks_right = counter_right;
  counter_right = 0;
  
  #if (NO_PHONE_MODE || HAS_OLED)
    float rpm_factor = 60.0*(1000.0/SEND_INTERVAL)/(DISK_HOLES);
    float rpm_left = ticks_left*rpm_factor;
    float rpm_right = ticks_right*rpm_factor;
  #endif
  #if (NO_PHONE_MODE)
    Serial.print("Voltage: "); Serial.println(voltage_value, 2);
    Serial.print("Left RPM: "); Serial.println(rpm_left, 0);
    Serial.print("Right RPM: "); Serial.println(rpm_right, 0);
    Serial.print("Distance: "); Serial.println(distance_estimate);
    Serial.println("------------------");
  #else
    Serial.print(voltage_value);
    Serial.print(",");
    Serial.print(ticks_left);
    Serial.print(",");
    Serial.print(ticks_right);
    Serial.print(",");
    Serial.print(distance_estimate);
    Serial.println();
  #endif 
  
  #if HAS_OLED
    // Set display information
    drawString(
      "Voltage:    " + String(voltage_value,2), 
      "Left RPM:  " + String(rpm_left,0), 
      "Right RPM: " + String(rpm_right, 0), 
      "Distance:   " + String(distance_estimate));
  #endif
}

#if HAS_VOLTAGE_DIVIDER
  float get_voltage () {
    unsigned long array_sum = 0;
    unsigned int array_size = min(VIN_ARR_SZ,vin_counter);
    for(unsigned int index = 0; index < array_size; index++)
    { 
      array_sum += vin_array[index]; 
    }
    return float(array_sum)/array_size/ADC_MAX*VREF*VOLTAGE_DIVIDER_FACTOR;
  }
#else
  float get_voltage () {
    return -1;
  }
#endif

#if HAS_INDICATORS
  void update_indicators() {
    if (indicator_val < 0) {
      digitalWrite(PIN_LED_LB, !digitalRead(PIN_LED_LB));
      digitalWrite(PIN_LED_RB, 0);
    }
    else if (indicator_val > 0) {
      digitalWrite(PIN_LED_LB, 0);
      digitalWrite(PIN_LED_RB, !digitalRead(PIN_LED_RB));
    }
    else {
      digitalWrite(PIN_LED_LB, 0);
      digitalWrite(PIN_LED_RB, 0);
    }
  }
#endif

#if HAS_OLED
// Function for drawing a string on the OLED display
#if HAS_SPI_OLED
void drawString(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  // set text color
  display.setTextColor(WHITE);
  // set text size
  display.setTextSize(1);
  // set text cursor position
  display.setCursor(1,0);
  // show text
  display.println(line1);
  display.setCursor(1,8);
  // show text
  display.println(line2);
  display.setCursor(1,16);
  // show text
  display.println(line3);
  display.setCursor(1,24);
  // show text
  display.println(line4);    
  display.display();
}
#else
void drawString(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  // set text color
  display.setTextColor(WHITE);
  // set text size
  display.setTextSize(1);
  // set text cursor position
  display.setCursor(1,0);
  // show text
  display.println(line1);
  display.setCursor(1,8);
  // show text
  display.println(line2);
  display.setCursor(1,16);
  // show text
  display.println(line3);
  display.setCursor(1,24);
  // show text
  display.println(line4);    
  display.display();
}
#endif
#endif
#if USE_MEDIAN
  unsigned int get_median(unsigned int a[], unsigned int sz) {
    //bubble sort
    for(unsigned int i=0; i<(sz-1); i++) {
      for(unsigned int j=0; j<(sz-(i+1)); j++) {
        if(a[j] > a[j+1]) {
            unsigned int t = a[j];
            a[j] = a[j+1];
            a[j+1] = t;
        }
      }
    }
    return a[sz/2];
  }
#endif

//------------------------------------------------------//
// INTERRUPT SERVICE ROUTINES (ISR)
//------------------------------------------------------//

#if HAS_SPEED_SENSORS
  // ISR: Increment speed sensor counter (right)
  void update_speed_left() {
    if (ctrl_left < 0) {
      counter_left--; 
    }
    else if (ctrl_left > 0) {
      counter_left++;
    }
  }
  
  // ISR: Increment speed sensor counter (right)
  void update_speed_right() {
    if (ctrl_right < 0) {
      counter_right--; 
    }
    else if (ctrl_right > 0){
      counter_right++;
    }
  }
#endif

#if HAS_SONAR
  // Send pulse by toggling trigger pin
  void send_ping() {
    ping_success = false;
    ping_timeout = millis() + PING_INTERVAL; // Set next ping time.
    sonar.ping_timer(echo_check); // Send out the ping, calls "echo_check" function every 24uS where you can check the ping status.
     }
void echo_check() { // Timer2 interrupt calls this function every 24uS.
    if (sonar.check_timer()) { // Check ping status
      distance = sonar.ping_result / US_ROUNDTRIP_CM; // Ping returned in uS, convert to cm
      ping_success = true;
    }
}
#endif

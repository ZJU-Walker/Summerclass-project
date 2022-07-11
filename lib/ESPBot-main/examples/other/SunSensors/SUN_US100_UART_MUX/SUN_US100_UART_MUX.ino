//超声波多路串口复用测试
#include "SunConfig.h"
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
unsigned int Len_mm  = 0;
//SUNUARTMUX(uint8_t INHPin, uint8_t APin, uint8_t BPin);
SUNUARTMUX uartMux(30,31,32);
void setup()
{   //connect RX (Pin 0 of Arduino digital IO) to Echo/Rx (US-100), TX (Pin 1 of Arduino digital IO) to Trig/Tx (US-100) 
    Serial3.begin(9600);  //set baudrate as 9600bps.
    Serial.begin(115200);  //set baudrate as 9600bps.
}

void loop()
{
    Serial3.flush();     // clear receive buffer of serial port
    uartMux.enableUART(2);
    Serial3.write(0X55); // trig US-100 begin to measure the distance
    delay(500);          //delay 500ms to wait result
    if(Serial3.available() >= 2)                    //when receive 2 bytes 
    {
        HighLen = Serial3.read();                   //High byte of distance
        LowLen  = Serial3.read();                   //Low byte of distance
        Len_mm  = HighLen*256 + LowLen;             //Calculate the distance
        if((Len_mm > 1) && (Len_mm < 10000))        //normal distance should between 1mm and 10000mm (1mm, 10m)
        {
            Serial.print("Present Length is: ");   //output the result to serial monitor
            Serial.print(Len_mm, DEC);             //output the result to serial monitor
            Serial.println("mm");                  //output the result to serial monitor
        }
    }
    delay(500);                                     //wait 500ms
}

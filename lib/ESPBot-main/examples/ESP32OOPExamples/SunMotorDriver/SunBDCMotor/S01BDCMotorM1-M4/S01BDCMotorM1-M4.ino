/*
 * This example uses the SunBDCMotor library to drive each motor with the
 *  AS4950 4 channel Motor Driver Board for Arduino forward, then backward. 
 * 每个轮子轮流正转反转，请检查转向是否正确，修改flipMotors
 */

#include "OOPConfig.h"

BDCMotor motors;

void setup()
{
  motors.init();
  motors.flipMotors(false,false,false,false);//根据实际转向进行调整false or true 黑色PCB电机 false  绿色PCB电机true
  Serial.begin(BAUDRATE);
  Serial.println("AS4950 Quad Motor Driver Board for Arduino");
}

void loop()
{
  // run M1 motor with positive speed

  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  // run M1 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM1Speed(speed);
    delay(2);
  }

  // run M2 motor with positive speed



  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  // run M2 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM2Speed(speed);
    delay(2);
  }

  // run M3 motor with positive speed



  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  // run M3 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM3Speed(speed);
    delay(2);
  }

  // run M4 motor with positive speed


  for (int speed = 0; speed <= 255; speed++)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  for (int speed = 255; speed >= 0; speed--)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  // run M4 motor with negative speed



  for (int speed = 0; speed >= -255; speed--)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  for (int speed = -255; speed <= 0; speed++)
  {
    motors.setM4Speed(speed);
    delay(2);
  }

  delay(500);
}
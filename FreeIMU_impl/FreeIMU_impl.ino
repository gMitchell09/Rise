#define ARDUIMU_v3

#include "calibration.h"
#include <CommunicationUtils.h>
#include <FreeIMU.h>
#include <vector_math.h>

#include <helper_3dmath.h>

#include <HMC58X3.h>
#include <ITG3200.h>
#include <MS561101BA.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <EEPROM.h>
#include <Wire.h>

//#define DEBUG
#include "DebugUtils.h"
#include <SPI.h>

//#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

//SoftwareSerial serial(10, 11);
FreeIMU IMU;
bool outEnable = false;

int send_binary (void* arg, unsigned int len);
void send_timestamp();

void setup()
{
  IMU.init(true);
  Wire.begin();
  
  if (Serial)
  {
    Serial.begin(115200);
  }
}

void loop()
{
  float quat[4];
  if (Serial)
  {
    if (outEnable)
    {
      IMU.getQ(quat);
      Serial.print('S');
      unsigned long timestamp = millis();
      send_binary(&timestamp, sizeof(unsigned long));
      Serial.print('T');
      Serial.print('q');
      Serial.print('L');
      byte len = 4 * sizeof(float);
      send_binary(&len, sizeof(unsigned byte));
      Serial.print('D');
      send_binary(quat, sizeof(float));
      send_binary(quat + 1, sizeof(float));
      send_binary(quat + 2, sizeof(float));
      send_binary(quat + 3, sizeof(float));
      Serial.print('E');
      delay(10);
    }
    
    while (Serial.available() > 0)
    {
      char inByte = Serial.read();
      switch (inByte)
      {
        case '1':
          outEnable = true;
          break;
        case '0':
          outEnable = false;
          break;
        case 'T':
          send_timestamp();
          break;
        default:
          break;
      }
    }
  }
}

void send_timestamp()
{
  Serial.print('S');
  unsigned long timestamp = millis();
  send_binary(&timestamp, sizeof(unsigned long));
  Serial.print('T');
  Serial.print('t');
  Serial.print('L');
  byte len = sizeof(float);
  send_binary(&len, sizeof(unsigned byte));
  Serial.print('D');
  float f = 0.0f;
  send_binary(&f, sizeof(float));
  Serial.print('E');
}

int send_binary (void* arg, unsigned int len)
{
  byte* data = (byte*)arg;
  return Serial.write(data, len);
}

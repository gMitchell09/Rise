#define ARDUIMU_v3

//#include <calibration.h>
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
#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <SPI.h>

//#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE

//SoftwareSerial serial(10, 11);
FreeIMU IMU;
bool outEnable = false;

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
      Serial.print('~');
      Serial.print(millis());
      Serial.print('D');
      send_float(quat[0]);
      send_float(quat[1]);
      send_float(quat[2]);
      send_float(quat[3]);
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
        case 'T':
          Serial.print('T');
          Serial.print(millis());
          Serial.print('E');
          break;
        default:
          break;
      }
    }
  }
}

int send_float(float arg)
{
  byte * data = (byte*) &arg;
  return Serial.write(data, sizeof(arg));
}

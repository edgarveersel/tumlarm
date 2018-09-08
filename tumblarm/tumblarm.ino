 /* Experimental ESP8266 piece of code
  *
  *  Start Date: 2018-09-07
  *  Coding: Edgar, Nick
  *
  *  Version: 0.0.1
  *
  */
//****************************************
//        INCLUDES
//****************************************
#include "Wire.h"
// uncomment the following to disable serial DEBUG statements
//#define SERIAL_DEBUG false
#include <SerialDebug.h>


//****************************************
//        CONSTANTS / CONFIGURATION
//****************************************
//SYSTEM SETTINGS
const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050

//USER SETTINGS
const unsigned long culBeepTime = 1000; //millis of beep duration
const unsigned long culCheckGyro = 300; //millis to check the Gyro position
const int16_t AcMargin = 300; //all dimensions margin for the Accellerator
const int16_t GyMargin = 300; //all dimensions margin for the Gyro

#define AlarmTriggerPin D8


//****************************************
//        STRUCTURES
//****************************************
struct rawdata {
  int16_t AcX;
  int16_t AcY;
  int16_t AcZ;
  int16_t Tmp;
  int16_t GyX;
  int16_t GyY;
  int16_t GyZ;
};
//
//struct scaleddata{
//float AcX;
//float AcY;
//float AcZ;
//float Tmp;
//float GyX;
//float GyY;
//float GyZ;
//};


//****************************************
//        GLOBAL VARIABLES
//****************************************
rawdata rdRestState;
unsigned long ulLastGyroCheck = 0;
unsigned long ulLastBeepStart = 0;
bool fAlarmTriggered = false;


//****************************************
//        FUNCTON DECLARATIONS
//****************************************
bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr);
bool CheckAccMovementOutOfBound(rawdata State);
bool CheckGyroMovementOutOfBound(rawdata State);
void SetAlarmTrigger (bool);

//****************************************
//        SETUP
//****************************************
void setup() {
  Wire.begin();

  SERIAL_DEBUG_SETUP(9600);
  DEBUG("Let the gyro stabilize for 3 sec...");

  mpu6050Begin(MPU_addr);
  delay(3000);

  rdRestState =  mpu6050Read(MPU_addr);
  DEBUG("");
  DEBUG("Gyro values stored");

  analogWriteRange(1000);

  analogWrite(AlarmTriggerPin, 512);
  delay(1000);
  analogWrite(AlarmTriggerPin, 0);
  pinMode(AlarmTriggerPin, OUTPUT);
  digitalWrite(AlarmTriggerPin, LOW);
  delay(1000);
}



//****************************************
//        LOOP
//****************************************
void loop() {

   if ((ulLastGyroCheck + culCheckGyro <= millis()) && !fAlarmTriggered){
      rawdata rdNewState = mpu6050Read(MPU_addr);

      bool fAccOutOfBound = CheckAccMovementOutOfBound(rdNewState);
      bool fGyroOutOfBound = CheckGyroMovementOutOfBound(rdNewState);

      if (fAccOutOfBound || fGyroOutOfBound)
          SetAlarmTrigger(true);

      ulLastGyroCheck = millis();
   }

   if (ulLastBeepStart + culBeepTime <= millis())
      SetAlarmTrigger(false);

}


//****************************************
//        FUNCTIONS
//****************************************

void mpu6050Begin(byte addr){
  // This function initializes the MPU-6050 IMU Sensor
  // It verifys the address is correct and wakes up the
  // MPU.
  if (checkI2c(addr)){
    Wire.beginTransmission(addr);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);

  delay(30); // Ensure gyro has enough time to power up
  }
}

bool checkI2c(byte addr){
  // We are using the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  DEBUG(" ");
  Wire.beginTransmission(addr);

  if (Wire.endTransmission() == 0)
  {
    return true;
  }
  else
  {
    DEBUG("No Device Found at 0x");
    DEBUG("");
    DEBUG(addr,HEX);
    return false;
  }
}


rawdata mpu6050Read(byte addr){
  // This function reads the raw 16-bit data values from
  // the MPU-6050

  rawdata State;

  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  State.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  State.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  State.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  State.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  State.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  State.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  State.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)


  DEBUG(State.AcX);
  DEBUG(";");
  DEBUG(rdRestState.AcX);
  DEBUG(";");
  DEBUG(abs(State.AcX - rdRestState.AcX));
//      DEBUG("Values:"); DEBUG(values.GyX);
//      DEBUG(";"); DEBUG(values.GyY);
//    DEBUG(";");  DEBUG(values.GyZ);
//    DEBUG(";");  DEBUG(values.Tmp);
//    DEBUG(";");  DEBUG(values.AcX);
//    DEBUG(";");  DEBUG(values.AcY);
//    DEBUG(";");  DEBUG(values.AcZ);
//    DEBUG("DELTA:"); DEBUG(values.GyX - rdRestState.GyX);
//    DEBUG(";");  DEBUG(values.GyY - rdRestState.GyY);
//    DEBUG(";");  DEBUG(values.GyZ - rdRestState.GyZ);
//    DEBUG(";");  DEBUG(values.Tmp - rdRestState.Tmp);
//    DEBUG(";");  DEBUG(values.AcX - rdRestState.AcX);
//    DEBUG(";");  DEBUG(values.AcY - rdRestState.AcY);
//    DEBUG(";");  DEBUG(values.AcZ - rdRestState.AcZ);
  return State;
};


bool CheckAccMovementOutOfBound(rawdata State){

  bool fX = abs(State.AcX - rdRestState.AcX) > AcMargin;
  bool fY = abs(State.AcY - rdRestState.AcY) > AcMargin;
  bool fZ = abs(State.AcZ - rdRestState.AcZ) > AcMargin;

  return (fX || fY || fZ);

};

bool CheckGyroMovementOutOfBound(rawdata State){

  bool fX = abs(State.GyX - rdRestState.GyX) > AcMargin;
  bool fY = abs(State.GyY - rdRestState.GyY) > AcMargin;
  bool fZ = abs(State.GyZ - rdRestState.GyZ) > AcMargin;

  return (fX || fY || fZ);

};

void SetAlarmTrigger (bool fArm){

  if (fArm){  //switch ON AlarmTriggerPin
    analogWriteRange(1000);
    analogWrite(AlarmTriggerPin, 512);
    ulLastBeepStart = millis();
    fAlarmTriggered = true;
  }
  else
  { //switch OFF AlarmTriggerPin
    analogWrite(AlarmTriggerPin, 0);
    pinMode(AlarmTriggerPin, OUTPUT);
    digitalWrite(AlarmTriggerPin, LOW);
    fAlarmTriggered = false;
  }

};

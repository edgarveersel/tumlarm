 /* Experimental ESP8266 piece of code
  *
  *  Start Date: 2018-09-07
  *  Coding: Edgar, Nick
  *
  *  Version: 0.0.1
  *
  * Capacitor issue: http://forum.arduino.cc/index.php?topic=394691.0
  *
  */
//****************************************
//        INCLUDES
//****************************************
#include "Wire.h"
// uncomment the following to disable serial DEBUG statements
//#define SERIAL_DEBUG false
#include <SerialDebug.h>
#include "MPU6050.h"


//****************************************
//        CONSTANTS / CONFIGURATION
//****************************************
//SYSTEM SETTINGS
const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050
#define AlarmTriggerPin D8

//USER SETTINGS
const unsigned long culAlarmDuration = 1000; //millis of beep duration
const unsigned long culCheckGyro = 300; //millis to check the Gyro position
const float AcMargin = 0.04; //all dimensions margin for the Accellerator
const float GyMargin = 0.04; //all dimensions margin for the Gyro


//****************************************
//        STRUCTURES
//****************************************
struct rawdata {
  int16_t iAcX;
  int16_t iAcY;
  int16_t iAcZ;
  int16_t iTmp;
  int16_t iGyX;
  int16_t iGyY;
  int16_t iGyZ;
};

struct scaleddata{
  float AcX;
  float AcY;
  float AcZ;
  float Tmp;
  float GyX;
  float GyY;
  float GyZ;
};

//****************************************
//        GLOBAL VARIABLES
//****************************************
scaleddata RestState;
unsigned long ulLastGyroCheck = 0;
unsigned long ulAlarmEndMillis = 0;
bool fAlarmTriggered = false;


//****************************************
//        FUNCTON DECLARATIONS
//****************************************
bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
scaleddata mpu6050Read(byte addr);
bool CheckAccMovementOutOfBound(scaleddata State, float Xmargin, float Ymargin, float Zmargin);
bool CheckGyroMovementOutOfBound(scaleddata State, float Xmargin, float Ymargin, float Zmargin);
void SetAlarmTrigger (bool fArm);

//****************************************
//        SETUP
//****************************************
void setup() {
  Wire.begin();

  SERIAL_DEBUG_SETUP(9600);
  DEBUG(millis(), "Let the gyro stabilize for 3 sec...");

  mpu6050Begin(MPU_addr);
  delay(3000);

  RestState =  mpu6050Read(MPU_addr);
  DEBUG(millis(), "Gyro values stored", RestState.AcX, RestState.AcY, RestState.AcZ);

  analogWriteRange(1000);

  //two beeps
  SetAlarmTrigger (true, 500);
  delay(500);
  SetAlarmTrigger (false, 500);
  delay(500);
  SetAlarmTrigger (true, 500);
  delay(500);
  SetAlarmTrigger (false, 500);
  delay(1000);

}



//****************************************
//        LOOP
//****************************************
void loop() {
  rawdata newrawstate;

  if ((ulLastGyroCheck + culCheckGyro <= millis()) && !fAlarmTriggered){
      scaleddata rdNewState = mpu6050Read(MPU_addr);
//      DEBUG ("Measured", rdNewState.AcX, rdNewState.AcY, rdNewState.AcZ);

      bool fAccOutOfBound = CheckAccMovementOutOfBound(rdNewState, AcMargin, AcMargin, AcMargin);
//      bool fGyroOutOfBound = CheckGyroMovementOutOfBound(rdNewState, GyMargin, GyMargin, GyMargin);

//      if (fAccOutOfBound || fGyroOutOfBound)
      if (fAccOutOfBound)
          SetAlarmTrigger(true, culAlarmDuration);

      ulLastGyroCheck = millis();
   }

   if (fAlarmTriggered && culAlarmDuration <= millis())
      SetAlarmTrigger(false, 0);



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
  Wire.beginTransmission(addr);

  if (Wire.endTransmission() == 0)
  {
    return true;
  }
  else
  {
    DEBUG(millis(), "No Device Found at 0x", addr);
    return false;
  }
}


  scaleddata mpu6050Read(byte addr){
  // This function reads the raw 16-bit data values from
  // the MPU-6050

  rawdata State;
  scaleddata ScaledState;

  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  State.iAcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  State.iAcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  State.iAcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  State.iTmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  State.iGyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  State.iGyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  State.iGyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  ScaledState.AcX = State.iAcX / 256.0;
  ScaledState.AcY = State.iAcY / 256.0;
  ScaledState.AcZ = State.iAcZ / 256.0;
  ScaledState.GyX = State.iGyX / 256.0;
  ScaledState.GyY = State.iGyY / 256.0;
  ScaledState.GyZ = State.iGyZ / 256.0;

  return ScaledState;
};


bool CheckAccMovementOutOfBound(scaleddata State, float Xmargin, float Ymargin, float Zmargin){
  //built for speed
  //Will be within bounds most of the times - inner cube: 0.5 * sqrt(2)
  bool fX = abs(State.AcX - RestState.AcX) <= 0.7 * Xmargin;
  bool fY = abs(State.AcY - RestState.AcY) <= 0.7 * Ymargin;
  bool fZ = abs(State.AcZ - RestState.AcZ) <= 0.7 * Zmargin;

  if (fX && fY && fZ){
    DEBUG (millis(), "Within inner cube", abs(State.AcX - RestState.AcX), abs(State.AcY - RestState.AcY), abs(State.AcZ - RestState.AcZ));
    return false; //within a cube that is completely inside the allowed sphere
  }
  else
  {
    ///Hmm, let's try outside of the full cube
    fX = abs(State.AcX - RestState.AcX) > Xmargin;
    fY = abs(State.AcY - RestState.AcY) > Ymargin;
    fZ = abs(State.AcZ - RestState.AcZ) > Zmargin;

    if (fX || fY || fZ){
      DEBUG (millis(), "Outside outer cube",
                  abs(State.AcX - RestState.AcX),
                  abs(State.AcY - RestState.AcY),
                  abs(State.AcZ - RestState.AcZ));
      return true;
    }
    else
    {//Dang, have to calculate the ball coordinates...
      float radius = sqrt(sq(State.AcX - RestState.AcX) + sq(State.AcY - RestState.AcY) + sq(State.AcZ - RestState.AcZ));

      DEBUG(millis(), "Raduis", radius,
                      abs(State.AcX - RestState.AcX),
                      abs(State.AcY - RestState.AcY),
                      abs(State.AcZ - RestState.AcZ),
                      radius <= AcMargin);

      return (radius <= AcMargin);
    }
  }
};

bool CheckGyroMovementOutOfBound(scaleddata State, float Xmargin, float Ymargin,float Zmargin){

  bool fX = abs(State.GyX - RestState.GyX) > Xmargin;
  bool fY = abs(State.GyY - RestState.GyY) > Ymargin;
  bool fZ = abs(State.GyZ - RestState.GyZ) > Zmargin;

  return (fX || fY || fZ);

};

void SetAlarmTrigger (bool fArm, unsigned long ulDuration){

  if (fArm){  //switch ON AlarmTriggerPin
    analogWriteRange(1000);
    analogWrite(AlarmTriggerPin, 512);
    ulAlarmEndMillis = millis() + ulDuration;
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

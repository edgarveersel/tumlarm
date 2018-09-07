#include "Wire.h"


//****************************************
//        CONSTANTS
//****************************************
const uint8_t MPU_addr=0x68; // I2C address of the MPU-6050
const unsigned long culBeepTime = 1000; //millis of beep duration
const unsigned long culCheckGyro = 300; //millis to check the Gyro position
const bool cbDebug = true;
const int16_t AcMargin = 300;

#define buzzer D8


//****************************************
//        SRUCTURES
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
//        VARIABLES
//****************************************
rawdata rdRestState;
unsigned long ulLastGyroCheck = 0;
unsigned long ulLastBeepStart = 0;
bool fBuzzing = false;


//****************************************
//        FUNCTON DECLARATIONS
//****************************************
bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);




//****************************************
//        SETUP
//****************************************
void setup() {
  Wire.begin();
  
  Serial.begin(115200);
  Serial.println("Let the gyro stabilize for 3 sec...");

  mpu6050Begin(MPU_addr);
  delay(3000);
  
  rdRestState =  mpu6050Read(MPU_addr, cbDebug);
  Serial.println("");
  Serial.println("Gyro values stored");

    analogWriteRange(1000);

    analogWrite(buzzer, 512);
    delay(1000);
    analogWrite(buzzer, 0);
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, LOW);
    delay(1000);

}



//****************************************
//        LOOP
//****************************************
void loop() {
  
 if ((ulLastGyroCheck + culCheckGyro <= millis()) && (!fBuzzing)){
    rawdata rdNewState;
    
    
    
    
    
    rdNewState = mpu6050Read(MPU_addr, cbDebug);

    if (abs(rdNewState.AcX - rdRestState.AcX) > AcMargin ){
        analogWriteRange(1000);
      analogWrite(buzzer, 512);
      ulLastBeepStart = millis();
      fBuzzing = true;
    }
  
    ulLastGyroCheck = millis();
 }

 if (ulLastBeepStart + culBeepTime <= millis()){
      analogWrite(buzzer, 0);
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, LOW);
          fBuzzing = false;
 }

 
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
  Serial.println(" ");
  Wire.beginTransmission(addr);
   
  if (Wire.endTransmission() == 0)
  {
    return true;
  }
  else
  {
    Serial.print("No Device Found at 0x");
    Serial.println(addr,HEX);
    return false;
  }
}


rawdata mpu6050Read(byte addr, bool Debug){
  // This function reads the raw 16-bit data values from
  // the MPU-6050
   
  rawdata values;
   
  Wire.beginTransmission(addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(addr,14,true); // request a total of 14 registers
  values.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  values.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  values.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  values.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  values.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  values.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  values.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
  if(Debug){
    
Serial.print(values.AcX);
Serial.print(";"); 
Serial.print(rdRestState.AcX);
Serial.print(";"); 
 Serial.println(abs(values.AcX - rdRestState.AcX));
//    Serial.print("Values:"); Serial.print(values.GyX);
//    Serial.print(";"); Serial.print(values.GyY);
//    Serial.print(";");  Serial.print(values.GyZ);
//    Serial.print(";");  Serial.print(values.Tmp);
//    Serial.print(";");  Serial.print(values.AcX);
//    Serial.print(";");  Serial.print(values.AcY);
//    Serial.print(";");  Serial.println(values.AcZ);
//    Serial.print("DELTA:"); Serial.print(values.GyX - rdRestState.GyX);
//    Serial.print(";");  Serial.print(values.GyY - rdRestState.GyY);
//    Serial.print(";");  Serial.print(values.GyZ - rdRestState.GyZ);
//    Serial.print(";");  Serial.print(values.Tmp - rdRestState.Tmp);
//    Serial.print(";");  Serial.print(values.AcX - rdRestState.AcX);
//    Serial.print(";");  Serial.print(values.AcY - rdRestState.AcY);
//    Serial.print(";");  Serial.println(values.AcZ - rdRestState.AcZ);
  }
 
  return values;
}

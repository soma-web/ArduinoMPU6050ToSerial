#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h> //library includes mathematical functions

const int MPU=0x68; //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ; //16-bit integers
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; //calibration variables
double pitch,roll;
int accel[3];
int gyro[3];

void setup()
{
    Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    Serial.begin(38400); //serial communication at 38400 bauds
}

void loop()
{


  
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU,14,true); //request 14 registers in total  

    //Acceleration data correction
    AcXcal = -950;
    AcYcal = -300;
    AcZcal = 0;

    //Temperature correction
    //tcal = -1600;

    //Gyro correction
    GyXcal = 480;
    GyYcal = 170;
    GyZcal = 210;


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
    
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);
  
    //printing values to serial port

    //Acceleration Values
    accel[0] = AcX + AcXcal;
    accel[1] = AcY + AcYcal;
    accel[2] = AcZ + AcZcal;

    //Gyro Values
    gyro[0] = GyX + GyXcal;
    gyro[1] = GyY + GyYcal;
    gyro[2] = GyZ + GyZcal;
    /*
    Serial.print("X = "); Serial.print(gyro[0]);
    Serial.print(" Y = "); Serial.print(gyro[1]);
    Serial.print(" Z = "); Serial.println(gyro[2]); 
    */
    StaticJsonBuffer<200>  jBuffer;
    JsonObject& root = jBuffer.createObject();
    
    JsonObject& jAngle = root.createNestedObject("Angle");
    jAngle["roll"] = roll;
    jAngle["pitch"] = pitch;
    /*
    JsonArray& jAngle = root.createNestedArray("Angle");
    jAngle.add(pitch);
    jAngle.add(roll);
    */
    JsonArray& jAccelerometer = root.createNestedArray("Accel");
    jAccelerometer.add(accel[0]);
    jAccelerometer.add(accel[1]);
    jAccelerometer.add(accel[2]);

    JsonArray& jGyro = root.createNestedArray("Gyro");
    jGyro.add(gyro[0]);
    jGyro.add(gyro[1]);
    jGyro.add(gyro[2]);
    
    root.printTo(Serial);
    Serial.println();

    
}

//function to convert accelerometer values into pitch and roll
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}

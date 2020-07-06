#include "complementary-filter.h"
#include <MPU9250_asukiaaa.h>
#include "FreeRTOS.h"

MPU9250_asukiaaa mySensor;
float accelX, accelY, accelZ, aSqrt, gyroX, gyroY, gyroZ, mDirection, mX, mY, mZ;

float X = 0;
float Y = 0;
float Z = 0;


void compensate(){
  accelZ = accelZ - 0.18;
  accelY = accelY + 0.09;
  accelX = accelX + 0.07;
  
  float tmp = accelX;
  accelX = accelY;
  accelY = -tmp;

  
  gyroX = gyroX + 1.2;
  gyroZ = gyroZ - 0.6;

  tmp = gyroX;
  gyroX = gyroY*-1;
  gyroY = tmp;
  gyroZ = gyroZ*-1;
}

void complementary_filter(){
  float h = 0.002;
  float alpha = 0.1;
  float gamma = alpha/(h+alpha);

  // There really should be a lock or something on these
  float gyrX = degToRad(gyroX)*h;
  float gyrY = degToRad(gyroY)*h;
  float gyrZ = degToRad(gyroZ)*h;

  // Add previous estimated angles to gyro estimated angle change and multiply with gamma
  gyrX = (gyrX + X) * gamma;
  gyrY = (gyrY + Y) * gamma;
  gyrZ = (gyrZ + Z) * 0.99; //Forgetting factor

  //Convert acc to rad
  float g = -9.81;
  float accX = accelX*g;
  float accY = accelY*g;
  float accZ = accelZ*g;
  

  float phi = atan2( accY, accZ);
  float theta = atan2( -accX, sqrt( pow(accY, 2) + pow(accZ, 2) ));
  
  phi = phi*(1-gamma);
  theta = theta*(1-gamma);

  // Add the estimations of the gyro and accelerometer
  X = phi + gyrX;
  Y = theta + gyrY;
  Z = gyrZ;
}

void init_mpu(){
    mySensor.beginAccel();
    mySensor.beginGyro();
    mySensor.beginMag();
}

void update_filter(){
    if (mySensor.accelUpdate() == 0 && mySensor.gyroUpdate() == 0) {
    accelX = mySensor.accelX();
    accelY = mySensor.accelY();
    accelZ = mySensor.accelZ();
    gyroX = mySensor.gyroX();
    gyroY = mySensor.gyroY();
    gyroZ = mySensor.gyroZ();
    compensate();

    complementary_filter();
  }
}

float get_X(){return X;}
float get_Y(){return Y;}
float get_Z(){return Z;}
#include "complementary-filter.h"
#include <MPU9250_asukiaaa.h>
#include "FreeRTOS.h"

MPU9250_asukiaaa mySensor;
xSemaphoreHandle wire_lock_filter;
float accelX, accelY, accelZ, aSqrt, gyroX, gyroY, gyroZ, mDirection, mX, mY, mZ;

float X = 0;
float Y = 0;
float Z = 0;

unsigned long last_update_filter;


void compensate(){
  accelZ = accelZ;
  accelY = accelY + 0.0039;
  accelX = accelX + 0.02;
  
  accelX = accelX;
  accelY = -accelY;
  accelZ = -accelZ;

  
  gyroX = gyroX - 0.3;
  gyroZ = gyroZ;

  gyroX = -gyroX*0;
  gyroY = gyroY*0;
  gyroZ = gyroZ*0;
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

void init_mpu(xSemaphoreHandle wire_lock){
  wire_lock_filter = wire_lock;
  last_update_filter = millis();
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
}

void update_filter(){

  // Check if it's time to update
  if(last_update_filter + 1.f / FILTER_UPDATE_HZ * 1000 > millis())
    return;
    
  last_update_filter = millis();

  // Make sure wire is available
  xSemaphoreTake(wire_lock_filter, portMAX_DELAY);

  // check if there's new data
  if (mySensor.accelUpdate() != 0 || mySensor.gyroUpdate() != 0){
    xSemaphoreGive(wire_lock_filter);
    return;
  }
  
  // It's time to update, wire is available and there is new data. Estimate orientation
  accelX = mySensor.accelX();
  accelY = mySensor.accelY();
  accelZ = mySensor.accelZ();
  gyroX = mySensor.gyroX();
  gyroY = mySensor.gyroY();
  gyroZ = mySensor.gyroZ();
  xSemaphoreGive(wire_lock_filter);

  // Take the readings and convert them into the wanted coordinate frame
  compensate();
  complementary_filter();
}

float get_X(){return X;}
float get_Y(){return Y;}
float get_Z(){return Z;}
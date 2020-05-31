#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setFullScaleGyroRange(0); //0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
  accelgyro.setFullScaleAccelRange(0);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 

/*
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  */
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  double ax_conv =  ax / 16384.0 * 9.81;
  double ay_conv =  ay / 16384.0 * 9.81;
  double az_conv =  az / 16384.0 * 9.81;

  


  Serial.print("Acc_x = ");
  Serial.print(ax);
  Serial.print("     Acc_y = ");
  Serial.print(ay);
  Serial.print("     Acc_z = ");
  Serial.println(az);

  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(250);
}
#include <Arduino.h>
#include <MPU9250_asukiaaa.h>
#include <Servo.h>

/*TODO:
  Control altitude?

*/

Servo ESC;

#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

MPU9250_asukiaaa mySensor;
float accelX, accelY, accelZ, aSqrt, gyroX, gyroY, gyroZ, mDirection, mX, mY, mZ;

float X = 0;
float Y = 0;
float Z = 0;
void compensate();
void complementary_filter();

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
  ESC.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(SDA_PIN, SCL_PIN);
  mySensor.setWire(&Wire);
#endif

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
}

void loop() {
  if (mySensor.accelUpdate() == 0 && mySensor.gyroUpdate() == 0) {
    accelX = mySensor.accelX();
    accelY = mySensor.accelY();
    accelZ = mySensor.accelZ();
    gyroX = mySensor.gyroX();
    gyroY = mySensor.gyroY();
    gyroZ = mySensor.gyroZ();
    compensate();
    //Serial.print("accel: " + String(accelX) + " " + String(accelY) + " " + String(accelZ));
    //Serial.println(" gyro: " + String(gyroX) + " " + String(gyroY) + " " + String(gyroZ));

    Serial.print("X :");
    Serial.print(radToDeg(X));
    Serial.print("  Y :");
    Serial.print(radToDeg(Y));
    Serial.print("  Z :");
    Serial.println(radToDeg(Z));
    complementary_filter();
    delay(20);
  }

  /*for (int i = 0; i < 1024; i+=100){ 
    int val = map(i, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(val);    // Send the signal to the ESC
    Serial.println(i);
    delay(500);
  }*/


}

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
  float h = 0.025;
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
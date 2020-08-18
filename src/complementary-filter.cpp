#include "complementary-filter.h"
#include "FreeRTOS.h"
#include "controller.h"
#include <MPU9250_asukiaaa.h>

#define g -9.81

MPU9250_asukiaaa mySensor;
xSemaphoreHandle wire_lock_filter, XYZ_lock;
float accelX, accelY, accelZ, aSqrt, gyroX, gyroY, gyroZ, mDirection, mX, mY,
    mZ;

float X = 0;
float Y = 0;
float Z = 0;

// Private functions
void set_X(float);
void set_Y(float);
void set_Z(float);
void filter_task(void *);

// Function implementations
void compensate() {
  accelZ = accelZ;
  accelY = accelY + 0.0039;
  accelX = accelX + 0.02;

  accelX = -accelX;
  accelY = accelY;
  accelZ = -accelZ;

  gyroX = gyroX - 0.3;
  gyroZ = gyroZ;

  gyroX = -gyroX;
  gyroY = gyroY;
  gyroZ = gyroZ;
}

void complementary_filter() {
  float h = 0.002;
  float alpha = 0.1;
  float gamma = alpha / (h + alpha);

  float gyrX = degToRad(gyroX) * h;
  float gyrY = degToRad(gyroY) * h;
  float gyrZ = degToRad(gyroZ) * h;

  // Add previous estimated angles to gyro estimated angle change and multiply
  // with gamma
  gyrX = (gyrX + X) * gamma;
  gyrY = (gyrY + Y) * gamma;
  gyrZ = (gyrZ + Z) * 0.99; // Forgetting factor

  // Convert acc to rad
  float accX = accelX * g;
  float accY = accelY * g;
  float accZ = accelZ * g;

  float phi = atan2(accY, accZ);
  float theta = atan2(-accX, sqrt(pow(accY, 2) + pow(accZ, 2)));

  phi = phi * (1 - gamma);
  theta = theta * (1 - gamma);

  // Add the estimations of the gyro and accelerometer
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  X = phi + gyrX;
  Y = theta + gyrY;
  Z = gyrZ;
  xSemaphoreGive(XYZ_lock);
}

void start_filter(xSemaphoreHandle wire_lock) {
  wire_lock_filter = wire_lock;
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  XYZ_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(XYZ_lock);
  xTaskCreatePinnedToCore(filter_task, "Filter-task",
                          configMINIMAL_STACK_SIZE * 10, NULL, 5, NULL, 0);
}

bool update_filter() {
  // Make sure wire is available
  xSemaphoreTake(wire_lock_filter, portMAX_DELAY);

  // check if there's new data
  if (mySensor.accelUpdate() != 0 || mySensor.gyroUpdate() != 0) {
    xSemaphoreGive(wire_lock_filter);
    return false;
  }

  // It's time to update, wire is available and there is new data. Estimate
  // orientation
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
  return true;
}

// This task continiously runs. It updates the orientation estimating whenever
// there's new data available on the imu. It also notifies the controller update
// task.
void filter_task(void *) {
  bool updated_orientation;

  for (;;) {
    // Update estimated orientation
    updated_orientation = update_filter();

    // If the orientation was updated then update the controller
    if (updated_orientation) {
      // printf("Gonna update controller\n");
      controller_update();
      vTaskDelay(1.f / FILTER_UPDATE_HZ *
                 1000); // Only wait if estimation is updated
    }
  }
}

float get_X() {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  float tmp = X;
  xSemaphoreGive(XYZ_lock);
  return tmp;
}

float get_Y() {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  float tmp = Y;
  xSemaphoreGive(XYZ_lock);
  return tmp;
}

float get_Z() {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  float tmp = Z;
  xSemaphoreGive(XYZ_lock);
  return tmp;
}

void set_X(float tmp) {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  X = tmp;
  xSemaphoreGive(XYZ_lock);
}

void set_Y(float tmp) {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  Y = tmp;
  xSemaphoreGive(XYZ_lock);
}

void set_Z(float tmp) {
  xSemaphoreTake(XYZ_lock, portMAX_DELAY);
  Z = tmp;
  xSemaphoreGive(XYZ_lock);
}
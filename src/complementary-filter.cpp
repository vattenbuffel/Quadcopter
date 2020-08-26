#include "complementary-filter.h"
#include "FreeRTOS.h"
#include "controller.h"
#include <MPU9250_asukiaaa.h>

#define g -9.81

MPU9250_asukiaaa mySensor;
xSemaphoreHandle wire_lock_filter, XYZ_lock;
float accelX, accelY, accelZ, aSqrt, gyroX, gyroY, gyroZ, mDirection, mX, mY,
    mZ, accelX_correction, accelY_correction, accelZ_correction,
    gyroX_correction, gyroY_correction, gyroZ_correction, ddx, ddy, ddz;

float X = 0;
float Y = 0;
float Z = 0;

void set_X(float);
void set_Y(float);
void set_Z(float);
void filter_task(void *);

// Function implementations
void compensate() {
  accelZ = accelZ + accelZ_correction;
  accelY = accelY + accelY_correction;
  accelX = accelX + accelX_correction;

  accelX = -accelX;
  accelY = accelY;
  accelZ = -accelZ;

  gyroX = gyroX + gyroX_correction;
  gyroY = gyroY + gyroY_correction;
  gyroZ = gyroZ + gyroZ_correction;

  gyroX = -gyroX;
  gyroY = gyroY;
  gyroZ = gyroZ;
}

// Takes the accelerations and removes the effect of g and turns them into the
// world frame
void filter_calc_accelerations_world_frame() {

  // Remove the effect of g
  float ddx_body_frame = 9.81 * accelX - g * sin(-Y) * cos(Z);
  float ddy_body_frame = 9.81 * accelY - g * sin(X) * cos(Z);
  //  float ddz_body_frame = ??;

  // Add the effect of the accelerations in body x-dir
  ddx = ddx_body_frame * cos(Y) * cos(Z);
  // ddy = ddx_body_frame * ?;

  // Add the effect of the accelerations in body y-dir
  ddx +=  ddy_body_frame * sin(Z) * sin(Y);
  // ddy += ddy_body_frame * ?;

  // Add the effect of the accelerations in body z-dir
  // ddx += ddz_body_frame * ?;
  // ddy += ddz_body_frame * ?;

  // printf("ddx: %f\n", ddx);
  // printf("AccelX: %f\n", accelX);
  // printf("Correction term: %f\n", - g*sin(-Y)*cos(Z));
  // printf("ddx: %f\tddy: %f\n", ddx, ddy);
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
  gyrZ = (gyrZ + Z);

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
  xSemaphoreTake(wire_lock_filter, portMAX_DELAY);
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();
  xSemaphoreGive(wire_lock_filter);

  accelX_correction = -0.008054409495549;
  accelY_correction = -0.009032411473788;
  accelZ_correction = -0.012852250244348;
  gyroX_correction = -0.138294016798419;
  gyroY_correction = 0.191549025691701;
  gyroZ_correction = -0.137208373517786;

  XYZ_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(XYZ_lock);
  xTaskCreatePinnedToCore(filter_task, "Filter-task",
                          configMINIMAL_STACK_SIZE * 10, NULL, 5, NULL, 0);
  printf("Started complementary filter.\n");
}

bool update_filter() {
  // Make sure wire is available
  // printf("filter trying to take wire_lock\n");
  xSemaphoreTake(wire_lock_filter, portMAX_DELAY);
  // printf("filter took wire_lock\n");

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
      filter_calc_accelerations_world_frame();
      controller_update();
      vTaskDelay(1.f / FILTER_UPDATE_HZ * 1000 /
                 portTICK_RATE_MS); // Only wait if estimation is updated
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

float get_ddx() { return ddx; }

float get_ddy() { return ddy; }

float get_ddz() {
  printf("ERROR: get_ddz() is not fully implemented\n");
  return ddz;
}
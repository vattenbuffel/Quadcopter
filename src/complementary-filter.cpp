#include "complementary-filter.h"
#include "FreeRTOS.h"
#include "controller.h"
#include <MPU9250_asukiaaa.h>
#include "SensorFusion.h"

#define g 9.81

MPU9250_asukiaaa mySensor;
SF fusion;
xSemaphoreHandle  XYZ_lock;
float ax, ay, az, gx, gy, gz;

float X = 0;
float Y = 0;
float Z = 0;

void set_X(float);
void set_Y(float);
void set_Z(float);
void filter_task(void *);

// Function implementations


void start_filter() {
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();


  XYZ_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(XYZ_lock);
  xTaskCreatePinnedToCore(filter_task, "Filter-task",
                          configMINIMAL_STACK_SIZE * 10, NULL, 5, NULL, 0);
  printf("Started complementary filter.\n");
}

bool update_filter() {
  // check if there's new data
  if (mySensor.accelUpdate() != 0 || mySensor.gyroUpdate() != 0) {
    return false;
  }

  // It's time to update, and there is new data. Estimate
  // orientation
    ax = mySensor.accelX();
    ay = mySensor.accelY();
    az = mySensor.accelZ();

    gx = mySensor.gyroX();
    gy = mySensor.gyroY();
    gz = mySensor.gyroZ();
    
    az = az + AZ_CORR;
    ay = ay + AY_CORR;
    ax = ax + AX_CORR;

    ax = -ax*g;
    ay = ay*g;
    az = -az*g;
    
    

    gx = gx + GX_CORR;
    gy = gy + GY_CORR;
    gz = gz + GZ_CORR;

    gx = degToRad(gx);
    gy = degToRad(-gy);
    gz = degToRad(gz);

    
    float deltat = fusion.deltatUpdate();
    // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, 0, 0, 0, deltat); // mahony is suggested if there isn't the mag
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, 0, 0, 0, deltat); //else use the magwick

    float roll = fusion.getRoll();
    if (roll < 180) 
        roll += 180;
    else    roll -= 180;
    if (roll > 180) roll-=360;
     
    float pitch = fusion.getPitch();
    float yaw = fusion.getYaw();

    xSemaphoreTake(XYZ_lock, portMAX_DELAY);
    X = degToRad(roll);
    Y = degToRad(-pitch);
    Z = degToRad(-yaw);
    xSemaphoreGive(XYZ_lock);

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
      controller_update();
      // vTaskDelay(1.f / FILTER_UPDATE_HZ * 1000 /
      //            portTICK_RATE_MS); // Only wait if estimation is updated
    }
    vTaskDelay(1);
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


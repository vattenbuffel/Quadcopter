#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include "VL53L0X.h"
#include "complementary-filter.h"
#include <Wire.h>
#include <math.h>
#include "controller.h"

QueueHandle_t distance_queue__;
VL53L0X distance_sensor;

// xSemaphoreHandle wire_lock_distance;
unsigned long distance_last_time_update;

float latest_height = CONTROLLER_HEIGHT_BASE_REF;
xSemaphoreHandle latest_height_lock = NULL;

// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void *pvParameters);

// Functions implementations

// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input) {
  printf("Starting distance sensor\n");
  Wire1.begin(WIRE1_SDA_PIN_NR, WIRE1_SCL_PIN_NR, 100000);
  distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
  if (!distance_sensor.init()) {
    printf("Failed to init distance sensor, VL53L0X\n");
    for (;;) {
    }
  }
  printf("Distance sensor initialized\n");
  distance_sensor.startContinuous();
  
  distance_queue__ = distance_queue_input;

  latest_height_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(latest_height_lock);

  xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement",
                          configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, 0);
  printf("Distance sensor started\n");

  distance_last_time_update = millis();
}

float distance_measurement_get_height() {
  if (latest_height_lock == NULL)
    return latest_height;
  
  float temp_height;
  xSemaphoreTake(latest_height_lock, portMAX_DELAY);
  temp_height = latest_height;
  xSemaphoreGive(latest_height_lock);
  return temp_height;
}

// This is the main distance_measurement task. It is responsible for adding the
// latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ
// dictates.
void distance_measurement_task(void *pvParameters) {
  for (;;) {
    height_type distance_m = distance_sensor.readRangeContinuousMillimeters() /
                             1000.f; // Convert mm to m
    
    // If the read distance is too big, i.e. the quad is too high or the
    // measurement freaks out, set the read value to a safe value rather than
    // the max which it's output as.
    if (distance_m > DISTANCE_MEASUREMENT_OOR_VALUE)
      distance_m = DISTANCE_MEASUREMENT_OOR_VALUE;

    // Sometime it will freak out for unkown reason. Restarting it seems to do
    // the trick.
    /*if (distance_sensor.timeoutOccurred()) {
      printf("Timeout on distance measurement\n");
      distance_sensor = VL53L0X();
      while (!distance_sensor.init()) {
        vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
      }
      distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
      distance_sensor.startContinuous();

      xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
      for(;;){
        printf("dist: %d\n",
        distance_sensor.readRangeContinuousMillimeters());
      }
    }*/

    // Calculate how high above ground the quadcopter is
    height_type height_m = distance_m * cos(get_X()) * cos(get_Y());
    // TEMP
    // height_type height_m = distance_m;
    // height_m = CONTROLLER_HEIGHT_BASE_REF;
    /////

    xQueueOverwrite(distance_queue__, &height_m);
    xSemaphoreTake(latest_height_lock, portMAX_DELAY);
    latest_height = height_m;
    xSemaphoreGive(latest_height_lock);

    vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
  }
}

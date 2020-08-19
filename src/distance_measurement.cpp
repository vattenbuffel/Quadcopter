#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include "VL53L0X.h"
#include "complementary-filter.h"
#include <Wire.h>
#include <math.h>

/**
 * This file takes care of measuring the distance downwards and with the help of
 * the estimated orientation calculates how far above the ground the quadcopter
 * is. It has occasionally freaked out and become unresponsive. It's unclear
 * what the cause of this is. Or how to combat it. Maybe it's due to bad
 * soldering and will get better once the custom PCB is installed.
 */

QueueHandle_t distance_queue__;
VL53L0X distance_sensor;

xSemaphoreHandle wire_lock_distance;

unsigned long distance_last_time_update;

// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void *pvParameters);

// Functions implementations

// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input,
                               xSemaphoreHandle wire_lock) {
  wire_lock_distance = wire_lock;
  printf("distance trying to take wire_lock\n");
  xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
  printf("distance took wire_lock\n");
  printf("gonna init distance\n");
  if (!distance_sensor.init()) {
    xSemaphoreGive(wire_lock_distance);
    printf("Failed to init distance sensor, VL53L0X\n");
    for (;;) {
    }
  }
  distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
  distance_sensor.startContinuous();
  xSemaphoreGive(wire_lock_distance);
  printf("Distance sensor initialized\n");

  xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement",
                          configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, 0);

  distance_queue__ = distance_queue_input;

  distance_last_time_update = millis();
}

// This is the main distance_measurement task. It is responsible for adding the
// latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ
// dictates.
void distance_measurement_task(void *pvParameters) {
  for (;;) {
    // printf("gonna read distance\n");
    // Make sure wire is available before reading
    xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
    // printf("got wire_lock so can now read distance\n");
    height_type distance_m = distance_sensor.readRangeContinuousMillimeters() /
                             1000.f; // Convert mm to m
    xSemaphoreGive(wire_lock_distance);
    // printf("read distance\n");

    // If the read distance is too big, i.e. the quad is too high or the
    // measurement freaks out, set the read value to a safe value rather than
    // the max which it's output as.
    // printf("Checkout distan oor\n");
    if (distance_m > DISTANCE_MEASUREMENT_OOR_VALUE)
      distance_m = DISTANCE_MEASUREMENT_OOR_VALUE;

    // Sometime it will freak out for unkown reason. Restarting it seems to do
    // the trick.
    // printf("Check distance timeout\n");
    if (distance_sensor.timeoutOccurred()) {
      printf("Timeout on distance measurement\n");
      // distance_sensor = VL53L0X();
      // while (!distance_sensor.init()) {
      //   vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
      // }
      // distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
      // distance_sensor.startContinuous();

      // xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
      // for(;;){
      //   printf("dist: %d\n",
      //   distance_sensor.readRangeContinuousMillimeters());
      // }
    }
    // printf("Checked distance timeout\n");

    // Calculate how high above ground the quadcopter is
    height_type height_m = distance_m;// * cos(get_X()) * cos(get_Y());
    // printf("Compensated reading for orientation\n");

    // printf("Gonna write to queue\n");
    xQueueOverwrite(distance_queue__, &height_m);
    // printf("wrote to queue\n");

    vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
    // printf("time since last distance update: %lu\n", millis() -
    // distance_last_time_update); distance_last_time_update = millis();
  }
}

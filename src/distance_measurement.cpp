#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include <math.h>
#include "complementary-filter.h"
#include "VL53L0X.h"
#include <Wire.h>

/**
 * This file takes care of measuring the distance downwards and with the help of the estimated orientation calculates how far above the ground the quadcopter is.
 * It has occasionally freaked out and become unresponsive. It's unclear what the cause of this is. Or how to combat it. Maybe it's due to bad soldering and will
 * get better once the custom PCB is installed.
*/

QueueHandle_t distance_queue__;
VL53L0X distance_sensor;

xSemaphoreHandle wire_lock_distance;

unsigned long distance_last_time_update;

// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void* pvParameters);

// Functions implementations

// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input, xSemaphoreHandle wire_lock){
  wire_lock_distance = wire_lock;
    
  
  if (!distance_sensor.init()){
    printf("Failed to init distance sensor, VL53L0X\n");
    for(;;){}
  }
  distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
  distance_sensor.startContinuous();
  printf("Distance sensor initialized\n");
  
  xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement", configMINIMAL_STACK_SIZE*4, NULL,  1, NULL, 0);
  
  distance_queue__ = distance_queue_input;


  distance_last_time_update = millis();
}


// This is the main distance_measurement task. It is responsible for adding the latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ dictates.
void distance_measurement_task(void* pvParameters){
  for(;;){
    // Make sure wire is available before reading
    xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
    height_type distance_m = distance_sensor.readRangeContinuousMillimeters() / 1000.f; // Convert mm to m
    xSemaphoreGive(wire_lock_distance);

    // If the read distance is too big, i.e. the quad is too high or the measurement freaks out up set the read value to a safe value rather than the max which it's output as.
    if (distance_m > DISTANCE_MEASUREMENT_OOR_VALUE) distance_m = DISTANCE_MEASUREMENT_OOR_VALUE;

    // Sometime it will freak out for unkown reason. Restarting it seems to do the trick.
    if(distance_sensor.timeoutOccurred()){
      printf("Timeout on distance measurement\n");
      xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
      // for(;;){
      //   printf("dist: %d\n", distance_sensor.readRangeContinuousMillimeters());
      // }
    }

    //Calculate how high above ground the quadcopter is
    height_type height_m = distance_m* cos(get_X())*cos(get_Y()); 

    // height_m = 1;
    xQueueOverwrite(distance_queue__, &height_m);
    
    vTaskDelay(1.0/DISTANCE_MEASUREMENT_HZ *1000 / portTICK_RATE_MS);
    // printf("time since last distance update: %lu\n", millis() - distance_last_time_update);
    // distance_last_time_update = millis();
  }
}



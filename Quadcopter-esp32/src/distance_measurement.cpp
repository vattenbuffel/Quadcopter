#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include <math.h>
#include "complementary-filter.h"
#include "VL53L0X.h"
#include <Wire.h>

/**
 * This file takes care of measuring the distance downwards and with the help of the estimated orientation calculates how far above the ground the quadcopter is.
 */

QueueHandle_t distance_queue__;
VL53L0X distance_sensor;

xSemaphoreHandle wire_lock_distance;

// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void* pvParameters);

// Functions implementations

// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input, xSemaphoreHandle wire_lock){

  wire_lock_distance = wire_lock;
    
    Wire.begin();
    if (!distance_sensor.init()){
      printf("Failed to init distance sensor, VL53L0X\n");
      for(;;){}
    }
    distance_sensor.setTimeout(500);
    distance_sensor.startContinuous();
    printf("Distance sensor initialized\n");
    
    xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement", configMINIMAL_STACK_SIZE*4, NULL,  1, NULL, 0);
    
    distance_queue__ = distance_queue_input;
}


// This is the main distance_measurement task. It is responsible for adding the latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ dictates.
void distance_measurement_task(void* pvParameters){

  for(;;){

    // Make sure wire is available before reading
    xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
    // printf("distance took lock\n");
    float distance_m = distance_sensor.readRangeContinuousMillimeters() / 1000.f; // Convert mm to m
    xSemaphoreGive(wire_lock_distance);
    // printf("read distance: %f\n", distance_m);

    // If the read distance is too big, i.e. the quad is too high or the measurement freaks out up set the read value to a safe value rather than the max which it's output as.
    if (distance_m > DISTANCE_MEASUREMENT_OOR_VALUE) distance_m = DISTANCE_MEASUREMENT_OOR_VALUE;

    //Calculate how high above ground the quadcopter is
    //printf("Cur X: %f\n", get_X());
    //printf("Cur Y: %f\n", get_Y());
    float height_m = distance_m * cos(get_X())*cos(get_Y());  
    //printf("height: %f\n", height_m);
    xQueueOverwrite(distance_queue__, &height_m);

    // xSemaphoreGive(wire_lock_distance);
    // printf("distance gave lock\n");
    
  
    vTaskDelay(1.0/DISTANCE_MEASUREMENT_HZ *1000 / portTICK_RATE_MS);
  }
}



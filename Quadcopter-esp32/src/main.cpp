#include "Arduino.h"
#include "FreeRTOS.h"
#include "command.h"
#include "distance_measurement.h"
#include "complementary-filter.h"
#include "controller.h"
#include "PID.h"

#include <Wire.h>
#include <VL53L0X.h>

QueueHandle_t distance_queue;
QueueHandle_t command_queue;

xSemaphoreHandle wire_lock;

unsigned long t_cur = millis();


void setup(){
  Serial.begin(115200);
  printf("\n");

  /*xSemaphoreHandle*/ wire_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(wire_lock);
  init_mpu(wire_lock);
  

  /*QueueHandle_t*/ command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  /*QueueHandle_t*/ distance_queue = xQueueCreate(1, sizeof(float));
  distance_measurement_init(distance_queue, wire_lock);
  
  
  controller_init(distance_queue, command_queue);
  


}

void loop(){
  volatile int data;
  float height;
  if(xQueueReceive(distance_queue, &height, 0) == pdTRUE) printf("distance: %f\n", height);
  //if(xQueueReceive(command_queue,  &data, 0) == pdTRUE) printf("command: %d\n", data);

  update_filter();
  // Serial.print("X :");
  // Serial.print(radToDeg(get_X()));
  // Serial.print("  Y :");
  // Serial.print(radToDeg(get_Y()));
  // Serial.print("  Z :");
  // Serial.println(radToDeg(get_Z()));
  

  //controller_update();
 
}










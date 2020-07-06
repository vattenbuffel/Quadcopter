#include "Arduino.h"
#include "FreeRTOS.h"
#include "command.h"
#include "distance_measurement.h"
#include "complementary-filter.h"
#include "controller.h"
#include "PID.h"

void core_zero_task(void* pvParameters);
QueueHandle_t distance_queue;
QueueHandle_t command_queue;


void setup(){
  Serial.begin(115200);

  init_mpu();

  /*QueueHandle_t*/ command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);
  
  /*QueueHandle_t*/ distance_queue = xQueueCreate(1, sizeof(volatile int));
  distance_measurement_init(distance_queue);
  
  controller_init(distance_queue, command_queue);
  
}

void loop(){
  volatile int data;
  //if(xQueueReceive(distance_queue, (void*) &data, 0) == pdTRUE) printf("distance: %d\n", data);
  //if(xQueueReceive(command_queue, (void*) &data, 0) == pdTRUE) printf("command: %d\n", data);

  update_filter();
  /*Serial.print("X :");
  Serial.print(radToDeg(get_X()));
  Serial.print("  Y :");
  Serial.print(radToDeg(get_Y()));
  Serial.print("  Z :");
  Serial.println(radToDeg(get_Z()));
  delay(20);*/

  controller_update();

}










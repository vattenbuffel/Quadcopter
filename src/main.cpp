#include "Arduino.h"
#include "FreeRTOS.h"
#include "PID.h"
#include "command.h"
#include "complementary-filter.h"
#include "controller.h"
#include "distance_measurement.h"
#include "node_red.h"
#include <Servo.h>
#include <Wire.h>

// This dictates if data should be published on node-red
#define NODE_RED

QueueHandle_t distance_queue;
QueueHandle_t command_queue;

// Lock for the complementary filter to give when it is done updating the
// orientation
xSemaphoreHandle orientation_updated;
xSemaphoreHandle wire_lock;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");

  wire_lock = xSemaphoreCreateBinary();
  orientation_updated = xSemaphoreCreateBinary();
  xSemaphoreGive(wire_lock);
  init_mpu(wire_lock, orientation_updated);

  command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  distance_queue = xQueueCreate(1, sizeof(height_type));
  // distance_measurement_init(distance_queue, wire_lock);

  controller_init(distance_queue, command_queue);

#ifdef NODE_RED
  node_red_start();
#endif // NODE_RED
}

void loop() {
  volatile int data;
  height_type height;
  // if(xQueueReceive(distance_queue, &height, 0) == pdTRUE) printf("distance:
  // %f\n", height);
  // if(xQueueReceive(command_queue,  &data, 0) == pdTRUE) printf("command:
  // %d\n", data);

  update_filter();
  // Serial.print("X :");
  // Serial.print(radToDeg(get_X()));
  // Serial.print("  Y :");
  // Serial.print(radToDeg(get_Y()));
  // Serial.print("  Z :");
  // Serial.println(radToDeg(get_Z()));

  if (pdTRUE == xSemaphoreTake(orientation_updated, 0)) {
    // controller_update();
  }
}

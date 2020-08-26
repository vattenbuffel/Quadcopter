#include "Arduino.h"
#include "FreeRTOS.h"
#include "PID.h"
#include "command.h"
#include "complementary-filter.h"
#include "controller.h"
#include "distance_measurement.h"
#include "location_estimation.h"
#include "node_red.h"
#include <MPU9250_asukiaaa.h>
#include <Servo.h>
#include <Wire.h>

// This dictates if data should be published on node-red
#define NODE_RED

QueueHandle_t distance_queue;
QueueHandle_t command_queue;

// Lock for the complementary filter to give when it is done updating the
// orientation
xSemaphoreHandle wire_lock;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");

  Servo ESC_NEm, ESC_SEm, ESC_SWm, ESC_NWm;
  ESC_NEm.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_NEm.writeMicroseconds(1050);
  ESC_NWm.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_NWm.writeMicroseconds(1050);
  ESC_SEm.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_SEm.writeMicroseconds(1050);
  ESC_SWm.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_SWm.writeMicroseconds(1050);
  for(;;){}

  wire_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(wire_lock);
  start_filter(wire_lock);

  command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  distance_queue = xQueueCreate(1, sizeof(height_type));
  distance_measurement_init(distance_queue, wire_lock);

  // location_estimation_start();

  controller_start(distance_queue, command_queue);

#ifdef NODE_RED
  node_red_start();
#endif // NODE_RED
}

void loop() {
  volatile int data;
  // if(xQueueReceive(command_queue,  &data, 0) == pdTRUE) printf("command:
  // %d\n", data);

  // printf("Cur height: %f\n", distance_measurement_get_height());

  // Serial.print("X :");
  // Serial.print(radToDeg(get_X()));
  // Serial.print("  Y :");
  // Serial.print(radToDeg(get_Y()));
  // Serial.print("  Z :");
  // Serial.println(radToDeg(get_Z()));
}

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

xSemaphoreHandle wire_lock;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");
  delay(1000);
  Servo NE, SE, SW, NW;
  bool pin_error = false;
  pin_error = !NE.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  NE.writeMicroseconds(1050);
  pin_error =
      pin_error ||
      !SE.attach(
          SE_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  SE.writeMicroseconds(1050);
  pin_error =
      pin_error ||
      !SW.attach(
          SW_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  SW.writeMicroseconds(1050);
  pin_error =
      pin_error ||
      !NW.attach(
          NW_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  NW.writeMicroseconds(1050);

  // vTaskDelay(5000/portTICK_RATE_MS);
  // NW.writeMicroseconds(1000);
  // SW.writeMicroseconds(1000);
  // SE.writeMicroseconds(1000);
  // NE.writeMicroseconds(1000);






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

#include "Arduino.h"
#include "FreeRTOS.h"
#include "PID.h"
#include "command.h"
#include "complementary-filter.h"
#include "controller.h"
#include "distance_measurement.h"
#include "node_red.h"
#include <MPU9250_asukiaaa.h>
#include <Servo.h>
#include <Wire.h>

// This dictates if data should be published on node-red
// #define NODE_RED

QueueHandle_t distance_queue;
QueueHandle_t command_queue;

// Lock for the complementary filter to give when it is done updating the
// orientation
xSemaphoreHandle wire_lock;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");

  wire_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(wire_lock);
  start_filter(wire_lock);

  command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  distance_queue = xQueueCreate(1, sizeof(height_type));
  distance_measurement_init(distance_queue, wire_lock);

  controller_start(distance_queue, command_queue);

#ifdef NODE_RED
  node_red_start();
#endif // NODE_RED
}

void loop() {
  volatile int data;
  // if(xQueueReceive(command_queue,  &data, 0) == pdTRUE) printf("command:
  // %d\n", data);

  printf("Cur height: %f\n", distance_measurement_get_height());

  // Serial.print("X :");
  // Serial.print(radToDeg(get_X()));
  // Serial.print("  Y :");
  // Serial.print(radToDeg(get_Y()));
  // Serial.print("  Z :");
  // Serial.println(radToDeg(get_Z()));
}

// #include <Wire.h>
// #include <VL53L0X.h>
// // #include "Arduino.h"
// VL53L0X sensor;

// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin();

//   sensor.setTimeout(500);
//   if (!sensor.init())
//   {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1) {}
//   }

//   // Start continuous back-to-back mode (take readings as
//   // fast as possible).  To use continuous timed mode
//   // instead, provide a desired inter-measurement period in
//   // ms (e.g. sensor.startContinuous(100)).
//   sensor.startContinuous();
// }

// void loop()
// {
//   Serial.print(sensor.readRangeContinuousMillimeters());
//   if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

//   Serial.println();
// }
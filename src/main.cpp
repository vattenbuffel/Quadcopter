#include "Arduino.h"
#include "FreeRTOS.h"
#include "PID.h"
#include "command.h"
#include "orientation-estimation.h"
#include "controller.h"
#include "distance_measurement.h"
#include "location_estimation.h"
#include "node_red.h"
#include <MPU9250_asukiaaa.h>
#include <Servo.h>
#include <Wire.h>

// This dictates if data should be published on node-red
// #define NODE_RED

QueueHandle_t distance_queue;
QueueHandle_t command_queue;


void setup() {
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");
  // delay(1000);
  // Servo NE, SE, SW, NW;
  // bool pin_error = false;
  // int speed = 1050;
  // pin_error = !NE.attach(
  //     NE_PIN, -1, 0, 180, 1000,
  //     2000); // (pin, min pulse width, max pulse width in microseconds)
  // NE.writeMicroseconds(1000);
  // pin_error =
  //     pin_error ||
  //     !SE.attach(
  //         SE_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // SE.writeMicroseconds(1000);
  // pin_error =
  //     pin_error ||
  //     !SW.attach(
  //         SW_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // SW.writeMicroseconds(1000);
  // pin_error =
  //     pin_error ||
  //     !NW.attach(
  //         NW_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // NW.writeMicroseconds(1000);


  // vTaskDelay(1000/portTICK_RATE_MS);
  // NE.writeMicroseconds(speed);
  // NW.writeMicroseconds(speed);
  // SE.writeMicroseconds(speed);
  // SW.writeMicroseconds(speed);

  // vTaskDelay(50000/portTICK_RATE_MS);
  // NW.writeMicroseconds(1000);
  // SW.writeMicroseconds(1000);
  // SE.writeMicroseconds(1000);
  // NE.writeMicroseconds(1000);

  start_orientation_estimation();

  command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  distance_queue = xQueueCreate(1, sizeof(height_type));
  distance_measurement_init(distance_queue);

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

  // delay(100);
}




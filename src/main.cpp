#include "Arduino.h"
#include "FreeRTOS.h"
#include "command.h"
#include "distance_measurement.h"
#include "complementary-filter.h"
#include "controller.h"
#include "PID.h"
#include <Servo.h>
#include <Wire.h>

/** TODO
 * Skriv funktion för att kalibrera motorerna
 * 
 * 
 */


QueueHandle_t distance_queue;
QueueHandle_t command_queue;

xSemaphoreHandle wire_lock;


void setup(){
  Serial.begin(115200);
  Wire.begin();

  printf("\nStarted!\n");
  
  wire_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(wire_lock);
  init_mpu(wire_lock);
  
  command_queue = xQueueCreate(10, sizeof(int));
  command_init(command_queue);

  distance_queue = xQueueCreate(1, sizeof(height_type));
  // distance_measurement_init(distance_queue, wire_lock);
  
  
  // controller_init(distance_queue, command_queue);
  

  // Servo test_motor0, test_motor15, test_motor2, test_motor4;
  // if(!test_motor0.attach(0, -1, 0, 180, 1000, 2000)) printf("Faulty servo pin\n");
  // if(!test_motor15.attach(15, -1, 0, 180, 1000, 2000)) printf("Faulty servo pin\n");
  // if(!test_motor2.attach(2, -1, 0, 180, 1000, 2000)) printf("Faulty servo pin\n");
  // if(!test_motor4.attach(4, -1, 0, 180, 1000, 2000)) printf("Faulty servo pin\n");
  // test_motor0.writeMicroseconds(1000);
  // test_motor15.writeMicroseconds(1000);
  // test_motor2.writeMicroseconds(1000);
  // test_motor4.writeMicroseconds(1000);
  // delay(500);
  
  // for (;;) {
  //   printf("MAX!\n");
  //   test_motor0.writeMicroseconds(1100);
  //   // test_motor2.writeMicroseconds(2000);
  //   // test_motor15.writeMicroseconds(2000);
  //   // test_motor4.writeMicroseconds(2000);
  //   delay(100000000);

  //   printf("MIN!\n");
  //   test_motor0.writeMicroseconds(1000);
  //   test_motor15.writeMicroseconds(1000);
  //   test_motor2.writeMicroseconds(1000);
  //   test_motor4.writeMicroseconds(1000);
  //   delay(100);
  // }

  // Servo test_motor0;
  // if(!test_motor0.attach(0, -1, 0, 180, 1000, 2000)) printf("Faulty servo pin\n");
  // for (;;) {
  //   printf("MAX!\n");
  //   test_motor0.writeMicroseconds(2000);
  //   delay(7000);

  //   printf("MIN!\n");
  //   test_motor0.writeMicroseconds(1000);
  //   delay(100000000);
    
  // }

}

void loop(){
  volatile int data;
  height_type height;
  // if(xQueueReceive(distance_queue, &height, 0) == pdTRUE) printf("distance: %f\n", height);
  //if(xQueueReceive(command_queue,  &data, 0) == pdTRUE) printf("command: %d\n", data);

  update_filter();
  // Serial.print("X :");
  // Serial.print(radToDeg(get_X()));
  // Serial.print("  Y :");
  // Serial.print(radToDeg(get_Y()));
  // Serial.print("  Z :");
  // Serial.println(radToDeg(get_Z()));
  
  controller_update();
 
}










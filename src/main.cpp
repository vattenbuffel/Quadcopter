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
  // delay(1000);
  // Servo NE, SE, SW, NW;
  // bool pin_error = false;
  // int speed = 1400;
  // pin_error = !NE.attach(
  //     NE_PIN, -1, 0, 180, 1000,
  //     2000); // (pin, min pulse width, max pulse width in microseconds)
  // NE.writeMicroseconds(speed);
  // pin_error =
  //     pin_error ||
  //     !SE.attach(
  //         SE_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // SE.writeMicroseconds(speed);
  // pin_error =
  //     pin_error ||
  //     !SW.attach(
  //         SW_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // SW.writeMicroseconds(speed);
  // pin_error =
  //     pin_error ||
  //     !NW.attach(
  //         NW_PIN, -1, 0, 180, 1000,
  //         2000); // (pin, min pulse width, max pulse width in microseconds)
  // NW.writeMicroseconds(speed);

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

//   Serial.print("X :");
//   Serial.print(radToDeg(get_X()));
//   Serial.print("  Y :");
//   Serial.print(radToDeg(get_Y()));
//   Serial.print("  Z :");
//   Serial.println(radToDeg(get_Z()));
}




// #include "Arduino.h"
// #include "SensorFusion.h"
// #include <MPU9250_asukiaaa.h>
// #include <Wire.h>

// #define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
// SF fusion;

// float pitch, roll, yaw;
// float deltat;

// MPU9250_asukiaaa mySensor;
// float ax, ay, az, aSqrt, gx, gy, gz, mDirection, mx, my,
//     mz, ax_corr, ay_corr, az_corr, gx_corr, gy_corr, gz_corr, ddx, ddy, ddz;

// int status;

// #define g 9.81

// void setup() {
//   // serial to display data
//   Serial.begin(115200);
//   while (!Serial) {
//   }

//   //mySensor.setWire(&Wire);
//   delay(100);
//   mySensor.beginAccel();
//   mySensor.beginGyro();
//   mySensor.beginMag();

//   ax_corr = -0.003;
//   ay_corr = 0.023;
//   az_corr = -0.043;
//   gx_corr = 0.35;
//   gy_corr = 0.06;
//   gz_corr = -0.73;
// }

// void loop() {
//   if (mySensor.accelUpdate() == 0 && mySensor.gyroUpdate() == 0 ){
//     ax = mySensor.accelX();
//     ay = mySensor.accelY();
//     az = mySensor.accelZ();
    
//     az = az + az_corr;
//     ay = ay + ay_corr;
//     ax = ax + ax_corr;

//     ax = -ax*g;
//     ay = ay*g;
//     az = -az*g;
    
//     gx = mySensor.gyroX();
//     gy = mySensor.gyroY();
//     gz = mySensor.gyroZ();

//     gx = gx + gx_corr;
//     gy = gy + gy_corr;
//     gz = gz + gz_corr;

//     gx = degToRad(gx);
//     gy = degToRad(-gy);
//     gz = degToRad(gz);

//     mx = mySensor.magX();
//     my = mySensor.magX();
//     mz = mySensor.magX();
    

// //   printf("ACC:\tx:%f  \t\ty:%f  \t\tz:%f  \n\n", ax, ay, az);
//     // printf("GYRO:\tx: %f  \t\ty:%f  \t\tz:%f \n", gx, gy, gz);
    

//     //   printf("From last Update:\t"); Serial.println(deltat, 6);
//     //   printf("MAG:\tx:%f  \t\ty:%f  \t\tz:%f  \n");

//     deltat = fusion.deltatUpdate();
//     // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, 0, 0, 0, deltat); // mahony is suggested if there isn't the mag
//       fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, 0, 0, 0, deltat); //else use the magwick

//     roll = fusion.getRoll();
//     if (roll < 180) 
//         roll += 180;
//     else    roll -= 180;
//     if (roll > 180) roll-=360;
     
//     pitch = fusion.getPitch();
//     yaw = fusion.getYaw();

//     float X = roll;
//     float Y = -pitch;
//     float Z = -yaw;

//     printf("X: %f\t\t Y: %f\t\t Z: %f\t  \n", X, Y, Z);

// }
// //   delay(200); //for readability
// }




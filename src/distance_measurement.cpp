#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include "VL53L0X.h"
#include "controller.h"
#include "orientation-estimation.h"
#include <Wire.h>
#include <math.h>
#include "node_red.h"
#include "config.h"
#include "math_.h"
#include "config.h"


QueueHandle_t distance_queue__;
VL53L0X distance_sensor;
Kalman kalman;


// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void *pvParameters);
void distance_measurement_kalman_init();
void distance_measurement_kalman_predict_step();
void distance_measurement_kalman_update_step(float reading);



// Functions implementations


void distance_measurement_kalman_init(){
  kalman.A = 1;
  kalman.C = 1;
  kalman.Q  = DISTANCE_MEASUREMENT_Q_H;
  kalman.R   = DISTANCE_MEASUREMENT_R;
  kalman.x_hat_predicted = 0;
  kalman.x_hat_updated = 0;
  kalman.P = 0; 
}

void distance_measurement_kalman_predict_step(){
  // calculate ts

  kalman.x_hat_predicted = kalman.A * kalman.x_hat_updated;
  // printf("kalman predicted height: %f\n", kalman.x_hat_predicted);

  kalman.P = kalman.A * kalman.P * kalman.A + kalman.Q;

  // printf("\n"); 
  // delay(100);
}

void distance_measurement_kalman_update_step(height_type reading){
  kalman.measured_height = reading;
  
  float S = kalman.C * kalman.P * kalman.C + kalman.R;
  float S_inv = 1/S;
  float v = reading - kalman.C * kalman.x_hat_predicted;
  float K = kalman.P*kalman.C*S_inv;

  kalman.x_hat_updated = kalman.x_hat_predicted + K*v;
  kalman.P -= K*S*K;

  // printf("Measured height: %f\n", reading);
  // printf("kalman filtered height: %f\n", kalman.x_hat_updated);
  
  // delay(100);  
  // printf("\n");
}




// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input) {
  printf("Starting distance sensor\n");
  printf("Gonna init wire1 with SDA pin: %d, and SCL pin: %d\n", WIRE1_SDA_PIN_NR, WIRE1_SCL_PIN_NR);
  if(!Wire1.begin(WIRE1_SDA_PIN_NR, WIRE1_SCL_PIN_NR, 100000)){
    printf("Failed to init wire1\n");
    for(;;){}
  }
  printf("Innited wire1\n");
  distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
  if (!distance_sensor.init()) {
    printf("Failed to init distance sensor, VL53L0X\n");
    for (;;) {
    }
  }
  printf("Distance sensor initialized\n");
  distance_sensor.startContinuous();

  distance_queue__ = distance_queue_input;

  

  // Initialize the kalman filter
  distance_measurement_kalman_init();

  xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement",
                          configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, 0);
  printf("Distance sensor started\n");

}

float distance_measurement_get_estimated_height() {
    
  float temp_height;
  
  temp_height = kalman.filtered_height;
  
  return temp_height;
}

float distance_measurement_get_predicted_height(){
  float temp_height;
  
  temp_height = kalman.x_hat_predicted;
  
  return temp_height;
  
}

float distance_measurement_get_height(){
  return distance_measurement_get_estimated_height();
}

float distance_measurement_get_measured_height(){
  float temp_height;
  
  temp_height = kalman.measured_height;
  
  return temp_height;
}

// This is the main distance_measurement task. It is responsible for adding the
// latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ
// dictates.
void distance_measurement_task(void *pvParameters) {
  for (;;) {
    height_type distance_m = distance_sensor.readRangeContinuousMillimeters() /
                             1000.f; // Convert mm to m

    // If the read distance is too big, i.e. the quad is too high or the
    // measurement freaks out, set the read value to a safe value rather than
    // the max which it's output as.
    if (distance_m > DISTANCE_MEASUREMENT_OOR_VALUE)
      distance_m = DISTANCE_MEASUREMENT_OOR_VALUE;

    // Sometime it will freak out for unkown reason. Restarting it seems to do
    // the trick.
    /*if (distance_sensor.timeoutOccurred()) {
      printf("Timeout on distance measurement\n");
      distance_sensor = VL53L0X();
      while (!distance_sensor.init()) {
        vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
      }
      distance_sensor.setTimeout(DISTANCE_MEASUREMENT_TIME_OUT_MS);
      distance_sensor.startContinuous();

      xSemaphoreTake(wire_lock_distance, portMAX_DELAY);
      for(;;){
        printf("dist: %d\n",
        distance_sensor.readRangeContinuousMillimeters());
      }
    }*/

    // if it timed out
    if (distance_sensor.timeoutOccurred()) {
      char* error_msg = "Timeout on distance measurement";
      printf("%s\n", error_msg);
      #ifdef CONFIG_NODE_RED_ENABLE
        node_red_publish_error(error_msg);
        node_red_publish_error("Stopping controller");
      #endif //CONFIG_NODE_RED_ENABLE
      controller_stop();


    }

    // Calculate how high above ground the quadcopter is
    height_type height_m = distance_m * cos(get_X()) * cos(get_Y());
    // Predict kalman
    distance_measurement_kalman_predict_step();
    // Update kalman
    distance_measurement_kalman_update_step(height_m);
    kalman.filtered_height = kalman.x_hat_updated;

    xQueueOverwrite(distance_queue__, &kalman.filtered_height);

    vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
  }
}

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
#include <BasicLinearAlgebra.h>
#include "math_.h"
#include "config.h"


QueueHandle_t distance_queue__;
VL53L0X distance_sensor;
Kalman kalman;

// xSemaphoreHandle wire_lock_distance;
// unsigned long distance_last_time_update;

// float latest_height = CONTROLLER_HEIGHT_BASE_REF;
xSemaphoreHandle latest_height_lock = NULL;

// Private functions
void distance_measurement_send_new_data();
void distance_measurement_task(void *pvParameters);
void distance_measurement_kalman_init();
void distance_measurement_kalman_predict_step();
void distance_measurement_kalman_update_step(float reading);
void distance_measurement_kalman_print();



// Functions implementations

void distance_measurement_kalman_print(){
  printf("Going to print height kalman struct.\n");
}

void distance_measurement_kalman_init(){
  kalman.A << 1 , 0,
              0, 1;
  kalman.B  << 1,
              0;
  kalman.C  << 1, 0;
  kalman.Q  << DISTANCE_MEASUREMENT_Q_H, 0,
               0, DISTANCE_MEASUREMENT_Q_V;
  kalman.R   = DISTANCE_MEASUREMENT_R;
  kalman.acceleration_rotation_matrix.Fill(0); // This has to be filled in later
  kalman.x_hat_predicted.Fill(0);
  kalman.x_hat_updated.Fill(0);
  kalman.P.Fill(0); 
  kalman.last_time_update_ms = millis();
}

void distance_measurement_kalman_predict_step(){
  // Calculate the acceleration straight up
  // Fill in acceleration rotation matrix
  kalman.acceleration_rotation_matrix(0,0) = -sin(get_Y());
  kalman.acceleration_rotation_matrix(0,1) = sin(get_X());
  kalman.acceleration_rotation_matrix(0,2) = cos(get_X())*cos(get_Y());
  BLA::Matrix<3> a_vector = {get_ddx(),
                             get_ddy(),
                             get_ddz()};
  auto a_temp = (kalman.acceleration_rotation_matrix * a_vector) + G;
  float a = a_temp(0);

  // float X = radToDeg(get_X());
  // float Y = radToDeg(get_Y());
  // Serial << "orientation: " << X << ", " << Y  << '\n';
  // Serial << "kalman.acceleration_rotation_matrix: " << kalman.acceleration_rotation_matrix << '\n';
  // Serial << "a: " << a_temp << '\n';

  // calculate ts
  float ts = MILLIES_TO_S(millis() - kalman.last_time_update_ms);
  kalman.A(0,1) = ts;
  kalman.B(1,0) = ts;

  kalman.x_hat_predicted = kalman.A * kalman.x_hat_updated + kalman.B * a;
  // Serial << "Predicted states: " << kalman.x_hat_predicted << "\n"; 

  kalman.P = kalman.A * kalman.P * ~kalman.A + kalman.Q;

  // printf("\n"); 
  // delay(100);
}

void distance_measurement_kalman_update_step(height_type reading){
  kalman.measured_height = reading;
  BLA::Matrix<1,1> y = {reading};
  
  int res;


  auto S = kalman.C * kalman.P * ~kalman.C + kalman.R;
  auto S_inv = S.Inverse(&res);
  if (res != 0){
    printf("Error in distance measurement kalman filter. Couldn't invert S matrix.\n.");
    Serial << "S: " << S <<"\n";
    #ifdef CONFIG_NODE_RED_ENABLE
      node_red_publish_error("Error in distance measurement kalman filter. Couldn't invert S matrix.");
    #endif
  }
  auto v = y - kalman.C * kalman.x_hat_predicted;
  auto K = kalman.P*~kalman.C*S_inv;

  
  xSemaphoreTake(latest_height_lock, portMAX_DELAY);
  kalman.x_hat_updated = kalman.x_hat_predicted + K*v;
  xSemaphoreGive(latest_height_lock);
  kalman.P -= K*S*~K;

  // printf("Measured height: %f\n", reading);
  // Serial << "Filtered state vector:" << kalman.x_hat_updated <<"\n";
  
  kalman.last_time_update_ms = millis();
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

  latest_height_lock = xSemaphoreCreateBinary();
  xSemaphoreGive(latest_height_lock);

  // Initialize the kalman filter
  distance_measurement_kalman_init();

  xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement",
                          configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL, 0);
  printf("Distance sensor started\n");

  // distance_last_time_update = millis();
}

float distance_measurement_get_estimated_height() {
  if (latest_height_lock == NULL){
    return kalman.filtered_height;
  }
    
  float temp_height;
  xSemaphoreTake(latest_height_lock, portMAX_DELAY);
  temp_height = kalman.filtered_height;
  xSemaphoreGive(latest_height_lock);
  return temp_height;
}

float distance_measurement_get_predicted_height(){
  float temp_height;
  xSemaphoreTake(latest_height_lock, portMAX_DELAY);
  temp_height = (float)kalman.x_hat_predicted(0);
  xSemaphoreGive(latest_height_lock);
  return temp_height;
  
}

float distance_measurement_get_height(){
  return distance_measurement_get_estimated_height();
}

float distance_measurement_get_measured_height(){
  float temp_height;
  xSemaphoreTake(latest_height_lock, portMAX_DELAY);
  temp_height = kalman.measured_height;
  xSemaphoreGive(latest_height_lock);
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

    xQueueOverwrite(distance_queue__, &height_m);

    vTaskDelay(1.0 / DISTANCE_MEASUREMENT_HZ * 1000 / portTICK_RATE_MS);
  }
}

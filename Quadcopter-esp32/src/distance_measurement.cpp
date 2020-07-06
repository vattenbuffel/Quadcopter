#include "distance_measurement.h"
#include "Arduino.h"
#include "FreeRTOS.h"
#include <math.h>
#include "complementary-filter.h"

/**
 * This file takes care of measuring the distance downwards and with the help of the estimated orientation calculates how far above the ground the quadcopter is. 
 * This code is not thread safe, which is a problem. It works fine now but is unstable.
 * 
 */


volatile unsigned long start, end;
volatile bool finished;
bool send;
float distance_cm;
QueueHandle_t distance_queue__;

// Private functions
void IRAM_ATTR isr_dist();
void distance_measurement_send_new_data();
void distance_measurement_start_measurement_task(void* pvParameters);
void distance_measurement_convert_time_to_cm();

// Functions implementations

// Inits the distance measurement
void distance_measurement_init(QueueHandle_t distance_queue_input){
    pinMode(DISTANCE_MEASUREMENT_TRIG_PIN, OUTPUT);
    pinMode(DISTANCE_MEASUREMENT_ECHO_PIN, INPUT_PULLUP);
    attachInterrupt(DISTANCE_MEASUREMENT_ECHO_PIN, isr_dist, CHANGE);

    xTaskCreatePinnedToCore(distance_measurement_task, "Distance_measurement", configMINIMAL_STACK_SIZE*4, NULL,  1, NULL, 1);
    xTaskCreatePinnedToCore(distance_measurement_start_measurement_task, "Start_measurement", configMINIMAL_STACK_SIZE*4, NULL, 1, NULL, 1);

    distance_queue__ = distance_queue_input;
    finished = true;
    send = false;
}

void distance_measurement_convert_time_to_cm(){
  distance_cm = (end - start) / 58;
  distance_cm*=cos(get_X())*cos(get_Y()); 

}

void distance_measurement_send_new_data(int distance_cm){
    xQueueOverwrite(distance_queue__, (void*)&distance_cm);
}

// This is the main distance_measurement task. It is responsible for adding the latest measurement to the distance_queue as often as DISTANCE_MEASUREMENT_HZ dictates.
void distance_measurement_task(void* pvParameters){
    for(;;){
      if(send){
        distance_measurement_send_new_data(distance_cm);
        send = false;
      }
      vTaskDelay(1.0/DISTANCE_MEASUREMENT_HZ *1000 / portTICK_RATE_MS);
    }
}

// Starts the measurement and signals the main task when a new measurement is ready to be published. This loops as fast as possible. Not really a problem since it's on the general core
void distance_measurement_start_measurement_task(void* pvParameters){
  for(;;){
    if (finished){
      //Serial.println(distanceCm);
      digitalWrite(DISTANCE_MEASUREMENT_TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(DISTANCE_MEASUREMENT_TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(DISTANCE_MEASUREMENT_TRIG_PIN, LOW);

      distance_measurement_convert_time_to_cm();
      send = true;
      finished = false;
    }
    // This is required to keep the watchdog fed.
    vTaskDelay(1);
  }
}

// Measures how long echo is active, then sets finished to true signaling that the measurement is done and is ready to be published
void IRAM_ATTR isr_dist(){
  if(digitalRead(DISTANCE_MEASUREMENT_ECHO_PIN) == HIGH){
    start = micros();
    return;
  }
  else if(digitalRead(DISTANCE_MEASUREMENT_ECHO_PIN) == LOW){
    end = micros();
    finished = true;
    return;
  }
}

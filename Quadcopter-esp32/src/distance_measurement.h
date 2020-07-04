#ifndef DISTANCE_MEASUREMENT_H_
#define DISTANCE_MEASUREMENT_H_

#define DISTANCE_MEASUREMENT_TRIG_PIN  GPIO_NUM_32
#define DISTANCE_MEASUREMENT_ECHO_PIN GPIO_NUM_35
#define DISTANCE_MEASUREMENT_HZ 10


#include "FreeRTOS.h"


void distance_measurement_init(QueueHandle_t distance_queue);
void distance_measurement_task(void* pvParameters);



#include "Arduino.h"

#endif //DISTANCE_MEASUREMENT_H_
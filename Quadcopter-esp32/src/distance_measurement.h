#ifndef DISTANCE_MEASUREMENT_H_
#define DISTANCE_MEASUREMENT_H_

#define DISTANCE_MEASUREMENT_HZ 100
#define DISTANCE_MEASUREMENT_OOR_VALUE 1.5


#include "FreeRTOS.h"

typedef float height_type;

void distance_measurement_init(QueueHandle_t distance_queue, xSemaphoreHandle wire_lock);



#include "Arduino.h"

#endif //DISTANCE_MEASUREMENT_H_
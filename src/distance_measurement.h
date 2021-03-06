#ifndef DISTANCE_MEASUREMENT_H_
#define DISTANCE_MEASUREMENT_H_

#define DISTANCE_MEASUREMENT_HZ 100
#define DISTANCE_MEASUREMENT_OOR_VALUE 1.5
#define DISTANCE_MEASUREMENT_TIME_OUT_MS ((uint16_t)(1.f / DISTANCE_MEASUREMENT_HZ))
#define WIRE1_SDA_PIN_NR 25
#define WIRE1_SCL_PIN_NR 26

#include "FreeRTOS.h"

typedef float height_type;

void distance_measurement_init(QueueHandle_t distance_queue);
float distance_measurement_get_height();

#include "Arduino.h"

#endif // DISTANCE_MEASUREMENT_H_
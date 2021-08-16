#ifndef DISTANCE_MEASUREMENT_H_
#define DISTANCE_MEASUREMENT_H_

#include "FreeRTOS.h"

#define DISTANCE_MEASUREMENT_HZ 100
#define DISTANCE_MEASUREMENT_OOR_VALUE 1.5
#define DISTANCE_MEASUREMENT_TIME_OUT_MS ((uint16_t)(1.f / DISTANCE_MEASUREMENT_HZ))
#define WIRE1_SDA_PIN_NR 25
#define WIRE1_SCL_PIN_NR 26

// Noise for height in kalman filter
#define DISTANCE_MEASUREMENT_Q_H 0.01
// Noise for velocity in kalman filter
#define DISTANCE_MEASUREMENT_Q_V 0.1
// Noise for measurement in kalman filter
#define DISTANCE_MEASUREMENT_R 0.001


typedef float height_type;

struct Kalman{
    float A;
    float C;
    float P;
    float x_hat_predicted;
    float x_hat_updated;
    float Q;
    height_type measured_height;
    height_type filtered_height;
    float R;
    
};


void distance_measurement_init(QueueHandle_t distance_queue);
float distance_measurement_get_height();
float distance_measurement_get_predicted_height();
float distance_measurement_get_estimated_height();
float distance_measurement_get_measured_height();

#include "Arduino.h"

#endif // DISTANCE_MEASUREMENT_H_
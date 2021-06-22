#ifndef DISTANCE_MEASUREMENT_H_
#define DISTANCE_MEASUREMENT_H_

#include "FreeRTOS.h"
#include <BasicLinearAlgebra.h>

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

using namespace BLA;
struct Kalman{
    BLA::Matrix<2,2> A;
    BLA::Matrix<2,1> B;
    BLA::Matrix<1,2> C;
    BLA::Matrix<2,2> P;
    BLA::Matrix<2,1> x_hat_predicted;
    BLA::Matrix<2,1> x_hat_updated;
    BLA::Matrix<2,2> Q;
    height_type measured_height;
    height_type filtered_height;
    float R;
    BLA::Matrix<1,3> acceleration_rotation_matrix;
    unsigned long last_time_update_ms;
    
};


void distance_measurement_init(QueueHandle_t distance_queue);
float distance_measurement_get_height();
float distance_measurement_get_predicted_height();
float distance_measurement_get_estimated_height();
float distance_measurement_get_measured_height();

#include "Arduino.h"

#endif // DISTANCE_MEASUREMENT_H_
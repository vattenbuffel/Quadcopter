#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

#define FILTER_UPDATE_HZ 1000

#include "FreeRTOS.h"


void compensate();
void complementary_filter();
float get_X();
float get_Y();
float get_Z();
void init_mpu(xSemaphoreHandle wire_lock, xSemaphoreHandle orientation_updated);
void update_filter();

#endif //COMPLEMENTARY_FILTER_H_
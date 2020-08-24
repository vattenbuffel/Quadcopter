#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

#define FILTER_UPDATE_HZ 1000

#include "FreeRTOS.h"

void compensate();
void complementary_filter();
float get_X();
float get_Y();
float get_Z();
float get_ddx();
float get_ddy();
float get_ddz();

void start_filter(xSemaphoreHandle wire_lock);
bool update_filter();


#endif // COMPLEMENTARY_FILTER_H_
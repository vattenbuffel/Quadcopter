#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_
    

#define  AX_CORR -0.003
#define  AY_CORR  0.023
#define  AZ_CORR -0.043
#define  GX_CORR  0.35
#define  GY_CORR  0.06
#define  GZ_CORR -0.73

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

void start_orientation_estimation();
bool update_orientation_estimation();


#endif // COMPLEMENTARY_FILTER_H_
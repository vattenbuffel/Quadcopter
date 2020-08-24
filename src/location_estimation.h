#ifndef LOCATION_ESTIMATION_H_
#define LOCATION_ESTIMATION_H_

#define LOCATION_ESTIMATION_UPDATE_HZ 256

float location_estimation_get_x();
float location_estimation_get_y();
void location_estimation_start();
void location_estimation_update();

#endif // LOCATION_ESTIMATION_H_
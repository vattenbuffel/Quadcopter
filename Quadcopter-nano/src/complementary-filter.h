#ifndef COMPLEMENTARY_FILTER_H_
#define COMPLEMENTARY_FILTER_H_

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

void compensate();
void complementary_filter();
float get_X();
float get_Y();
float get_Z();
void init_mpu();
void update_filter();

#endif //COMPLEMENTARY_FILTER_H_
#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

typedef enum {
    POS_NE,
    POS_SE, 
    POS_SW,
    POS_NW,
} position;

typedef struct PID_t
{
    uint8_t pin;
    float throttle, base_throttle; // eventually update base_throttle based on the distance from the ground
    float Kp, Ki, IX, IY, IZ;
    float rX, rY, rZ; // in rad
    long t_prev;
    position pos;

};

void update_throttle(PID_t* pid);
void limit_throttle(PID_t* pid);
void change_ref(PID_t* pid, float rX, float rY, float rZ); // never used

#endif //PID_H_
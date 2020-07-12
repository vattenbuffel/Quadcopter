#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

typedef enum {
    POS_NE,
    POS_SE, 
    POS_SW,
    POS_NW,
} position;

typedef struct PID_orientation_t {
    uint8_t pin;
    float throttle, base_throttle, min_throttle, max_throttle; 
    float Kp, Ki, IX, IY, IZ;
    float rX, rY, rZ; // in rad
    long t_prev;
    position pos;

};

typedef struct PID_height_t {
    float base_throttle; // eventually update base_throttle based on the distance from the ground
    float Kp, Ki, I;
    float r; // in m
    long t_prev;
    QueueHandle_t distance_queue;
};

void update_throttle(PID_orientation_t* pid);
void limit_throttle(PID_orientation_t* pid);
void change_ref(PID_orientation_t* pid, float rX, float rY, float rZ);
void change_base_throttle(PID_orientation_t* pid, float base_throttle);

void update_throttle(PID_height_t* pid);
void change_ref(PID_height_t* pid, float r);

#endif //PID_H_
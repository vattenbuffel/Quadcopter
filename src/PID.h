#ifndef PID_H_
#define PID_H_

#include <Arduino.h>
#include "distance_measurement.h"


// Macro to calculate the I value corresponding to throttle in a PID
#define CONTROLLER_THROTTLE_TO_I(throttle, i) ((throttle)/min((double)i, 0.000000001))

typedef enum {
  POS_NE,
  POS_SE,
  POS_SW,
  POS_NW,
} position;

typedef struct PID_orientation_t {
  uint8_t pin;
  float throttle, base_throttle, min_throttle, max_throttle;
  float Kp, Ki, Kd, IX, IY, IZ, e_prev_X, e_prev_Y, e_prev_Z;
  float rX, rY, rZ; // in rad
  float prev_p_effect, prev_i_effect, prev_d_effect;
  long t_prev;
  position pos;
};

typedef struct PID_height_t {
  float base_throttle; 
  float Kp, Ki, I;
  float r; // in m
  long t_prev;
};

void update_throttle(PID_orientation_t *pid);
void limit_throttle(PID_orientation_t *pid);
void change_ref(PID_orientation_t *pid, float rX, float rY, float rZ);
void change_base_throttle(PID_orientation_t *pid, float base_throttle);

void update_throttle(PID_height_t *pid, height_type height);
void change_ref(PID_height_t *pid, float r);

#endif // PID_H_
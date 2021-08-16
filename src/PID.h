#ifndef PID_H_
#define PID_H_

#include <Arduino.h>
#include "distance_measurement.h"


// Macro to calculate the I value corresponding to throttle in a PID. Returns 0 if i is 0
#define CONTROLLER_THROTTLE_TO_I(throttle, i) (!i ? (double) 0 : (throttle)/(i))

typedef enum {
  POS_NE,
  POS_SE,
  POS_SW,
  POS_NW,
} position;

struct PID_orientation_t {
  uint8_t pin;
  float throttle, output_throttle;
  float Kp, Ki, Kd, IX, IY, IZ, e_prev_X, e_prev_Y, e_prev_Z;
  float rX, rY, rZ; // in rad
  float prev_p_effect, prev_i_effect, prev_d_effect;
  long t_prev;
  position pos;
};

struct PID_height_t {
  float throttle;
  float Kp, Ki, I;
  float r; // in m
  long t_prev;
};

void update_throttle(PID_orientation_t *pid);
void change_ref(PID_orientation_t *pid, float rX, float rY, float rZ);

void update_throttle(PID_height_t *pid, height_type height);
void change_ref(PID_height_t *pid, float r);

#endif // PID_H_
#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Arduino.h>
#include "min_max.h"

#define NE_PIN 2
#define SE_PIN 4
#define SW_PIN 15
#define NW_PIN 0

#define CONTROLLER_PID_ORIENTATION_P 100
#define CONTROLLER_PID_ORIENTATION_I 10
#define CONTROLLER_PID_ORIENTATION_D 20

#define CONTROLLER_PID_HEIGHT_P 0
#define CONTROLLER_PID_HEIGHT_I 0

#define CONTROLLER_HEIGHT_BASE_REF 1
#define CONTROLLER_ORIENTATION_BASE_REF_X 0
#define CONTROLLER_ORIENTATION_BASE_REF_Y 0
#define CONTROLLER_ORIENTATION_BASE_REF_Z 0

#define CONTROLLER_HEIGHT_BASE_THROTTLE_VAL 200.f


#define CONTROLLER_MAX_X degToRad(40)
#define CONTROLLER_MAX_Y degToRad(40)

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

// These are parameters for the bluetooth controller
#define CONTROLLER_ORIENTATION_CHANGE degToRad(1)
#define CONTROLLER_BASE_THROTTLE_CHANGE 1
#define CONTROLLER_HEARTBEAT_HZ 0.05

#define CONTROLLER_MIN_THROTTLE 0
#define CONTROLLER_MAX_THROTTLE 1000.f

// Macro to limit throttle to between min and max value
#define CONTROLLER_LIMIT_THROTTLE(throttle) (max(CONTROLLER_MIN_THROTTLE, min(CONTROLLER_MAX_THROTTLE, (throttle))))

#include "PID.h"

typedef enum {
  command_left,
  command_up,
  command_right,
  command_down,
  command_select,
  command_start,
  command_square,
  command_triangle,
  command_cross,
  command_circle
} command_enum;

void controller_start(QueueHandle_t distance_queue, QueueHandle_t command_queue);
void controller_update();
void controller_update_ref_orientation();
void controller_update_ref_height();
void controller_update();
bool controller_set_orientation_p(float);
bool controller_set_orientation_i(float);
bool controller_set_orientation_d(float);
bool controller_set_height_p(float);
bool controller_set_height_i(float);
PID_orientation_t controller_get_NW();
PID_orientation_t controller_get_NE();
PID_orientation_t controller_get_SW();
PID_orientation_t controller_get_SE();
PID_height_t controller_get_height_pid();
bool controller_stopped();
float controller_get_base_throttle();
void controller_set_base_throttle(float throttle);


#endif // CONTROLLER_H_
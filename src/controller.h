#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define NE_PIN 2
#define SE_PIN 4
#define SW_PIN 0
#define NW_PIN 15

#define CONTROLLER_PID_ORIENTATION_P 100
#define CONTROLLER_PID_ORIENTATION_I 20
#define CONTROLLER_PID_ORIENTATION_D 0

// CONTROLLER_PID_HEIGHT_I can't be 0
#define CONTROLLER_PID_HEIGHT_P 0
#define CONTROLLER_PID_HEIGHT_I 20

#define CONTROLLER_HEIGHT_BASE_REF 1
#define CONTROLLER_ORIENTATION_BASE_REF_X 0
#define CONTROLLER_ORIENTATION_BASE_REF_Y 0
#define CONTROLLER_ORIENTATION_BASE_REF_Z 0

#define CONTROLLER_HEIGHT_PID_START_THROTTLE_VAL 450
#define CONTROLLER_HEIGHT_PID_START_I (CONTROLLER_HEIGHT_PID_START_THROTTLE_VAL/CONTROLLER_PID_HEIGHT_I)

#define CONTROLLER_MAX_X degToRad(30)
#define CONTROLLER_MAX_Y degToRad(30)

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0 / M_PI)

// These are parameters for the bluetooth controller
#define CONTROLLER_ORIENTATION_CHANGE degToRad(1)
#define CONTROLLER_BASE_THROTTLE_CHANGE 1
#define CONTROLLER_HEARTBEAT_HZ 0.05

#define CONTROLLER_MIN_THROTTLE 0
#define CONTROLLER_MAX_THROTTLE 1000

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


#endif // CONTROLLER_H_
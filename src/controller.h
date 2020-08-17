#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define NE_PIN 2
#define SE_PIN 4
#define SW_PIN 0
#define NW_PIN 15

#define CONTROLLER_PID_ORIENTATION_P 100
#define CONTROLLER_PID_ORIENTATION_I 50
#define CONTROLLER_PID_ORIENTATION_D 20
#define CONTROLLER_PID_HEIGHT_P 1
#define CONTROLLER_PID_HEIGHT_I 1

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

void controller_init(QueueHandle_t distance_queue, QueueHandle_t command_queue);
void controller_update();
void controller_update_ref_orientation();
void controller_update_ref_height();
PID_orientation_t controller_get_NW();
PID_orientation_t controller_get_NE();
PID_orientation_t controller_get_SW();
PID_orientation_t controller_get_SE();

#endif // CONTROLLER_H_
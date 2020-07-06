#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#define NE_PIN 11
#define SE_PIN 10
#define SW_PIN 9
#define NW_PIN 6

#define CONTROLLER_ORIENTATION_CHANGE 1
#define CONTROLLER_BASE_THROTTLE_CHANGE 1
#define CONTROLLER_HEARTBEAT_HZ 0.05

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
void controller_update_ref_orientation(); // Implement
void controller_update_ref_height(); // Implement



#endif //CONTROLLER_H_
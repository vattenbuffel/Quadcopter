#include "controller.h"
#include "PID.h"
#include "math.h"
#include "node_red.h"
#include <Servo.h>

PID_orientation_t pid_NE, pid_SE, pid_SW, pid_NW;
Servo ESC_NE, ESC_SE, ESC_SW, ESC_NW;

PID_height_t pid_height;

unsigned long heart_beat_time;
bool stop, calibration_active;
float bluetooth_base_throttle;

static TaskHandle_t motor_calibration_handle = NULL;
static TaskHandle_t controller_task_handle = NULL;

// Private functions
void controller_update_orientation();
void controller_actuate_motors();
void controller_set_base_throtle_orientation(float base_throttle);
void controller_set_ref_orientation(float rX, float rY, float rZ);
void controller_reset_controllers();
void controller_command_handler_task(void *pvParameter);
void controller_motor_calibration_handler();
void controller_motor_calibration_task(void *pvParameter);
void controller_update_task(void *);

void controller_start(QueueHandle_t distance_queue,
                      QueueHandle_t command_queue) {
  stop = true;
  calibration_active = false;
  heart_beat_time = millis();
  bluetooth_base_throttle = 0;

  pid_NE.pin = NE_PIN;
  pid_NE.pos = POS_NE;
  pid_NE.t_prev = micros();
  pid_NE.rZ = pid_NE.rY = pid_NE.rX = 0;
  pid_NE.Ki = CONTROLLER_PID_ORIENTATION_I;
  pid_NE.Kp = CONTROLLER_PID_ORIENTATION_P;
  pid_NE.Kd = CONTROLLER_PID_ORIENTATION_D;
  pid_NE.IX = pid_NE.IY = pid_NE.IZ = 0;
  pid_NE.base_throttle = 0;
  pid_NE.max_throttle = CONTROLLER_MAX_THROTTLE;
  pid_NE.min_throttle = CONTROLLER_MIN_THROTTLE;

  // Attach the motors, and set them to stationary. If a motor is connected to a
  // pin which doesn't support PWM, stop the code from running.
  bool pin_error = false;
  pin_error = !ESC_NE.attach(
      NE_PIN, -1, 0, 180, 1000,
      2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_NE.writeMicroseconds(1000);
  pin_error =
      pin_error ||
      !ESC_SE.attach(
          SE_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_SE.writeMicroseconds(1000);
  pin_error =
      pin_error ||
      !ESC_SW.attach(
          SW_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_SW.writeMicroseconds(1000);
  pin_error =
      pin_error ||
      !ESC_NW.attach(
          NW_PIN, -1, 0, 180, 1000,
          2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_NW.writeMicroseconds(1000);
  if (pin_error) {
    printf("ERROR, A servo pin does not support pwn\n");
    for (;;) {
      vTaskDelay(1000000);
    }
  }
  printf("Attached servos\n");

  memcpy(&pid_SE, &pid_NE, sizeof(pid_NE)); // not sure if this works
  pid_SE.pos = POS_SE;
  pid_SE.pin = SE_PIN;

  memcpy(&pid_SW, &pid_NE, sizeof(pid_NE)); // not sure if this works
  pid_SW.pos = POS_SW;
  pid_SW.pin = SW_PIN;

  memcpy(&pid_NW, &pid_NE, sizeof(pid_NE)); // not sure if this works
  pid_NW.pos = POS_NW;
  pid_NW.pin = NW_PIN;

  pid_height.base_throttle = 0;
  pid_height.distance_queue = distance_queue;
  pid_height.I = 0;
  pid_height.Kp = CONTROLLER_PID_HEIGHT_P;
  pid_height.Ki = CONTROLLER_PID_HEIGHT_I;
  pid_height.r = 1;
  pid_height.t_prev = micros();


  // Start the command handler so that the quad can be
  // controlled via bluetooth
  xTaskCreatePinnedToCore(
      controller_command_handler_task, "Command_handler_controller",
      configMINIMAL_STACK_SIZE * 5, (void *)command_queue, 1, NULL, 1);

  // Start controller_update_task
  xTaskCreatePinnedToCore(controller_update_task, "Controller_update_task",
                          configMINIMAL_STACK_SIZE * 5, NULL, 5,
                          &controller_task_handle, 0);
}

void controller_update_private() {
  if (calibration_active) {
    // printf("Calibration in progress. Can't update controller. \n");
    return;
  }

  update_throttle(&pid_height);
  controller_set_base_throtle_orientation(pid_height.base_throttle +
                                          bluetooth_base_throttle);
  controller_update_orientation();
  controller_actuate_motors();
}

void controller_actuate_motors() {
  // This is to make sure that the heartbeat signal has arrived in time. If not,
  // stop the motors
  if (1.0 / CONTROLLER_HEARTBEAT_HZ * 1000 + heart_beat_time < millis() ||
      stop) {
    stop = true;
    // printf("STOP\n");
    ESC_NE.writeMicroseconds(1000);
    ESC_SE.writeMicroseconds(1000);
    ESC_SW.writeMicroseconds(1000);
    ESC_NW.writeMicroseconds(1000);
    controller_reset_controllers();
    return;
  }

  ESC_NE.writeMicroseconds(pid_NE.throttle + 1000.f);
  ESC_SE.writeMicroseconds(pid_SE.throttle + 1000.f);
  ESC_SW.writeMicroseconds(pid_SW.throttle + 1000.f);
  ESC_NW.writeMicroseconds(pid_NW.throttle + 1000.f);
}

void controller_update_orientation() {
  update_throttle(&pid_NE);
  update_throttle(&pid_SE);
  update_throttle(&pid_SW);
  update_throttle(&pid_NW);
}

void controller_set_base_throtle_orientation(float base_throttle) {
  change_base_throttle(&pid_NE, pid_height.base_throttle);
  change_base_throttle(&pid_SE, pid_height.base_throttle);
  change_base_throttle(&pid_SW, pid_height.base_throttle);
  change_base_throttle(&pid_NW, pid_height.base_throttle);
}

void controller_set_ref_orientation(float rX, float rY, float rZ) {
  change_ref(&pid_NE, rX, rY, rZ);
  change_ref(&pid_SE, rX, rY, rZ);
  change_ref(&pid_SW, rX, rY, rZ);
  change_ref(&pid_NW, rX, rY, rZ);
}

void controller_reset_controllers() {
  // printf("reset controllers\n");
  pid_NE.IX = 0;
  pid_NE.IY = 0;
  pid_NE.IZ = 0;

  pid_SE.IX = 0;
  pid_SE.IY = 0;
  pid_SE.IZ = 0;

  pid_SW.IX = 0;
  pid_SW.IY = 0;
  pid_SW.IZ = 0;

  pid_NW.IX = 0;
  pid_NW.IY = 0;
  pid_NW.IZ = 0;

  controller_set_ref_orientation(CONTROLLER_ORIENTATION_BASE_REF_X,
                                 CONTROLLER_ORIENTATION_BASE_REF_Y,
                                 CONTROLLER_ORIENTATION_BASE_REF_Z);

  pid_height.I = 0;
  change_ref(&pid_height, CONTROLLER_HEIGHT_BASE_REF);
}

void controller_motor_calibration_handler() {
  if (motor_calibration_handle == NULL) {
    xTaskCreatePinnedToCore(controller_motor_calibration_task,
                            "motor_calibration", configMINIMAL_STACK_SIZE * 5,
                            NULL, 1, &motor_calibration_handle, 0);
    return;
  }

  xTaskNotify(motor_calibration_handle, 0, eNoAction);
}

PID_orientation_t controller_get_NW() { return pid_NW; }
PID_orientation_t controller_get_NE() { return pid_NE; }
PID_orientation_t controller_get_SW() { return pid_SW; }
PID_orientation_t controller_get_SE() { return pid_SE; }
PID_height_t controller_get_height_pid() { return pid_height; }
bool controller_stopped() { return stop; }

bool controller_set_orientation_p(float p) {
  pid_NE.Kp = p;
  pid_NW.Kp = p;
  pid_SE.Kp = p;
  pid_SW.Kp = p;
  return true;
}

bool controller_set_orientation_i(float i) {
  pid_NE.Kp = i;
  pid_NW.Ki = i;
  pid_SE.Ki = i;
  pid_SW.Ki = i;
  return true;
}

bool controller_set_orientation_d(float d) {
  pid_NE.Kd = d;
  pid_NW.Kd = d;
  pid_SE.Kd = d;
  pid_SW.Kd = d;
  return true;
}

bool controller_set_height_p(float p) {
  pid_height.Kp = p;
  return true;
}

bool controller_set_height_i(float i) {
  pid_height.Ki = i;
  return true;
}

void controller_update() {
  if (NULL != controller_task_handle) {
    xTaskNotify(controller_task_handle, 0, eNoAction);
  }
}

void controller_update_task(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    controller_update_private();
  }
}

// Calibrates the ESC/motors
void controller_motor_calibration_task(void *pvParameter) {
  for (;;) {
    // Stop the motors and activate calibration mode
    stop = true;
    calibration_active = true;
    ESC_NE.writeMicroseconds(1000.f);
    ESC_SE.writeMicroseconds(1000.f);
    ESC_SW.writeMicroseconds(1000.f);
    ESC_NW.writeMicroseconds(1000.f);

    printf(
        "MOTOR/ESC CALIBRATION INSTRUCTIONS: \n1) Unplug battery. \n2) Press "
        "the calibration button again. \n3) Plug the battery in again. \n4) "
        "Wait for the motors to rapidly beep twice then press calibration "
        "button again. \n5) That's it. Preferably restart the mcu. \n");

    // Set throttle high
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESC_NE.writeMicroseconds(2000.f);
    ESC_SE.writeMicroseconds(2000.f);
    ESC_SW.writeMicroseconds(2000.f);
    ESC_NW.writeMicroseconds(2000.f);
    printf("\n\n\n\n\n\n\nPlug in the battery.\nOnce two rapid beeps are heard "
           "press calibration button again.\n");

    // Set throttle low
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESC_NE.writeMicroseconds(1000.f);
    ESC_SE.writeMicroseconds(1000.f);
    ESC_SW.writeMicroseconds(1000.f);
    ESC_NW.writeMicroseconds(1000.f);

    printf("\n\n\n\n\n\n\nDone with calibration.\nYou should now restart the "
           "mcu.\n");
    delay(5000);
    calibration_active = false;
    vTaskDelete(NULL);
  }
}

// Handles the commands which are sent over bluetooth
void controller_command_handler_task(void *pvParameter) {
  QueueHandle_t command_queue = (QueueHandle_t)pvParameter;
  int command;

  for (;;) {
    xQueueReceive(command_queue, &command, portMAX_DELAY);
    heart_beat_time = millis();

    // This should be changed to a switch statement
    if (command == command_left) {
      controller_set_ref_orientation(pid_NE.rX - CONTROLLER_ORIENTATION_CHANGE,
                                     pid_NE.rY, 0);
    } else if (command == command_up) {
      controller_set_ref_orientation(
          pid_NE.rX, pid_NE.rY + CONTROLLER_ORIENTATION_CHANGE, 0);
    } else if (command == command_right) {
      controller_set_ref_orientation(pid_NE.rX + CONTROLLER_ORIENTATION_CHANGE,
                                     pid_NE.rY, 0);
    } else if (command == command_down) {
      controller_set_ref_orientation(
          pid_NE.rX, pid_NE.rY - CONTROLLER_ORIENTATION_CHANGE, 0);
    } else if (command == command_select) {
      stop = true;
    } else if (command == command_start) {
      stop = false;
    } else if (command == command_square) {
      // Heartbeat command. Do nothing
    } else if (command == command_triangle) {
      bluetooth_base_throttle += CONTROLLER_BASE_THROTTLE_CHANGE;
    } else if (command == command_cross) {
      controller_motor_calibration_handler();
    } else if (command == command_circle) {
      bluetooth_base_throttle -= CONTROLLER_BASE_THROTTLE_CHANGE;
    }
  }
}

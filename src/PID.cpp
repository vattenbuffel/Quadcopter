#include "PID.h"
#include "FreeRTOS.h"
#include "distance_measurement.h"
#include "orientation-estimation.h"

void update_throttle(PID_orientation_t *pid) {
  // Ignore Z for now

  // Calc error
  float eX = pid->rX - get_X();
  float eY = pid->rY - get_Y();

  // Calc time since last update
  float dt = (micros() - pid->t_prev) / (1000000.f);
  pid->t_prev = micros();

  // Calc I
  pid->IX += eX * dt;
  pid->IY += eY * dt;

  // Calc D
  float DX = (eX - pid->e_prev_X) / dt;
  float DY = (eY - pid->e_prev_Y) / dt;
  pid->e_prev_X = eX;
  pid->e_prev_Y = eY;

  // Calc throttle with controller placement in mind
  float p_effect, i_effect, d_effect;
  if (pid->pos == POS_NE) {
    p_effect = -pid->Kp * eX - pid->Kp * eY;
    i_effect = -pid->Ki * pid->IX - pid->Ki * pid->IY;
    d_effect = -pid->Kd * DX - pid->Kd * DY;

  } else if (pid->pos == POS_SE) {
    p_effect = -pid->Kp * eX + pid->Kp * eY;
    i_effect = -pid->Ki * pid->IX + pid->Ki * pid->IY;
    d_effect = -pid->Kd * DX + pid->Kd * DY;

  } else if (pid->pos == POS_SW) {
    p_effect = pid->Kp * eX + pid->Kp * eY;
    i_effect = pid->Ki * pid->IX + pid->Ki * pid->IY;
    d_effect = pid->Kd * DX + pid->Kd * DY;

  } else if (pid->pos == POS_NW) {
    p_effect = pid->Kp * eX - pid->Kp * eY;
    i_effect = pid->Ki * pid->IX - pid->Ki * pid->IY;
    d_effect = pid->Kd * DX - pid->Kd * DY;
  } else {
    printf("ERROR: Incorrect motor position\n");
    for (;;) {
    }
  }
  pid->throttle = p_effect + i_effect + d_effect;

  pid->prev_p_effect = p_effect;
  pid->prev_i_effect = i_effect;
  pid->prev_d_effect = d_effect;
}

/**
 * Changes the reference values of the orientation pids
 */
void change_ref(PID_orientation_t *pid, float rX, float rY, float rZ) {
  pid->rX = rX;
  pid->rY = rY;
  pid->rZ = rZ;
}

void change_ref(PID_height_t *pid, float r) { pid->r = r; }

void update_throttle(PID_height_t *pid, height_type height) {
  float e = pid->r - height;

  float dt = (micros() - pid->t_prev) / (1000000.f);
  pid->t_prev = micros();

  pid->I += e * dt;
  pid->throttle = pid->Kp * e + pid->Ki * pid->I;
}

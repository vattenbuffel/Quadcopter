#include "PID.h"
#include "complementary-filter.h"
#include "FreeRTOS.h"
#include "distance_measurement.h"

void update_throttle(PID_orientation_t* pid){
    // Ignore Z for now

    // Calc error
    float eX = pid->rX - get_X();
    float eY = pid->rY - get_Y();

    // Calc time since last update
    float dt = (micros() - pid->t_prev)/(1000000.f);
    pid->t_prev = micros();

    
    // Calc I
    pid->IX += eX*dt;
    pid->IY += eY*dt;

    // Calc D
    float DX = (eX - pid->e_prev_X)/dt;
    float DY = (eY - pid->e_prev_Y)/dt;
    pid->e_prev_X = eX;
    pid->e_prev_Y = eY;

    // Calc throttle with controller placement in mind
    float throttle = pid->base_throttle;
    if (pid->pos == POS_NE){
        throttle += -pid->Kp*eX - pid->Kp*eY;
        throttle += -pid->Ki*pid->IX - pid->Ki*pid->IY;
        throttle += -pid->Kd*DX - pid->Kd*DY;
    }
    else if(pid->pos == POS_SE){
        throttle += -pid->Kp*eX + pid->Kp*eY;
        throttle += -pid->Ki*pid->IX + pid->Ki*pid->IY;
        throttle += -pid->Kd*DX + pid->Kd*DY;
    }
    else if(pid->pos == POS_SW){
        throttle += pid->Kp*eX + pid->Kp*eY;
        throttle += pid->Ki*pid->IX + pid->Ki*pid->IY;
        throttle += pid->Kd*DX + pid->Kd*DY;
    }
    else if(pid->pos == POS_NW){
        throttle += pid->Kp*eX - pid->Kp*eY;
        throttle += pid->Ki*pid->IX - pid->Ki*pid->IY;
        throttle += pid->Kd*DX - pid->Kd*DY;
    }
    
    pid->throttle = throttle;


    limit_throttle(pid);
}

void limit_throttle(PID_orientation_t* pid){
    if (pid->throttle > pid->max_throttle) pid->throttle = pid->max_throttle;
    else if (pid->throttle < pid->min_throttle) pid->throttle = pid->min_throttle;
}



/**
 * Changes the reference values of the orientation pids 
 */
void change_ref(PID_orientation_t* pid, float rX, float rY, float rZ){
    pid->rX = rX;
    pid->rY = rY;
    pid->rZ = rZ;
}

void change_base_throttle(PID_orientation_t* pid, float base_throttle){
    pid->base_throttle = base_throttle;
}

void change_ref(PID_height_t* pid, float r){
    pid->r = r;
}

void update_throttle(PID_height_t* pid){
    height_type current_height; 

    if(xQueueReceive(pid->distance_queue, &current_height, 0) == pdFALSE) return;
    
    float e = current_height/100.f - pid->r; // Why div by 100? It's already m right?

    float dt = (micros() - pid->t_prev)/(1000000.f);
    pid->t_prev = micros();

    pid->I += e*dt;
    pid->base_throttle = pid->Kp*e + pid->Ki*pid->I;

}

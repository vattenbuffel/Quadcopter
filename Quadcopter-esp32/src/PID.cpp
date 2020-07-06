#include "PID.h"
#include "complementary-filter.h"
#include "FreeRTOS.h"

void update_throttle(PID_orientation_t* pid){
    // Ignore Z for now

    float eX = pid->rX - get_X();
    float eY = pid->rY - get_Y();

    float dt = (micros() - pid->t_prev)/(1000000.f);
    pid->t_prev = micros();

    float throttle = pid->base_throttle;
    pid->IX += eX*dt;
    pid->IY += eY*dt;

    if (pid->pos == POS_NE){
        throttle += -pid->Kp*eX - pid->Kp*eY;
        throttle += -pid->Ki*pid->IX - pid->Ki*pid->IY;
    }
    else if(pid->pos == POS_SE){
        throttle += -pid->Kp*eX + pid->Kp*eY;
        throttle += -pid->Ki*pid->IX + pid->Ki*pid->IY;
    }
    else if(pid->pos == POS_SW){
        throttle += pid->Kp*eX + pid->Kp*eY;
        throttle += pid->Ki*pid->IX + pid->Ki*pid->IY;
    }
    else if(pid->pos == POS_NW){
        throttle += pid->Kp*eX - pid->Kp*eY;
        throttle += pid->Ki*pid->IX - pid->Ki*pid->IY;
    }
    
    pid->throttle = throttle;
    limit_throttle(pid);
}

void limit_throttle(PID_orientation_t* pid){
    if (pid->throttle > 1000) pid->throttle = 1000;
    else if (pid->throttle < 0) pid->throttle = 0;
}



/**
 * Changes the reference values of the orientation pids 
 * Pass '/0' as the parameter if you don't want to change it.
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
    int current_height = 1; 

    if(xQueueReceive(pid->distance_queue, (void*)&current_height, 0) == pdFALSE) return;
    
    float e = current_height/100.f - pid->r;

    float dt = (micros() - pid->t_prev)/(1000000.f);
    pid->t_prev = micros();

    pid->I += e*dt;
    pid->base_throttle = pid->Kp*e + pid->Ki*pid->I;

}

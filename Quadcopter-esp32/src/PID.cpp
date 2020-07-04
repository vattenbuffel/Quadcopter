#include "PID.h"
#include "complementary-filter.h"

void update_throttle(PID_t* pid){
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

void limit_throttle(PID_t* pid){
    if (pid->throttle > 1000) pid->throttle = 1000;
    else if (pid->throttle < 0) pid->throttle = 0;
}

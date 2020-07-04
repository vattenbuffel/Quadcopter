#include "PID.h"
#include <Servo.h>
#include "controller.h"


PID_t pid_NE, pid_SE, pid_SW, pid_NW;
Servo ESC_NE, ESC_SE, ESC_SW, ESC_NW;


void controller_init(){
    
    pid_NE.pin = NE_PIN;
    pid_NE.pos = POS_NE;
    pid_NE.t_prev = micros();
    pid_NE.rZ = pid_NE.rY = pid_NE.rX = 0;
    pid_NE.Ki = 0.1;
    pid_NE.Kp = 1;
    pid_NE.IX = pid_NE.IY = pid_NE.IZ = 0;
    pid_NE.base_throttle = 0;
    
    ESC_NE.attach(NE_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
    ESC_SE.attach(SE_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
    ESC_SW.attach(SW_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
    ESC_NW.attach(NW_PIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

    memcpy(&pid_SE, &pid_NE, sizeof(pid_NE)); //not sure if this works
    pid_SE.pos = POS_SE;
    pid_SE.pin = SE_PIN;

    memcpy(&pid_SW, &pid_NE, sizeof(pid_NE)); //not sure if this works
    pid_SW.pos = POS_SW;
    pid_SW.pin = SW_PIN;

    memcpy(&pid_NW, &pid_NE, sizeof(pid_NE)); //not sure if this works
    pid_NW.pos = POS_NW;
    pid_NW.pin = NW_PIN;
}


void controller_update(){
    update_throttle(&pid_NE);
    update_throttle(&pid_SE);
    update_throttle(&pid_SW);
    update_throttle(&pid_NW);

    ESC_NE.writeMicroseconds(pid_NE.throttle + 1000);
    ESC_SE.writeMicroseconds(pid_SE.throttle + 1000);
    ESC_SW.writeMicroseconds(pid_SW.throttle + 1000);
    ESC_NW.writeMicroseconds(pid_NW.throttle + 1000);


}

#include "PID.h"
#include <Servo.h>
#include "controller.h"


PID_orientation_t pid_NE, pid_SE, pid_SW, pid_NW;
Servo ESC_NE, ESC_SE, ESC_SW, ESC_NW;

PID_height_t pid_height;

unsigned long heart_beat_time;
bool stop;
float bluetooth_base_throttle;

// Private functions
void controller_update_orientation();
void controller_actuate_motors();
void controller_set_base_throtle_orientation(float base_throttle);
void controller_set_ref_orientation(float rX, float rY, float rZ);
void controller_reset_controllers();
void controller_command_handler_task(void* pvParameter);



void controller_init(QueueHandle_t distance_queue, QueueHandle_t command_queue){
    stop = true;
    heart_beat_time = millis();
    bluetooth_base_throttle = 0;

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

    pid_height.base_throttle = 0;
    pid_height.distance_queue = distance_queue;
    pid_height.I = 0;
    pid_height.Kp = 1;
    pid_height.Ki = 1;
    pid_height.r = 1;
    pid_height.t_prev = micros();

  // Start the command handler so that the quad can be controlled via bluetooth
  xTaskCreatePinnedToCore(controller_command_handler_task, "Command_handler_controller", configMINIMAL_STACK_SIZE*5, (void*)command_queue, 1, NULL, 0);
}



void controller_update(){
    update_throttle(&pid_height);
    controller_set_base_throtle_orientation(pid_height.base_throttle + bluetooth_base_throttle);
    controller_update_orientation();
    controller_actuate_motors();

}

void controller_actuate_motors(){

    // This is to make sure that the heartbeat signal has arrived in time. If not, stop the motors
    if (1.0 / CONTROLLER_HEARTBEAT_HZ * 1000 + heart_beat_time < millis() || stop){
        ESC_NE.writeMicroseconds(0);
        ESC_SE.writeMicroseconds(0);
        ESC_SW.writeMicroseconds(0);
        ESC_NW.writeMicroseconds(0);
        controller_reset_controllers();
        return;
    }

    ESC_NE.writeMicroseconds(pid_NE.throttle + 1000);
    ESC_SE.writeMicroseconds(pid_SE.throttle + 1000);
    ESC_SW.writeMicroseconds(pid_SW.throttle + 1000);
    ESC_NW.writeMicroseconds(pid_NW.throttle + 1000);
}

void controller_update_orientation(){
    update_throttle(&pid_NE);
    update_throttle(&pid_SE);
    update_throttle(&pid_SW);
    update_throttle(&pid_NW);
}

void controller_set_base_throtle_orientation(float base_throttle){
    change_base_throttle(&pid_NE, pid_height.base_throttle);
    change_base_throttle(&pid_SE, pid_height.base_throttle);
    change_base_throttle(&pid_SW, pid_height.base_throttle);
    change_base_throttle(&pid_NW, pid_height.base_throttle);
}


void controller_set_ref_orientation(float rX, float rY, float rZ){
    change_ref(&pid_NE, rX, rY, rZ);
    change_ref(&pid_SE, rX, rY, rZ);
    change_ref(&pid_SW, rX, rY, rZ);
    change_ref(&pid_NW, rX, rY, rZ);
}

void controller_reset_controllers(){
    //printf("reset controllers\n");
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

    controller_set_ref_orientation(0, 0, 0);

    pid_height.I = 0;
    change_ref(&pid_height, 0);
}

// Handles the commands which are sent over bluetooth
void controller_command_handler_task(void* pvParameter){
    QueueHandle_t command_queue = (QueueHandle_t) pvParameter;
    int command;

    for(;;){
        xQueueReceive(command_queue, &command, portMAX_DELAY);
        heart_beat_time = millis();

        if(command == command_left){
            controller_set_ref_orientation(pid_NE.rX - CONTROLLER_ORIENTATION_CHANGE, pid_NE.rY, 0);
        }        
        else if(command == command_up){
            controller_set_ref_orientation(pid_NE.rX, pid_NE.rY + CONTROLLER_ORIENTATION_CHANGE, 0);
        }
        else if(command == command_right){
            controller_set_ref_orientation(pid_NE.rX + CONTROLLER_ORIENTATION_CHANGE, pid_NE.rY, 0);
        }
        else if(command == command_down){
            controller_set_ref_orientation(pid_NE.rX, pid_NE.rY - CONTROLLER_ORIENTATION_CHANGE, 0);
        }        
        else if(command == command_select){
            stop = true;
        }
        else if(command == command_start){
            stop = false;
        }
        else if(command == command_square){
            // Heartbeat command. Do nothing
        }
        else if(command == command_triangle){
            bluetooth_base_throttle += CONTROLLER_BASE_THROTTLE_CHANGE;
        }
        else if(command == command_cross){
            //Unused
        }
        else if(command == command_circle){
            bluetooth_base_throttle -= CONTROLLER_BASE_THROTTLE_CHANGE;
        }
    }
}


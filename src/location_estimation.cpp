#include "location_estimation.h"
#include "Arduino.h"
#include "orientation-estimation.h"

float x, dx, y, dy;
unsigned long prev_update_time;

static TaskHandle_t location_estimation_task_handle = NULL;

void location_estimation_task(void *);

void location_estimation_start(){
    x = 0;
    dx = 0;
    y = 0;
    dy = 0;
    prev_update_time = micros();

    xTaskCreatePinnedToCore(location_estimation_task, "Location-estimator", configMINIMAL_STACK_SIZE*3, NULL, 4, &location_estimation_task_handle, 0);
}

float location_estimation_get_x(){return x;}

float location_estimation_get_y(){return y;}

void location_estimation_update(){
    unsigned long new_update_time = micros();
    float dt = (new_update_time - prev_update_time)/1000000.f;
    prev_update_time = new_update_time;

    float ddx = get_ddx();
    float ddy = get_ddy();

    dx += dt*ddx;
    x += dx*dt;
    
    dy += dt*ddy;
    y += dy*dt;

    printf("ddx: %f, dx: %f, x: %f\n", ddx, dx, x);
}   

void location_estimation_task(void *){
    for(;;){
        location_estimation_update();
        // printf("x: %f y: %f\n", x, y);
        vTaskDelay(1.f / LOCATION_ESTIMATION_UPDATE_HZ * 1000 * portTICK_RATE_MS);
    }
}
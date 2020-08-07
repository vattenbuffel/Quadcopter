#ifndef COMMAND_H_
#define COMMAND_H_

#define COMMAND_BLUETOOTH_NAME "Quadcopter-Noa"
#define COMMAND_SEND_HZ 10

#include "FreeRTOS.h"


void command_init(QueueHandle_t command_queue_);
void command_task(void* pvParameters);

#endif //COMMAND_H_
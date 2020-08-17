#include "command.h"
#include "Arduino.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
QueueHandle_t command_queuee;

// Private functions
void command_read_data();

// Opens up the bluetooth communication
void command_init(QueueHandle_t command_queue_) {
  command_queuee = command_queue_;
  SerialBT.begin(COMMAND_BLUETOOTH_NAME);
  printf("The device started, now you can pair it with bluetooth!\n");
  xTaskCreatePinnedToCore(command_task, "Receive_commands",
                          configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL, 1);
}

// If there is any data sent via bluetooth add it to the command queue
void command_read_data() {
  if (SerialBT.available()) {
    int data = SerialBT.read();
    if (!(data == 13 || data == 10)) { // Should there be {} here?
      data -= 48;                      // Ugly conversion from ascii to numbers
      xQueueSend(command_queuee, &data,
                 0); // Add the sent data to the command_queue.
    }
  }
}

void command_task(void *pvParameters) {
  for (;;) {
    command_read_data();
    vTaskDelay(1.0 / COMMAND_SEND_HZ * 1000 / portTICK_RATE_MS);
  }
}
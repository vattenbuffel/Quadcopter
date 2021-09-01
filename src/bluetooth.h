#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "BluetoothSerial.h"
#include <ArduinoJson.h>

#define BLUETOOTH_NAME "Quadcopter-Noa"
#define BLUETOOTH_JSON_SIZE_B 200


void bluetooth_init();
bool send(const char*);
bool get_latest_str(char* buffer, int buffer_size);
bool get_latest_json(); // Should return a pointer. Unsure how this will work though.... or if it should work like this

// Struct containing everything bluetootheiver needs
struct BluetoothData{
    StaticJsonDocument<BLUETOOTH_JSON_SIZE_B> json;
    String cur_msg, latest_msg;
    BluetoothSerial SerialBT;
    int left_curly_n, right_curly_n;
};

#endif // BLUETOOTH_H_
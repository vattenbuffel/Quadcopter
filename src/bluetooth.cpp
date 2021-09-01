#include "Arduino.h"
#include "BluetoothSerial.h"
#include "bluetooth.h"
#include "math_.h"

BluetoothData data;


//======================================//
// Local function declarations
//======================================//
/* When a char has been received this function will do all the processing required afterwards. 
   Ie append it to the received string, increment the counters, parse the json etc
*/
bool bluetooth_handle_received_char(char c);
char bluetooth_read_char();
void bluetooth_task(void*);

// Converts the received string into a json
bool bluetooth_parse_msg();

// Reset meant to be used when a full msg has been received. It resest the cur_msg and curly counters
void bluetooth_soft_reset();



//======================================//
// Local function implementations
//======================================//
void bluetooth_soft_reset(){
  data.left_curly_n = 0;
  data.right_curly_n = 0;
  data.latest_msg = data.cur_msg;
  data.cur_msg = "";
}

bool bluetooth_handle_received_char(char c){
    if (c < 0 || c > 127){
      printf("bluetooth_handle_received_char: BIG ERROR. Received char which is not an ascii char. c has value: %d\n", c);
    }

    data.cur_msg += c;

    // Count the number of { and }
    if (c == 123) data.left_curly_n++;
    else if (c == 125) data.right_curly_n++;

    // If the number of { is equal to the number of } then a full json has been received, parse it
    if (data.left_curly_n && data.left_curly_n == data.right_curly_n){
      bool success = bluetooth_parse_msg();
      bluetooth_soft_reset();
      return success;
    }

    return true;
}

bool bluetooth_parse_msg(){
  DeserializationError err = deserializeJson(data.json, data.cur_msg);
  if (err){
    Serial.print(F("Deserialization failed with code: "));
    Serial.println(err.f_str());
    return false;
  }

  // Print parsed json
  printf("Parsed json: \n");
  serializeJsonPretty(data.json, Serial);
  printf("\n");
  return true;
}

// Read the string sent via bluetooth
char bluetooth_read_char() {
  char c = data.SerialBT.read();
  if (!(c == 13 || c == 10)) {  
    return c;
  }
  return -1;
}

void bluetooth_task(void *pvParameters) {
  for (;;) {
    // Read all chars in bluetooth
    while (data.SerialBT.available()){
      char c = bluetooth_read_char();
	  if (c != 255)
      	bluetooth_handle_received_char(c); 
    }

    // Serial.print("Data received: ");
    // Serial.println(data.cur_msg);
    vTaskDelay(HZ_TO_TICKS(1));
  }
}

//======================================//
// Global function implementations
//======================================//

// Opens up the bluetooth communication
void bluetooth_init() {
  bluetooth_soft_reset();
  data.SerialBT.begin(BLUETOOTH_NAME);
  printf("The device started, now you can pair it with bluetooth!\n");
  xTaskCreatePinnedToCore(bluetooth_task, "Bluetooth",
                          configMINIMAL_STACK_SIZE * 10, NULL, 1, NULL, 1);
  
}


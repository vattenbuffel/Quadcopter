#include <Arduino.h>
#include <Servo.h>
#include "complementary-filter.h"

/*TODO:
  Control altitude? Using a distance sensor and increasing/decreasing the base thrust?

*/

Servo ESC;





void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
  ESC.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  init_mpu();
  
}

void loop() {
   
    update_filter();
    Serial.print("X :");
    Serial.print(radToDeg(get_X()));
    Serial.print("  Y :");
    Serial.print(radToDeg(get_Y()));
    Serial.print("  Z :");
    Serial.println(radToDeg(get_Z()));/**/
    delay(20);

  /*for (int i = 0; i < 0; i+=100){ 
    int val = map(i, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(val);    // Send the signal to the ESC
    Serial.println(i);
    delay(500);
  }*/


}

#include <Arduino.h>
#include <Servo.h>
#include "complementary-filter.h"
#include "controller.h"

/*TODO:
  Control altitude? Using a distance sensor and increasing/decreasing the base thrust?
  Calibrate the motors
*/

Servo ESC;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
  
  init_mpu();
  controller_init();
  ///ESC.attach(11,1000,2000);

  
  
  
}

void loop() {
   
    update_filter();
    /*Serial.print("X :");
    Serial.print(radToDeg(get_X()));
    Serial.print("  Y :");
    Serial.print(radToDeg(get_Y()));
    Serial.print("  Z :");
    Serial.println(radToDeg(get_Z()));
    delay(20);*/

  /*for (int i = 0; i < 0; i+=100){ 
    int val = map(i, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
    ESC.write(val);    // Send the signal to the ESC
    Serial.println(i);
    delay(500);
  }*/

  controller_update();
  //ESC.writeMicroseconds(1700);

  delay(20);



}

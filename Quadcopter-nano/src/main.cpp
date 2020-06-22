#include <Arduino.h>
#include "complementary-filter.h"
#include "controller.h"

/*TODO:
  Control altitude? Using a distance sensor and increasing/decreasing the base thrust?
  Calibrate the motors
*/



void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("started");
  
  init_mpu();
  controller_init();

  
  
  
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


  controller_update();


  delay(20); // The filter and controller update seems to take almost no time at all so this should definitliy satisfy h = 0.025



}

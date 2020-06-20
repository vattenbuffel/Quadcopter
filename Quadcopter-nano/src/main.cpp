#include <Arduino.h>

#include <math.h>
#include <I2C.h>
#include "complementary-filter.h"


using namespace TWI;

void setup() {
    Serial.begin(115200);
    I2CSetup(SELF_AD,I2C_FREQ);
    MPU9250Setup();
    timerInterruptSetup();
}

void loop(){
  if(new_readings()){
    reset_new_readings();
    compensate();

    complementary_filter();
    Serial.print("X :");
    Serial.print(radToDeg(get_X()));
    Serial.print("  Y :");
    Serial.print(radToDeg(get_Y()));
    Serial.print("  Z :");
    Serial.println(radToDeg(get_Z()));
  }
}


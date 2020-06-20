#include "complementary-filter.h"
#include "I2C.h"

float X = 0;
float Y = 0;
float Z = 0;

volatile float accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magneX,magneY,magneZ,asax,asay,asaz;
bool updated = false;

using namespace TWI;

float get_X() {return X;}
float get_Y() {return Y;}
float get_Z() {return Z;}
bool new_readings() {return updated;}
void reset_new_readings() {updated = false;}

void complementary_filter(){
  float h = 0.025;
  float alpha = 0.1;
  float gamma = alpha/(h+alpha);

  // There really should be a lock or something on these
  float gyrX = degToRad(gyroX/GYRO_SENS*2)*h;
  float gyrY = degToRad(gyroY/GYRO_SENS*2)*h;
  float gyrZ = degToRad(gyroZ/GYRO_SENS*2)*h;

  // Add previous estimated angles to gyro estimated angle change and multiply with gamma
  gyrX = (gyrX + X) * gamma;
  gyrY = (gyrY + Y) * gamma;
  gyrZ = (gyrZ + Z);

  //Convert acc to rad
  float g = -9.81;
  float accX = accelX*g;///ACCEL_SENS
  float accY = accelY*g;////ACCEL_SENS
  float accZ = accelZ*g;///ACCEL_SENS
  

  float phi = atan2( accY, accZ);
  float theta = atan2( -accX, sqrt( pow(accY, 2) + pow(accZ, 2) ));
  
  phi = phi*(1-gamma);
  theta = theta*(1-gamma);

  // Add the estimations of the gyro and accelerometer
  X = phi + gyrX;
  Y = theta + gyrY;
  Z = gyrZ;
}

void compensate(){
  accelZ = accelZ + 0.18*ACCEL_SENS;
  accelY = accelY - 0.09*ACCEL_SENS;
  accelX = accelX - 0.06*ACCEL_SENS;
  
  float tmp = accelX;
  accelX = accelY*-1;
  accelY = tmp;
  accelZ = accelZ*-1;

  
  gyroX = gyroX - 0.7*GYRO_SENS;
  gyroZ = gyroZ + 0.4*GYRO_SENS;

  tmp = gyroX;
  gyroX = gyroY*1;
  gyroY = -tmp;
  gyroZ = gyroZ*1;
}

ISR(TIMER1_COMPA_vect){
    updateGyroReading();
    updated = true;
}

void MPU9250Setup(){
    startTrans(MPU9250_AD);
    write(PWR_MGMT_1_AD);
    write(0x01,true); //set the clock reference to X axis gyroscope to get a better accuracy

    startTrans(MPU9250_AD);
    write(ACCEL_CONFIG_1_AD);
    write(0x08,true); //set the accel scale to 4g

    startTrans(MPU9250_AD);
    write(ACCEL_CONFIG_2_AD);
    //turn on the internal low-pass filter for accel with 5.05Hz bandwidth
    write(0x05,true);

    startTrans(MPU9250_AD);
    write(GYRO_CONFIG_AD);
    write(0x08,true); //set the gyro scale to 500 °/s and FCHOICE_B

    // turn on the internal low-pass filter for gyro with 10Hz bandwidth
    startTrans(MPU9250_AD);
    write(CONFIG_AD);
    write(0x05,true);
}

void updateGyroReading(){
    //read the accelerate
    startTrans(MPU9250_AD);
    write(ACCEL_XOUT_H_AD);
    requestFrom(MPU9250_AD,6,true);
    accelX = (readBuffer()<<8) | readBuffer();
    accelY = (readBuffer()<<8) | readBuffer();
    accelZ = (readBuffer()<<8) | readBuffer();

    //read the gyro
    startTrans(MPU9250_AD);
    write(GYRO_XOUT_H_AD);
    requestFrom(MPU9250_AD,6,true);
    gyroX = (readBuffer()<<8) | readBuffer();
    gyroY = (readBuffer()<<8) | readBuffer();
    gyroZ = (readBuffer()<<8) | readBuffer();
}

void timerInterruptSetup(){
    cli();  //disable the global interrupt
    //Timer/Counter 1
    TCCR1A = 0x00;
    TCCR1B = (_BV(WGM12)) | (_BV(CS11)) | (_BV(CS10));  //CTC mode, clk/64
    OCR1A = OUTPUT_COMPARE; //set to 200ms
    TCNT1 = 0x00; //initialise the counter
    TIMSK1 = _BV(OCIE1A);  //Output Compare A Match Interrupt Enable
    sei(); //enable global interrupt
}
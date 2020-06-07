#include <Arduino.h>
#include "orientation.h"
#include "MPU9250.h"
#include "MemoryFree.h"
#include <MatrixMath.h>

/*
    TODO:
    Optimise memory usage
    calculate the true offsets using the built in functions
*/

int status;
orientation Orientation;

//MPU9250 IMU(Wire,0x68);


void setup() {
    // serial to display data
    Serial.begin(115200);
    Serial.println(F("Begin initial wait"));
    delay(5000);
    Serial.println(F("End initial wait"));
    
    Serial.print(F("There are "));
    Serial.print(freeMemory());
    Serial.println(F(" bytes of ram left."));
    if(freeMemory() < 700) Serial.println(F("That's probably too little. The program might behave weird."));


    // start communication with IMU
    Serial.println(F("IMU BEGIN"));
    //status = IMU.begin();
    Serial.println(F("IMU BEGUN")); 

    if (status < 0) {
        Serial.println(F("IMU initialization unsuccessful"));
        Serial.println(F("Check IMU wiring or try cycling power"));
        Serial.print(F("Status: "));
        Serial.println(status);
        while(1) {}
    }
    // setting the accelerometer full scale range to +/-8G 
    /*IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    // setting the gyroscope full scale range to +/-500 deg/s
    //IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);
    IMU.setAccelCalZ(1.68,1); // bias of 1.68 m/s^2 and scale of 1
    IMU.setAccelCalY(0.751-0.09,1);
    IMU.setAccelCalX(0.751-0.02,1);*/
    
    while(1){
        Serial.println();
        Orientation.t_prev = millis();
        predict(&Orientation);
        Serial.print(F("There are "));
        Serial.print(freeMemory());
        Serial.println(F(" bytes of ram left after predict."));
        update(&Orientation);
        Serial.print(F("There are "));
        Serial.print(freeMemory());
        Serial.println(F(" bytes of ram left after update."));
        quat_to_euler(&Orientation);
        Serial.print(F("There are "));
        Serial.print(freeMemory());
        Serial.println(F(" bytes of ram left after quat_to_euler."));
        delay(100);
    }


    while(0){
        //Serial.println(F("Starting to update"));
        //IMU.readSensor();

        //double x[4] = {0.9061,    0.1802,   -0.3753,   -0.0747};

        //Orientation.x = x;
        

        /*Orientation.acc[0] = IMU.getAccelX_mss();
        Orientation.acc[1] = IMU.getAccelY_mss();
        Orientation.acc[2] = IMU.getAccelZ_mss();*/
        /*Serial.println();
        Serial.print(F("acc :"));
        Serial.print(Orientation.acc[0],3);
        Serial.print(F("\t"));

        //Serial.print(F("acc_y:"));
        Serial.print(Orientation.acc[1],3);
        Serial.print(F("\t"));
        
        //Serial.print(F("acc_z:"));
        Serial.print(Orientation.acc[2],3);
        Serial.println(F("\t"));*/


        
        /*Orientation.gyr[0] = IMU.getGyroX_rads();
        Orientation.gyr[1] = IMU.getGyroY_rads();
        Orientation.gyr[2] = IMU.getGyroZ_rads();*/
        //Orientation.t_cur = Orientation.t_prev + 500;  
        //Orientation.t_cur = millis();  

        
        //Serial.print(F("Free memory in setup "));
        //Serial.println(freeMemory());
        //update_estimation(&Orientation);
        

        /*Serial.print(F("gyr:"));
        Serial.print(Orientation.gyr[0],3);
        Serial.print(F("\t"));
        Serial.print(Orientation.gyr[1],3);
        Serial.print(F("\t"));
        Serial.print(Orientation.gyr[2],3);
        Serial.println(F("\t"));*/

        //Serial.print("roll:" + String(Orientation.euler_angles[0]) + "   ");
        //Serial.print("pitch:" + String(Orientation.euler_angles[1]) + "   ");
        //Serial.println("yaw:" + String(Orientation.euler_angles[2]));
        /*Serial.print(F("roll:"));
        Serial.print(Orientation.euler_angles[0],2);
        Serial.print(F("\t"));

        Serial.print(F("pitch:"));
        Serial.print(Orientation.euler_angles[1],2);
        Serial.print(F("\t"));
        
        Serial.print(F("yaw:"));
        Serial.print(Orientation.euler_angles[2],2);
        Serial.println(F("\t"));*/
        //while(1){}
        delay(100);
        //Serial.println(F("Done updating. Time to start again"));
    }
    
  
  
}

void loop() {
    
}

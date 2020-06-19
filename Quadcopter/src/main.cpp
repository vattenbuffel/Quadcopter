#include <Arduino.h>
#include "orientation.h"
#include "MemoryFree.h"
#include "MatrixMath.h"
#include "MPU9250.h"

/*
    TODO:
    Optimise memory usage
    calculate the true offsets using the built in functions
    Get interrupt stuff to work
    change every double to float
    find error in calc_K
*/
//MPU9250 IMU(Wire,0x68);
int status;
orientation Orientation;


void setup() {
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
    //IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    // setting the gyroscope full scale range to +/-500 deg/s
    //IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    //IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    //IMU.setSrd(19);// enabling the data ready interrupt
    
    /*IMU.setAccelCalZ(1.68,1); // bias of 1.68 m/s^2 and scale of 1
    IMU.setAccelCalY(0.751-0.09,1);
    IMU.setAccelCalX(0.751-0.02,1);*/

    Orientation.t_prev = millis();
    
}

void loop() {
    
    //IMU.readSensor();
    
    /*Serial.println();
    Serial.print(F("acc :"));
    Serial.print(-IMU.getAccelY_mss(),3);
    Serial.print(F("\t"));
    Serial.print(-IMU.getAccelX_mss(),3);
    Serial.print(F("\t"));
    Serial.print(IMU.getAccelZ_mss(),3);
    Serial.println(F("\t"));*/

    /*double gyr[] = {IMU.getGyroY_rads(), IMU.getGyroX_rads(), -IMU.getGyroZ_rads()};
    Serial.print(F("gyr:"));
    print_matrix(gyr, 1, 3, 5);*/

   /* Orientation.acc[0] = IMU.getAccelX_mss(); 
    Orientation.acc[1] = IMU.getAccelY_mss(); 
    Orientation.acc[2] = IMU.getAccelZ_mss(); //Fucked up order

    Orientation.gyr[0] = IMU.getGyroX_rads(); 
    Orientation.gyr[1] = IMU.getGyroY_rads(); 
    Orientation.gyr[2] = IMU.getGyroZ_rads(); */
    Orientation.t_cur = millis();
    

    /*Orientation.t_cur = 500;
    Orientation.t_prev = 0;
    Orientation.acc[0] = 0; 
    Orientation.acc[1] = 0; 
    Orientation.acc[2] = -9.82; 

    Orientation.gyr[0] = 0; 
    Orientation.gyr[1] = 0.5; 
    Orientation.gyr[2] = 0;
    
    Orientation.x[0] = 1; 
    Orientation.x[1] = 0; 
    Orientation.x[2] = 0;
    Orientation.x[2] = 0;*/

    update_estimation(&Orientation);

    Serial.print(F("Eul_angles: "));
    print_matrix(Orientation.euler_angles, 1, 3, 5);
    Serial.print(F("q: "));
    print_matrix(Orientation.x, 1, 4, 5);
//    print_vector(Orientation.x, 4, 5);

    delay(100);
    
}





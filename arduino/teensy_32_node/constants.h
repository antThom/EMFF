#ifndef CONSTANTS_H
#define CONSTANTS_H
    
    //System Constants
    const float VCC_Reg = 5.0;    //Voltage of the 5V reg
    const float VCC_Teensy = 3.3; //Voltage of the Teensy 3.3V reg
    const int daySec = 24*60*60;  //Amount of seconds in a day
    const float gravity = 9.81;   //Gravity in m/s^2

    //Pin Assignment
    const int h_bridge_1 = 23;    //Digital pin 23
    const int h_bridge_2 = 22;    //Digital pin 22
    const int h_bridge_3 = 21;    //Digital pin 21
    const int h_bridge_4 = 20;    //Digital pin 20
    const int curSens = 1;        //Analog pin 1
    const int imu_LED = 12;       //Digital pin 12
    const int mag_LED = 11;       //Digital pin 11

    //System Variables
    float CS_Vout;
    LIS3MDL mag;
    LSM6 acc_gyr;
    
#endif

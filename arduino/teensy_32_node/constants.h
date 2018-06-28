#ifndef CONSTANTS_H
#define CONSTANTS_H
    
    //System Constants
    ros::NodeHandle  nh;
    const float VCC_Reg = 5.0;    //Voltage of the 5V reg
    const float VCC_Teensy = 3.3; //Voltage of the Teensy 3.3V reg
    const int daySec = 24*60*60;  //Amount of seconds in a day
    const float gravity = 9.81;   //Gravity in m/s^2
    const float accGain = 0.061;  //mg/LSB from dataSheet
    const float GRAVITY = 256.0;  //BYTES
    const float gyroGain = 70;    //mdps/LSB from dataSheet
    const float magGain = 6842.0; //LSB/gauss
    
    //Pin Assignment
    const int curSens1 = 0;        //Analog pin 1
    const int curSens2 = 1;        //Analog pin 2
    const int imu_LED = 12;        //Digital pin 12
    const int mag_LED = 11;        //Digital pin 11
    const int LED =10;             //Good status LED Digital pin 10

    //System Variables
    float CS_Vout;
    int dataSet = 1;
    float yaw;
    float pitch;
    float roll;
    float time_0, time_1, dt;
    LIS3MDL mag;
    LSM6 acc_gyr;

    //H-Bridge structures
    struct Coil 
    {
      const char ID[4];
      const int pin[4];
      int state[4];
    }; 
    //                   ID                PIN #          STATES
    Coil coil_1 = { {'a','b','c','d'} , {23,22,21,20} , {0,0,0,0} };
    Coil coil_2 = { {'a','b','c','d'} , {3,4,5,6} , {0,0,0,0} };

    
    
#endif

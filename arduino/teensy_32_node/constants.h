#ifndef CONSTANTS_H
#define CONSTANTS_H
    
    //System Constants
    ros::NodeHandle  nh;
    const float VCC_Reg = 5.0;    //Voltage of the 5V reg
    const float VCC_Teensy = 3.3; //Voltage of the Teensy 3.3V reg
    const int daySec = 24*60*60;  //Amount of seconds in a day
    const float gravity = 9.81;   //Gravity in m/s^2
    //const int GRAVITY = 256;      //this equivalent to 1G in the raw data coming from the accelerometer
    //const int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
    
    //Pin Assignment
    const int curSens1 = 1;        //Analog pin 1
    const int curSens2 = 2;        //Analog pin 2
    const int imu_LED = 12;       //Digital pin 12
    const int mag_LED = 11;       //Digital pin 11

    //System Variables
    float CS_Vout;
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
    

    /*
    // Euler angles
    float roll;
    float pitch;
    float yaw;

    float errorRollPitch[3]= {0,0,0};
    float errorYaw[3]= {0,0,0};

    unsigned int counter=0;
    byte gyro_sat=0;

    float DCM_Matrix[3][3]= {{1,0,0},{0,1,0},{0,0,1}};
    float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here

    float Temporary_Matrix[3][3]={{0,0,0},{0,0,0},{0,0,0}};
    */
#endif

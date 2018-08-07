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
    const float pi = 3.141592654; //pi
    float ohm = 0.5;              //resistance
    float Vbat = 11.1;            //Voltage of the Battery
    float N = 25.0;               //# of loops
    float R = 0.156775;           //Radius of coil meters
    float A = pi*R*R;             //Area of coil meters^2
    int timeON = 30;            //Amount of time the coils will be on (seconds)
    float GDC = 0.50;             //Global duty cycle of the coils (coil ON / coil off)
    float timeOFF = timeON/GDC;   //Amount of time the coils will be off (seconds)
    
    //Pin Assignment
    const int curSens1 = 0;        //Analog pin 0
    const int curSens2 = 1;        //Analog pin 1
    const int imu_LED = 12;        //Digital pin 12
    const int mag_LED = 11;        //Digital pin 11
    const int LED = 10;            //Good status LED Digital pin 10
    const int coil_11_dir = 23;      //Direction of coil_1
    const int coil_12_dir = 20;      //Direction of coil_1
    const int coil_21_dir = 3;      //Direction of coil_1
    const int coil_22_dir = 6;      //Direction of coil_2
    const int coil_11_pwm = 22;     //DutyCycle of coil_1
    const int coil_12_pwm = 21;     //DutyCycle of coil_2
    const int coil_21_pwm = 4;     //DutyCycle of coil_1
    const int coil_22_pwm = 5 ;     //DutyCycle of coil_2

    //System Variables
    float CS_Vout;
    int dataSet = 1;
    float yaw;
    float pitch;
    float roll;
    float DC_x;
    float DC_y;
    float time_0, time_1, dt, coilTime_0, coilTime_1, coil_dt;
    LIS3MDL mag;
    LSM6 acc_gyr;

    
#endif

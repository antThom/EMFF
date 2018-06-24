#ifndef VEHICLE_FUNC_H
#define VEHICLE_FUNC_H
#include "Arduino.h"
#include <ros.h>
#include <geometry_msgs/Twist.h> //Add the message types
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>  
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <LIS3MDL.h>
#include <LSM6.h>
#include <Wire.h>

/*
	This header file contains the functions needed for the coils and H-Bridges
*/

void STOP_COIL_Current(int coilNum) //Stop the current in the coil according to the coilNum sent from the user
#endif /* VEHICLE_FUNC_H */
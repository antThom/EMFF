#include "vehicle_func.h"
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

void STOP_COIL_Current(int coilNum)
{ 
  if(coilNum==1){
    digitalWrite(coil_1.pin[0],HIGH); 
    digitalWrite(coil_1.pin[1],HIGH);
    digitalWrite(coil_1.pin[2],LOW);
    digitalWrite(coil_1.pin[3],LOW); 
  }
  else if(coilNum==2){
    digitalWrite(coil_2.pin[0],HIGH); 
    digitalWrite(coil_2.pin[1],HIGH);
    digitalWrite(coil_2.pin[2],LOW);
    digitalWrite(coil_2.pin[3],LOW);
  }
  else {
    digitalWrite(coil_1.pin[0],HIGH); 
    digitalWrite(coil_1.pin[1],HIGH);
    digitalWrite(coil_1.pin[2],LOW);
    digitalWrite(coil_1.pin[3],LOW);
    
    digitalWrite(coil_2.pin[0],HIGH); 
    digitalWrite(coil_2.pin[1],HIGH);
    digitalWrite(coil_2.pin[2],LOW);
    digitalWrite(coil_2.pin[3],LOW);
  }
}
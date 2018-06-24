/*
 * This code should be uploaded onto a Teensy 3.2
 * This code should recieve incoming messages from
 * the odroid which got the message from the ground 
 * station. The objective of this code is to:
 *  1. Collect IMU data
 *  2. Collect current data
 *  3. Collect temperature data
 *  4, Operate the H-Bridge to power the coil
 *  5. Communicate with the Odroid via. ROS
 *  
 */

 #include <ros.h>
 #include <geometry_msgs/Twist.h> //Add the message types
 #include <geometry_msgs/Pose.h>
 #include <geometry_msgs/Vector3.h>  
 #include <std_msgs/Float32.h>
 #include <sensor_msgs/Imu.h>
 #include <LIS3MDL.h>
 #include <LSM6.h>
 #include <Wire.h>
 
 #include "constants.h"
 //#include "vehicle_func.h"

 
///////////////////////{ GLOBALS }////////////////////////////////////////
// ros::NodeHandle  nh;
///////////////////////////////////////////////////////////////////////////


//Callback Functions for COMMANDS coming from the groundStation

 void velocityCallback( const geometry_msgs::Twist& velocityCommand)
 {
     digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }

 void positionCallback( const geometry_msgs::Pose& positionCommand)
 {
     digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }

 void magnetCallback( const geometry_msgs::Vector3& magnetCommand)
 {
    /* When we get magnet commands we shoud compare them to our current magnet status and 
     * determine whether or not we are going to:
     *  1. Shut of Current
     *  2. Reverse Current 
     *  3. Forward Current
     * Note that since we are always applying the max amounts of amps, we only have these 3 options
     */
     
     // Saturation
     float mag_X=round(saturation(magnetCommand.x));
     float mag_Y=round(saturation(magnetCommand.y));
     float mag_Z=round(saturation(magnetCommand.z));
     
     //Turn Off Coil
     if (mag_X==0 && mag_Y==0 && mag_Z==0){
        STOP_COIL_Current(3);
     }
     //Reverse Current
    digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }

///////////////////////{ SUBSCRIBE }////////////////////////////////////////
// Initialized the Subscriber (must know the msg type and the topic that it is subscribing to
 ros::Subscriber<geometry_msgs::Pose> pos("POSITION", &positionCallback ); //Topic == POSITION && Node == pos
 ros::Subscriber<geometry_msgs::Twist> velocity("VELOCITY", &velocityCallback ); //Topic == VELOCITY && Node == velocity
 ros::Subscriber<geometry_msgs::Vector3> magnet("MAGNET", &magnetCallback );  //Topic == MAGNET && Node == magnet
///////////////////////////////////////////////////////////////////////////

///////////////////////{ PUBLISH }////////////////////////////////////////
geometry_msgs::Vector3 magnetApplied;
std_msgs::Float32 current;
sensor_msgs::Imu IMU;

ros::Publisher magnet_applied("MAGNET_APPLIED", &magnetApplied);
ros::Publisher current_applied("CURRENT_APPLIED", &current);
ros::Publisher imu("IMU", &IMU);
///////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT); //This the onboard LED
  nh.initNode();
  nh.subscribe(pos);
  nh.subscribe(velocity);
  nh.subscribe(magnet);

  nh.advertise(magnet_applied);
  nh.advertise(current_applied);
  nh.advertise(imu);

  Wire.begin(); //Initialize I2C

  // Make sure the sensors get initialized 
  if (!acc_gyr.init())
  {
    //Turn on imu_LED
    digitalWrite(imu_LED,HIGH);
    while(1);
  }
  acc_gyr.enableDefault();
  //acc_gyr.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
  
  if (!mag.init())
  {
    //Turn on mag_LED
    digitalWrite(mag_LED,HIGH);
    while(1);
  }
  mag.enableDefault();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Read IMU Data
  mag.read();         //get data from the magnetometer
  acc_gyr.read();     //get data from the accelerometer and gyro

  //Assign values to the magnet
  magnetApplied.x=mag.m.x; 
  magnetApplied.y=mag.m.y; 
  magnetApplied.z=mag.m.z;

  //Assign values to the accelerometer
  IMU.linear_acceleration.x=acc_gyr.a.x; 
  IMU.linear_acceleration.y=acc_gyr.a.y; 
  IMU.linear_acceleration.z=acc_gyr.a.z;  

  //Assign values to the gyro
  IMU.angular_velocity.x=acc_gyr.g.x; 
  IMU.angular_velocity.y=acc_gyr.g.x; 
  IMU.angular_velocity.z=acc_gyr.g.x;

  //Assign values to the orientation
  IMU.orientation.x=0.0; //We're getting data from MOCAP so I ignored these values
  IMU.orientation.y=0.0; 
  IMU.orientation.z=0.0; 
  IMU.orientation.w=0.0;
  
  IMU.header.frame_id = "IMU"; // Give the Sensor an ID
  IMU.header.stamp = nh.now(); // This is NEEDED TO GET A TIME STEP

  //Assign values to the Current Sensor
  current.data = currentSensor(analogRead(curSens));
 
  
  /*//////////////////////{ SAMPLE CODE, NEEDS TO BE DELETED } //////////////////////////////
  current.data = 1.0;
  magnetApplied.x=1.0; magnetApplied.y=2.0; magnetApplied.z=3.0;
  IMU.orientation.x=1.0; IMU.orientation.y=1.0; IMU.orientation.z=1.0; IMU.orientation.w=1.0;
  IMU.angular_velocity.x=0.0; IMU.angular_velocity.y=0.0; IMU.angular_velocity.z=0.0;
  IMU.linear_acceleration.x=4.0; IMU.linear_acceleration.y=4.0; IMU.linear_acceleration.z=4.0;
  *//////////////////////////////////////////////////////////////////////////////////////////

  
  // Publish the applied values to their topics
  magnet_applied.publish( &magnetApplied );
  current_applied.publish( &current );
  imu.publish( &IMU );
  
  nh.spinOnce();
  delay(100);
}

float currentSensor(float CS_Vout)
{
  current.data = 73.3*(CS_Vout/VCC_Reg)-36.7;
  return current.data;
}

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

float saturation(float mag)
{
  if (mag>1){
    mag=1;
  }
  else if (mag<-1){
    mag=-1;
  }
  return mag;
}


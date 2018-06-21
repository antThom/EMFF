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
 #include "constants.h"
 
///////////////////////{ GLOBALS }////////////////////////////////////////
 ros::NodeHandle  nh;
///////////////////////////////////////////////////////////////////////////


//Callback Functions for commands coming from the groundStation

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

}

void loop() {
  // put your main code here, to run repeatedly:

  ///////////////////////{ SAMPLE CODE, NEEDS TO BE DELETED } //////////////////////////////
  current.data = 1.0;
  magnetApplied.x=1.0; magnetApplied.y=2.0; magnetApplied.z=3.0;
  IMU.header.frame_id = "IMU"; 
  IMU.header.stamp = nh.now(); // This is NEEDED TO GET A TIME STEP
  IMU.orientation.x=1.0; IMU.orientation.y=1.0; IMU.orientation.z=1.0; IMU.orientation.w=1.0;
  IMU.angular_velocity.x=0.0; IMU.angular_velocity.y=0.0; IMU.angular_velocity.z=0.0;
  IMU.linear_acceleration.x=4.0; IMU.linear_acceleration.y=4.0; IMU.linear_acceleration.z=4.0;
  //////////////////////////////////////////////////////////////////////////////////////////
  
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


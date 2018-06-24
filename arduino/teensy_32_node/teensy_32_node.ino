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



//Callback Functions for COMMANDS coming from the groundStation
/*
 void velocityCallback( const geometry_msgs::Twist& velocityCommand)
 {
     digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }
*/
/*
 void positionCallback( const geometry_msgs::Pose& positionCommand)
 {
   // When we get a position and orientation command, we will use it as a reference and 
    * activate teh magnet controller. The magnet Controller will then control the vehicles
    * magnetic field and move the vehicle to the new position and orientataion. 
    * 
    * The POSITION AND ORIENTATION commands are relative to other vehicles. The other vehicle's
    * position and orientation is selected in MATLAB and the analysis is also done in MATLAB.
    * 
    * The postion data just gets saved into a ros data bag
   //
     digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }
*/

 void magnetCallback( const geometry_msgs::Vector3& magnetCommand)
 {
    /* When we get magnet commands we shoud compare them to our current magnet status and 
     * determine whether or not we are going to:
     *  1. Shut off Current
     *  2. Reverse Current 
     *  3. Forward Current
     * Note that since we are always applying the max amounts of amps, we only have these 3 options
     * 
     * Operation Sequence
     *  1. Get command, apply saturation, round to the nearest whole number
     *  2. Open all of the MOSFETS (Only here for safety)
     *  3. Apply the current to the coils
     *  4. Save data in ros bag
     *  
     *  Note: MOSFETS a & c , b & d CANNOT be closed at the same time {BAD!!!!!!!!!!!!!!!}
     */
     
     // Saturation and rounding... The output should be either a 1 or 0 {mag_Z is ignored for all 2D functions}
     float mag_X=round(saturation(magnetCommand.x));
     float mag_Y=round(saturation(magnetCommand.y));
     float mag_Z=round(saturation(magnetCommand.z));

     // Turn off all of the MOSFETS
     STOP_COIL_Current(0); //Floating
     delay(5);
     // Operate the H-Bridge

     // COIL #1
     if (mag_X==1){ 
        FWD_COIL_Current(1);
     }
     else if (mag_X==-1){ 
        RWD_COIL_Current(1);
     }
     else if (mag_X==0){  
        STOP_COIL_Current(1);
     }
     
     // COIL #2
     if (mag_Y==1){  
        FWD_COIL_Current(2);
     }
     else if (mag_Y==-1){  
        RWD_COIL_Current(2);
     }
     else if (mag_Y==0){  
        STOP_COIL_Current(2);
     }
     
     // Read pin States
     pinState();
 }

///////////////////////{ SUBSCRIBE }////////////////////////////////////////
// Initialized the Subscriber (must know the msg type and the topic that it is subscribing to
 //  ros::Subscriber<geometry_msgs::Pose> pos("POSITION", &positionCallback ); //Topic == POSITION && Node == pos
 //  ros::Subscriber<geometry_msgs::Twist> velocity("VELOCITY", &velocityCallback ); //Topic == VELOCITY && Node == velocity
 ros::Subscriber<geometry_msgs::Vector3> magnet("MAGNET", &magnetCallback );  //Topic == MAGNET && Node == magnet
///////////////////////////////////////////////////////////////////////////

///////////////////////{ PUBLISH }////////////////////////////////////////
geometry_msgs::Vector3 magnetApplied;
std_msgs::Float32 current1;
std_msgs::Float32 current2;
sensor_msgs::Imu IMU;

ros::Publisher magnet_applied("MAGNET_APPLIED", &magnetApplied);
ros::Publisher current_applied_1("CURRENT_APPLIED_1", &current1);
ros::Publisher current_applied_2("CURRENT_APPLIED_2", &current2);
ros::Publisher imu("IMU", &IMU);
///////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT); //This the onboard LED
  nh.initNode();
  //  nh.subscribe(pos);
  //  nh.subscribe(velocity);
  nh.subscribe(magnet);

  nh.advertise(magnet_applied);
  nh.advertise(current_applied_1);
  nh.advertise(current_applied_2);
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
  current1.data = currentSensor(analogRead(curSens1));
  current2.data = currentSensor(analogRead(curSens2));
 
  
  /*//////////////////////{ SAMPLE CODE, NEEDS TO BE DELETED } //////////////////////////////
  current.data = 1.0;
  magnetApplied.x=1.0; magnetApplied.y=2.0; magnetApplied.z=3.0;
  IMU.orientation.x=1.0; IMU.orientation.y=1.0; IMU.orientation.z=1.0; IMU.orientation.w=1.0;
  IMU.angular_velocity.x=0.0; IMU.angular_velocity.y=0.0; IMU.angular_velocity.z=0.0;
  IMU.linear_acceleration.x=4.0; IMU.linear_acceleration.y=4.0; IMU.linear_acceleration.z=4.0;
  *//////////////////////////////////////////////////////////////////////////////////////////

  
  // Publish the applied values to their topics
  magnet_applied.publish( &magnetApplied );
  current_applied_1.publish( &current1 );
  current_applied_2.publish( &current2 );
  imu.publish( &IMU );
  
  nh.spinOnce();
  delay(10);
}





///////////////////////////////{ USER DEFINED FUNCTIONS }///////////////////////////////////////  

float currentSensor(float CS_Vout)
{
  float c = 73.3*(CS_Vout/VCC_Reg)-36.7;
  return c;
}

void FWD_COIL_Current(int coilNum)
  { 
  if(coilNum==1){
    digitalWrite(coil_1.pin[0],LOW);
    digitalWrite(coil_1.pin[1],HIGH);
    digitalWrite(coil_1.pin[2],LOW);
    digitalWrite(coil_1.pin[3],HIGH);
  }
  else if(coilNum==2){
    digitalWrite(coil_2.pin[0],LOW);
    digitalWrite(coil_2.pin[1],HIGH);
    digitalWrite(coil_2.pin[2],LOW);
    digitalWrite(coil_2.pin[3],HIGH);
  }
}

void RWD_COIL_Current(int coilNum)
{
  if(coilNum==1){
    digitalWrite(coil_1.pin[0],HIGH);
    digitalWrite(coil_1.pin[1],LOW);
    digitalWrite(coil_1.pin[2],HIGH);
    digitalWrite(coil_1.pin[3],LOW);
  }
  else if(coilNum==2){
    digitalWrite(coil_2.pin[0],HIGH);
    digitalWrite(coil_2.pin[1],LOW);
    digitalWrite(coil_2.pin[2],HIGH);
    digitalWrite(coil_2.pin[3],LOW);
  }
}

void STOP_COIL_Current(int coilNum)
{ 
  // Open all the MOSFETS
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
  //Apply a saturation to the desired magnet direction
  if (mag>1){
    mag=1;
  }
  else if (mag<-1){
    mag=-1;
  }
  return mag;
}

void pinState()
{
  //Read all of the states for the H-Bridge pins
  coil_1.state[0] = digitalRead(coil_1.pin[0]);
  coil_1.state[1] = digitalRead(coil_1.pin[1]);
  coil_1.state[2] = digitalRead(coil_1.pin[2]);
  coil_1.state[3] = digitalRead(coil_1.pin[3]);

  coil_2.state[0] = digitalRead(coil_2.pin[0]);
  coil_2.state[1] = digitalRead(coil_2.pin[1]);
  coil_2.state[2] = digitalRead(coil_2.pin[2]);
  coil_2.state[3] = digitalRead(coil_2.pin[3]);
}

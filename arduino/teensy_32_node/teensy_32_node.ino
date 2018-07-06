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
 #include <math.h>
 
 #include "constants.h"



//Callback Functions for COMMANDS coming from the groundStation
/*
 void velocityCallback( const geometry_msgs::Twist& velocityCommand)
 {
     digitalWrite(13, HIGH-digitalRead(13));  //Blink on board LED when a message is recieved 
 }
*/

 void positionCallback( const geometry_msgs::Pose& positionCommand)
 {
   /* The incoming position command will be sent from MOCAP and should only be used as an 
    * initial condition for intergration. For now I think we are mostly concerned with the
    * attitude and attitude rates
    *  1. Get orientation data
    *  2. Convert to euler angles
    *  3. Save the first set of data and save it as an Initial Condition
    *  4. Ignore the rest of incoming data
    */

    if (dataSet==1)
    {
      float x = positionCommand.orientation.x;
      float y = positionCommand.orientation.y;
      float z = positionCommand.orientation.z;
      float w = positionCommand.orientation.w;

      //Convert Quaternion to euler angles and these will be the initail conditions
      yaw = atan2(2*(x*y+w*z),(w*w-z*z-y*y+x*x)); // This is the most important one because our system shuldn't be changing it pitch or roll at all 
      pitch = asin(-2*(x*z-y*w));
      roll = atan2(2*(z*y+x*w),(w*w+z*z-y*y-x*x));
      dataSet++;
    }
    digitalWrite(LED, HIGH-digitalRead(LED));   // blink the led
 }


 void magnetCallback( const geometry_msgs::Vector3& magnetCommand)
 {
    /* When we get magnet commands we need to convert that to a PWM dutycyle and calculate the magnetic
     * dipole moment (mu) and magnetic field (B).To do the conversions and calculation we need to first 
     * state some system parameters.
     *  A = Area of Coil
     *  N = Number of loops in the coil
     *  L = Length of Coil (2*pi*R)
     *  R = Radius of the Coil
     *  Vbat = Voltage of the battery
     *  ohm = Resistance of the coil
     * 
     * Operation Sequence
     *  1. Get command, apply saturation, and determine direction
     *  2. Convert command into a dutycycle
     *  3. Calculate mu and B
     *  4. Send dutycycle and direction to the H-Bridge controller.
     */

     float Imax = 20;   //amps  
     float maxMag = N*Imax*A;
     
     // Saturation {mag_Z is ignored for all 2D functions}
     float mag_X=(saturation(magnetCommand.x, maxMag));
     float mag_Y=(saturation(magnetCommand.y, maxMag));
     // float mag_Z=round(saturation(magnetCommand.z));

     // Assign the direction of the current
     if (mag_X < 0)
        digitalWrite(coil_1_dir,LOW); 
     else
        digitalWrite(coil_1_dir,HIGH);

     if (mag_Y < 0)
        digitalWrite(coil_2_dir,LOW); 
     else
        digitalWrite(coil_2_dir,HIGH);

     //Convert the command into a dutycyle
     DC_x = (mag_X*ohm)/(Vbat*A*N); //Domain = [0,1]
     DC_y = (mag_Y*ohm)/(Vbat*A*N); //Domain = [0,1]

     DC_x = map(DC_x,0,1,0,255); //new Domain = [0,255] for pwm
     DC_y = map(DC_y,0,1,0,255);

     //Apply the DutyCyle to the H-Bridge Controller
     analogWrite(coil_1_pwm,DC_x);
     analogWrite(coil_2_pwm,DC_y);
     
 }

///////////////////////{ SUBSCRIBE }////////////////////////////////////////
// Initialized the Subscriber (must know the msg type and the topic that it is subscribing to
 ros::Subscriber<geometry_msgs::Pose> pos("POSITION", &positionCallback ); //Topic == POSITION && Node == pos
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
  pinMode(LED,OUTPUT);
  pinMode(imu_LED,OUTPUT);
  pinMode(mag_LED,OUTPUT);
  
  digitalWrite(13,HIGH); //Turn on initialization LED
  nh.initNode();
  nh.subscribe(pos);
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
  acc_gyr.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
  acc_gyr.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
  
  if (!mag.init())
  {
    //Turn on mag_LED
    digitalWrite(mag_LED,HIGH);
    while(1);
  }
  mag.enableDefault();
  delay(1000);
  analogWrite(LED,127);//Turn on good status LED
  digitalWrite(13,LOW); //Turn off initialization LED
  time_0=millis(); //Initial Time
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //Read IMU Data
  mag.read();         //get data from the magnetometer
  acc_gyr.readAcc();  //get data from the accelerometer
  
  acc_gyr.readGyro();     //get data from the accelerometer and gyro
  time_1=millis();    //time of sensor reading
  dt=time_1-time_0;   //Time difference
  time_0=time_1;      //Set new initial time for next integration
  
  //Assign values to the magnet
  magnetApplied.x=mag.m.x / magGain; 
  magnetApplied.y=mag.m.y / magGain; 
  magnetApplied.z=mag.m.z / magGain;


  IMU.header.frame_id = "IMU"; // Give the Sensor an ID
  IMU.header.stamp = nh.now(); // This is NEEDED TO GET A TIME STEP

  //Assign values to the accelerometer
  IMU.linear_acceleration.x=acc_gyr.a.x * accGain/GRAVITY*gravity; 
  IMU.linear_acceleration.y=acc_gyr.a.y * accGain/GRAVITY*gravity; 
  IMU.linear_acceleration.z=acc_gyr.a.z * accGain/GRAVITY*gravity;  

  //Assign values to the gyro
  IMU.angular_velocity.x=acc_gyr.g.x / gyroGain; 
  IMU.angular_velocity.y=acc_gyr.g.y / gyroGain; 
  IMU.angular_velocity.z=acc_gyr.g.z / gyroGain;

  //Assign values to the orientation
  float yawRate=acc_gyr.g.z;
  yaw=integrator(yawRate);  //Not being published as of yet
  
  IMU.orientation.x=0.0; //We're getting data from MOCAP so I ignored these values
  IMU.orientation.y=0.0; 
  IMU.orientation.z=0.0; 
  IMU.orientation.w=0.0;
  

  //Assign values to the Current Sensor
  current1.data = currentSensor(analogRead(curSens1));
  current2.data = currentSensor(analogRead(curSens2));
  
  // Once we have the current we can calculate the dipole moment (mu) and the Magnetic Field (B)
  float mu_x = current1.data*N*A; 
  float mu_y = current2.data*N*A; 

  
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
  CS_Vout = map(CS_Vout,0,1023,0,3.3);
  float c = (CS_Vout-0.5*VCC_Teensy)/(0.185);
  //float c = 73.3*(CS_Vout/VCC_Teensy)-36.7;
  return c;
}

float saturation(float mag, float maxMag) 
{
  //Apply a saturation to the desired magnet direction
  if (mag>maxMag){
    mag=maxMag;
  }
  else if (mag<-maxMag){
    mag=-maxMag;
  }
  return mag;
}

float integrator(float yawRate)
{
  yaw = yaw + yawRate*dt;
  return yaw;
}



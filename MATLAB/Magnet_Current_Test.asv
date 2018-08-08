clear all; close all; clc;
%% Magnet to Current Code INTRO

% This code is used to measure the magnetic field as seen be the satellite
% as we pump current through that satellite's coils. Once creating a single
% magnetic field, the satellite will naturally align with the earth's
% magnetic field. We will also be measuring the data from the IMU to
% capture this motion.

%% RosNetwork Setup
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.1.31:11311');
setenv('ROS_IP','192.168.1.26');
rosinit;
pause(2);

% Publishers
[mag_pub, mag_msg]=rospublisher('/MAGNET','geometry_msgs/Vector3');

% Subscribers
magApplied=rossubscriber('/MAGNET_APPLIED','geometry_msgs/Vector3');
receive(magApplied,10);
currApplied1=rossubscriber('/CURRENT_APPLIED_1','std_msgs/Float32');
receive(currApplied1,10);
currApplied2=rossubscriber('/CURRENT_APPLIED_2','std_msgs/Float32');
receive(currApplied2,10);
imu=rossubscriber('/IMU','sensor_msgs/Imu');
receive(imu,10);
%% Initialize Variables
wait=10e-3;
exTime=5*60;
s = exTime / wait;
acc = NaN*zeros(s,3); % Accelerometer data for each vehicle by row
gyro = NaN*zeros(s,3); % Gyro data for each vehicle by row
magnet = NaN*zeros(s,3); % Magnetometer for each vehicle by row
current1 = NaN*zeros(s,1); % Current data for each vehicle by row and coil by column
current2 = NaN*zeros(s,1); % Current data for each vehicle by row and coil by column
loops = 25; % The number of loops of each coil
area=(pi/4)*0.3236; % Area of each coil

%% Send Commands

% Note that the magnetic field commands get convert into current through
% the coil commands before being sent over to the teensy.

u=[10,pi/4]; %this is the polar input

% Convert the polar dipole commands into current commands for all
% satellites
mag_msg.X = (u(1)*cos(u(2)))/(area*loops); %Coil #1
mag_msg.Y = (u(1)*sin(u(2)))/(area*loops); %Coil #2
mag_msg.Z = 0;

% Current Saturation
if mag_msg.X > 20
    mag_msg.X = 20;
end

if mag_msg.Y >20
    mag_msg.Y = 20;
end

send(mag_pub,mag_msg); % Send the command to the teensy

%% Data Collection
for n=1:s
    acc(s)=imu.LatestMessage.LinearAcceleration.X;
    acc(s)=imu.LatestMessage.LinearAcceleration.Y;
    acc(s)=imu.LatestMessage.LinearAcceleration.Z;
    
    gyro(s)=imu.LatestMessage.AngularVelocity.X;
    gyro(s)=imu.LatestMessage.AngularVelocity.Y;
    gyro(s)=imu.LatestMessage.AngularVelocity.Z;
    
    magnet(s)=magApplied.LatestMessage.X;
    magnet(s)=magApplied.LatestMessage.Y;
    magnet(s)=magApplied.LatestMessage.Z;
    
    current1(s)=currApplied1.LatestMessage.Data;
    current2(s)=currApplied2.LatestMessage.Data;
    
    pause(wait);
end
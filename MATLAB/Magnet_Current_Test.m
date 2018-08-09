clear all; close all; clc;
%% Magnet to Current Code INTRO

% This code is used to measure the magnetic field as seen be the satellite
% as we pump current through that satellite's coils. Once creating a single
% magnetic field, the satellite will naturally align with the earth's
% magnetic field. We will also be measuring the data from the IMU to
% capture this motion.

%% RosNetwork Setup
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.1.32:11311');
setenv('ROS_IP','192.168.1.22');
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
exTime=0.1*60;
s = exTime / wait;
time = (0:wait:exTime)';
time(end)=[];

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
    acc(n,1)=imu.LatestMessage.LinearAcceleration.X;
    acc(n,2)=imu.LatestMessage.LinearAcceleration.Y;
    acc(n,3)=imu.LatestMessage.LinearAcceleration.Z;
    
    gyro(n,1)=imu.LatestMessage.AngularVelocity.X;
    gyro(n,2)=imu.LatestMessage.AngularVelocity.Y;
    gyro(n,3)=imu.LatestMessage.AngularVelocity.Z;
    
    magnet(n,1)=magApplied.LatestMessage.X;
    magnet(n,2)=magApplied.LatestMessage.Y;
    magnet(n,3)=magApplied.LatestMessage.Z;
    
    current1(n)=currApplied1.LatestMessage.Data;
    current2(n)=currApplied2.LatestMessage.Data;
    
    pause(wait);
end

%% Plotting
figure(1)
plot(time,acc(:,1)); hold on; grid on; plot(time,acc(:,2)); plot(time,acc(:,3)); 
xlabel('Time (sec)'); ylabel('Acceleration (m/s^2)'); 
title('Satellite Linear Acceleration v. Time');
legend('X','Y','Z')

figure(2)
plot(time,gyro(:,1)); hold on; grid on; plot(time,gyro(:,2)); plot(time,gyro(:,3)); 
xlabel('Time (sec)'); ylabel('Angular Velocity (deg/s)'); 
title('Satellite Angular Velocity v. Time');
legend('X','Y','Z')

figure(3)
plot(time,magnet(:,1)); hold on; grid on; plot(time,magnet(:,2)); plot(time,magnet(:,3)); 
xlabel('Time (sec)'); ylabel('Magnetic Field Strength (Guass)'); 
title('Satellite Magnetic Field Strength v. Time');
legend('X','Y','Z')

figure(4)
plot(time,current1); hold on; grid on; plot(time,current2);
xlabel('Time (sec)'); ylabel('Current (Amps)'); 
title('Satellite Coil Current v. Time');
legend('Coil_1','Coil_2')

figure(5)
plot(current1,magnet(:,1),'.'); hold on; grid on;
xlabel('Current (Amps)'); ylabel('Magnetic Field Strength (Guass)'); 
title('Satellite Magnetic Field Strength v. Satellite Coil Current');

figure(6)
plot(current2,magnet(:,2),'.'); hold on; grid on;
xlabel('Current (Amps)'); ylabel('Magnetic Field Strength (Guass)'); 
title('Satellite Magnetic Field Strength v. Satellite Coil Current');
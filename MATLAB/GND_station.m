clear all; close all; clc;
%% Program description
% This script file will set up the connection between the mocap software
% and matlab, apply an observer to eliminate some noise, process other
% incoming data from sensors, connect to a ROS network, and apply a control 
% law 

% NOTE!!!!! THIS CODE ONLY WORKS WHEN WE HAVE MORE THAN ONE RIGID BODIES IN
% MOTIVE

%% Setup Ros Connections
rosshutdown; %Shutdown any Ros connection before initializing a new one
setenv('ROS_MASTER_URI','http://192.168.1.32:11311'); % set the ros master to a specific URI
setenv('ROS_IP','192.168.1.22'); % set the ros ip address for this computer
rosinit %Initialize a new rosnode
% rosinit('192.168.1.24','NodeHost','192.168.1.22') %Initialize a new rosnode
pause(2);   %Give matlab some time to setup the node

%% User defined Parameters
prompt='Number of satellites to track? ';
Np=input(prompt); % # of vehicles that we are tracking at one time

% Vehicle ID specified in Motive
vIDs = 1:Np;
%% Setup Publishers

% We will be sending magnet dipole commands for the ground station to each
% vehicle. This means that we must have the same number of magnet commands
% as we do vehicles. In the cell created below each row corresponds to a
% vehicle

magCmd=cell(Np,2);
for i=1:Np
    [magCmd{i,1}, magCmd{i,2}]=rospublisher('/MAGNET','geometry_msgs/Vector3');
end

% We are not sending any position or velocity data to the vehicles
%[lin_pos_pub,lin_pos_msg] = rospublisher('LIN_POSITION_NOW','geometry_msgs/Vector3');
%[lin_vel_pub,lin_vel_msg] = rospublisher('LIN_VELOCITY_NOW','geometry_msgs/Vector3');
%[ang_pos_pub,ang_pos_msg] = rospublisher('ANG_POSITION_NOW','geometry_msgs/Vector3');
%[ang_vel_pub,ang_vel_msg] = rospublisher('ANG_POSITION_NOW','geometry_msgs/Vector3');

%% Setup Subscribers
% Each vehicle will send this ground station its respective sensor data. So
% we need to create a set of subscribers for each vehicle. I will use a
% cell to do this. Each row in the cell corresponds to a vehicle.

magApplied=cell(Np,1); % This is for the magnetometer data coming from the vehicle (Each row contains [x,y,z])
currApplied=cell(Np,2); % This is for the current sensor data applied to the vehicle's coils (2 per vehicle)
imu=cell(Np,1); % This is for vehicle IMU data.

% Initialize all the Subscribers
for i=1:Np
   magApplied{i,1}=rossubscriber('/MAGNET_APPLIED','geometry_msgs/Vector3');
   receive(magApplied{i,1},10); % Recieve a msg on this topic to ensure it is connected to the network (WAITS FOR 10 SECONDS TO RECIEVE)
   currApplied{i,1}=rossubscriber('/CURRENT_APPLIED_1','std_msgs/Float32');
   currApplied{i,2}=rossubscriber('/CURRENT_APPLIED_2','std_msgs/Float32');
   receive(currApplied{i,1},10);
   receive(currApplied{i,2},10);
   imu{i,1}=rossubscriber('/IMU','sensor_msgs/Imu');
   receive(imu{i,1},10); 
end
%%  Initialize: Motion Capture Interface
% Add NatNet library and java jar files
usr1= getenv('USERNAME');
if strcmp(usr1,'CDCL')
    Opti=NET.addAssembly('C:\Users\CDCL\Documents\newMotionCaptureCode\AnthonyREU2018\Quad_experiments\mocap_toolbox\NatNetSDK\lib\x64\NatNetML.dll');
elseif strcmp(usr1,'Daigo')
    Opti = NET.addAssembly('C:\Users\Daigo\Dropbox\Function_update\Quad_experiments\mocap_toolbox\NatNetSDK\lib\x64\NatNetML.dll');
elseif strcmp(usr1,'katar')
    Opti = NET.addAssembly('C:\Users\katar\Dropbox\Quad_experiments\mocap_toolbox\NatNetSDK\lib\x64\NatNetML.dll');
else
    fprintf('You need to sign into either CDCL,Daigo, or katar to set up this network');
end
client = NatNetML.NatNetClientML();

client.Initialize('127.0.0.1','127.0.0.1'); % for Local Loopback

addpath('C:\Users\CDCL\Documents\newMotionCaptureCode\AnthonyREU2018\Quad_experiments\swarming_algorithm\code\');
addpath('C:\Users\CDCL\Documents\newMotionCaptureCode\AnthonyREU2018\Quad_experiments\mocap_toolbox\');

fig=  figure(1);  
set(fig,'deletefcn','client.Uninitialize();  clear(''client'');','Position',[2812 472 560 420]);

%% Matrices for Luenburger observer
observer_gain
paramObs= v2struct(A,B,C,L_observer,L_obsAngle);

%% Initialize (Main Plot)
initialize_plot1
%% Initialize (Variables)
L= 10000; %This sets the size of the arrays. to store more data make L larger
time              = zeros(L,1);
run_time          = zeros(L,2);
x_observer        = zeros(L,6*Np);
x_mocap           = zeros(L,3*Np);
attitude_observer = zeros(L,6*Np);
attitude_mocap    = zeros(L,3*Np);

xhat= zeros(6*Np,1);
zhat= zeros(6*Np,1);
ind_var = repmat(6*[0:Np-1],3,1)+repmat([1;2;3],1,Np);
% This is used to extract the positions or angles but not the rates from
% the state vector; e.g., ind_var = [1,2,3,7,8,9,..];
ind_var= ind_var(:);

% Data coming from the Vehicles
acc(1:Np,1) = {NaN*zeros(L,3)}; % Accelerometer data for each vehicle by row
gyro(1:Np,1) = {NaN*zeros(L,3)}; % Gyro data for each vehicle by row
magnet(1:Np,1) = {NaN*zeros(L,3)}; % Magnetometer for each vehicle by row
%time_IMU(1:Np,1) = {NaN*zeros(L,1)}; % Time from the IMU from each vehicle
current(1:Np,1:2) = {NaN*zeros(L,1)}; % Current data for each vehicle by row and coil by column
loops = 25; % The number of loops of each coil
area=(pi/4)*0.3236; % Area of each coil
%% EVERYTHING ABOVE THIS POINT IS SETUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% THE EXPERIMENT IS RAN BELOW

tt= 1;
tnow= 0;
tend= 5; % (s) This is the experiment duration variable
% it would be nice to not have to specify a time for the experiment to last
% but rather to be able to tell the experiment whento stop by pressing a
% key

while tnow < tend
    
    pause(eps)
    
    % - Mocap data
    [position_raw,angle_raw,t,Rmat] = get_PosAng_ID(client);
    
    cols     = vIDs(1:Np); % columns that contain desired data
    position = position_raw(:,cols); %position = position_raw(:,cols); % in the first iteration cols =[1,2] but the position only has one column
    angle    = angle_raw(:,cols);
    
    % Augment missing position
    [pos, ~] = augment_data(position,xhat(ind_var));
    % Augment missing angles
    [ang, ~] = augment_data(angle,zhat(ind_var));
    
    % Initialization
    if tt==1
        tstart= t;
        tprev= 0;
        xhat(ind_var)= pos;
        zhat(ind_var)= ang;
        disp('Running...')
    end
    
    % Time
    tnow  = t-tstart;
    dt    = tnow-tprev;
    tprev = tnow;
    
    
    % Estimation of position and attitude
    % This part is used to
    % (1) smooth mocap data to reduce effect of noise; and
    % (2) estimate velocities from position data.
    [xhat,zhat] = myObserverAll(xhat,zhat,pos,ang,dt,paramObs);
    
    
    
    % PLOT
    % ** this part slows down the code significantly, so plot should not be
    %    updated when you are actually controlling the vehicles.
    for vid= mod(tt,Np)+1
        xhat_i = xhat([1:6]+(vid-1)*6);
        zhat_i = zhat([1:6]+(vid-1)*6);
        for hind= 1:numel(hquad{vid})
            move_body(hquad{vid}{hind},hdata{vid}(hind,:),xhat_i(1:3),Euler2Rmat(zhat_i(1:3),321))
        end
        % vehicle ID
        set(htext(vid),'position',xhat_i(1:3))
    end
    drawnow
    
     %% Get data from the vehicles
    for i=1:Np
        acc{i}(tt,1)=imu{i}.LatestMessage.LinearAcceleration.X;
        acc{i}(tt,2)=imu{i}.LatestMessage.LinearAcceleration.Y;
        acc{i}(tt,3)=imu{i}.LatestMessage.LinearAcceleration.Z;
        
        gyro{i}(tt,1)=imu{i}.LatestMessage.AngularVelocity.X;
        gyro{i}(tt,2)=imu{i}.LatestMessage.AngularVelocity.Y;
        gyro{i}(tt,3)=imu{i}.LatestMessage.AngularVelocity.Z;
        
        magnet{i}(tt,1)=magApplied{i}.LatestMessage.X;
        magnet{i}(tt,2)=magApplied{i}.LatestMessage.Y;
        magnet{i}(tt,3)=magApplied{i}.LatestMessage.Z;
        
        %time_IMU{Np}(tt)=imu{Np}.LatestMessage.Header.Stamp.Nsec;
        
        current{i,1}(tt)=currApplied{i}.LatestMessage.Data;
        current{i,2}(tt)=currApplied{i}.LatestMessage.Data;
    end
    
    %% SEND SATELLITE COMMANDS
    
    % NOTE: You should only be sending commands to control the magnetic field.
    
    % NOTE: Dr. Paley's controller will output a magnetic dipole moment in polar
    % coordinates, but the satellites reads the magnetometer in cartesian
    % coordinates, so we will have to convert his polar command to a
    % cartesian command. 
    
    % NOTE: We will also have to convert hos magnetic command into a
    % current command to send a current through the coils.
    
    %%%%% OPEN LOOP
    u=[10,pi/4]; %this is the polar input
    
    % Convert the polar dipole commands into current commands for all
    % satellites
    for sat=1:Np
        magCmd{sat,2}.X = (u(1)*cos(u(2)))/(area*loops); %Coil #1
        magCmd{sat,2}.Y = (u(1)*sin(u(2)))/(area*loops); %Coil #2
        magCmd{sat,2}.Z = 0;
        
        % Current Saturation
        if magCmd{sat,2}.X > 20
            magCmd{sat,2}.X = 20;
        end
        
        if magCmd{sat,2}.Y >20
            magCmd{sat,2}.Y = 20;
        end
        
        % Send the command
        send(magCmd{sat,1},magCmd{sat,2}); % Since it is in the for loop it will send this message to every satellite we can communicate with
    end
    
    
    

    %% Save variables
    time(tt)                = tnow;
    x_observer(tt,:)        = xhat;
    x_mocap(tt,:)           = position(:);
    attitude_mocap(tt,:)    = angle(:);
    attitude_observer(tt,:) = zhat;
    
    
    tt= tt+1;
    
end

disp('Stopped')

inde= find(time>0,1,'last');
% delete the portion that does not contain any data
time= time(1:inde);
x_mocap          = x_mocap(1:inde,:);
x_observer       = x_observer(1:inde,:); %[X_position, Y_position, Z_position, X_velocity, Y_velocity, Z_velocity]
attitude_mocap   = attitude_mocap(1:inde,:);
attitude_observer= attitude_observer(1:inde,:); % [Roll, Pitch, Yaw, RollRate, PitchRate, YawRate]
for i=1:Np
    acc{i} = acc{i}(1:inde,:);
    gyro{i} = gyro{i}(1:inde,:);
    magnet{i} = magnet{i}(1:inde,:);
    current{i,1} = current{i}(1:inde,:);
    current{i,2} = current{i}(1:inde,:);
    %time_IMU{Np} = time_IMU{Np}(1:inde);
end

%% PLOTS
% 
% figure(3)
% plot(time,attitude_observer(:,6)*(180/pi)); grid on; hold on; legend('YawRate');
% xlabel('Time (sec)'); ylabel('YawRate (deg/s)'); title('YawRate of Satellite1 v. time')
% 
% figure(4)
% plot(time,attitude_observer(:,5)); grid on; hold on; legend('PitchRate');
% xlabel('Time (sec)'); ylabel('PicthRate (rad/s)'); title('PitchRate of Satellite1 v. time')
% 
% figure(5)
% plot(time,attitude_observer(:,4)); grid on; hold on; legend('RollRate');
% xlabel('Time (sec)'); ylabel('RollRate (rad/s)'); title('RollRate of Satellite1 v. time')












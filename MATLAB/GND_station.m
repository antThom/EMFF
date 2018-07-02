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
rosinit('192.168.1.24','NodeHost','192.168.1.22') %Initialize a new rosnode
pause(2);   %Give matlab some time to setup the node

%% User defined Parameters
prompt='Number of vehicles to track? ';
Np=input(prompt); % # of vehicles that we are tracking at one time

% Vehicle ID specified in Motive
vIDs = 1:Np;

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

fig=  figure();  
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
    
    
    % Save variables
    time(tt)                = tnow;
    x_observer(tt,:)        = xhat;
    x_mocap(tt,:)           = position(:);
    attitude_mocap(tt,:)    = angle(:); 
    attitude_observer(tt,:) = zhat;
    
    tt= tt+1;
    
    
    % In this section I need to publish the data in x_mocap and in
    % attitude_observer
    
end

disp('Stopped')

inde= find(time>0,1,'last');
% delete the portion that does not contain any data
time= time(1:inde);
x_mocap          = x_mocap(1:inde,:);
x_observer       = x_observer(1:inde,:); %[X_position, Y_position, Z_position, X_velocity, Y_velocity, Z_velocity]
attitude_mocap   = attitude_mocap(1:inde,:);
attitude_observer= attitude_observer(1:inde,:); % [Roll, Pitch, Yaw, RollRate, PitchRate, YawRate]
%%  












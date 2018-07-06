function msgsAssign(linMocap, angMocap, lin_pos, ang_pos, lin_vel, ang_vel, PlinPub, VlinPub, PangPub, VangPub)
% This function will assign values to the appropriate ros messages and
% publish them to their respective topics

lin_pos.X = linMocap(:,1); %linear position from MOCAP
lin_pos.Y = linMocap(:,2); %linear position from MOCAP
lin_pos.Z = linMocap(:,3); %linear position from MOCAP

lin_vel.X = linMocap(:,4); %linear velocity from MOCAP
lin_vel.Y = linMocap(:,5); %linear velocity from MOCAP
lin_vel.Z = linMocap(:,6); %linear velocity from MOCAP

ang_pos.X = angMocap(:,1); % angular position from MOCAP
ang_pos.Y = angMocap(:,2); % angular position from MOCAP
ang_pos.Z = angMocap(:,3); % angular position from MOCAP

ang_vel.X = angMocap(:,4); % angular position from MOCAP
ang_vel.Y = angMocap(:,5); % angular position from MOCAP
ang_vel.Z = angMocap(:,6); % angular position from MOCAP

send(PlinPub,lin_pos); % send linear position to its topic
send(VlinPub,lin_vel); % send angular position to its topic
send(PangPub,ang_pos); % send linear velocity to its topic
send(VangPub,ang_vel); % send angular velocity to its topic
end


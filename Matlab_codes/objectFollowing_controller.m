function [isReturned_ret,leftStartArea_ret] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,startPosition,leftStartArea,direction)
vrep=remApi('remoteApi');

V_robot = 0.7;
V_l = 0.5;
V_r = 0.5;
d = 0.415;


% Controller parameters
ref = 0.9; % Reference Value
error = 0;  % Error init
Ap = 0.85; % Gain
Ki = 0.065; % Integrator Constant

% Sensor value to watch
if direction == 1
    sensor_handler = right_LaserSensor_front;
else
    sensor_handler = left_LaserSensor_front;
end


% Move the robot forward
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,V_l,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,V_r,vrep.simx_opmode_blocking);

% Stopping condition
[returnCode,stop,~,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_blocking);
isReturned = false;


% Read the Right laser's value - Controlled attribute [y]
[returnCode,dState,currentDistance,~,~]=vrep.simxReadProximitySensor(clientID,sensor_handler,vrep.simx_opmode_blocking);

distVal= [];
prev_err= [0];
% Control loop
tic
while ~stop && (~isReturned || ~leftStartArea)
    
    [returnCode,dState,currentDistance,~,~]=vrep.simxReadProximitySensor(clientID,sensor_handler,vrep.simx_opmode_blocking);
    tElapsed = toc;
    tic
    y = currentDistance(3); % Controlled signal
    error = ref - y; % Error signal
    prev_err = [prev_err tElapsed*error];
    I = sum(prev_err);
    
    u = Ap*error-Ki*I; % Controlling signal
    
    if direction == 1
        u = -u;
    end

    [V_l,V_r]=calculateWheelSpeed(u,d,V_robot);
    
    % In case of end of the wall (~90 degrees turn)
    if dState == 0
        if direction == 1
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.6,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.4,vrep.simx_opmode_blocking);
        else
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,0.4,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,0.6,vrep.simx_opmode_blocking);
        end
    else
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,V_l,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,V_r,vrep.simx_opmode_blocking);
    end

    
    [returnCode,stop,~,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_blocking);
    if leftStartArea 
        isReturned = isNearby(clientID,pioneer_Robot,reference_Box,startPosition,1);   
    else
        leftStartArea = isNearby(clientID,pioneer_Robot,reference_Box,startPosition,1);
        leftStartArea = ~leftStartArea; 
    end
    distVal=[distVal currentDistance(3)];

end
isReturned_ret = isReturned;
leftStartArea_ret = leftStartArea;

toc

% % Diagnostic plot
% sizeArr=size(distVal);
% sizeArr=sizeArr(2);
% timestamp = ones(size(distVal));
% refArr = timestamp;
% for i=1:sizeArr
%     timestamp(i)=timestamp(i)*i;
% end
% refArr = ref*refArr;
% plot(timestamp,refArr,timestamp,distVal)
end




function [velocity_left,velocity_right]=calculateWheelSpeed(omega,d,V_robot)
    velocity_left = V_robot + d/2 * omega;
    velocity_right = V_robot - d/2 * omega;  
end
function [isReturned_ret,leftStartArea_ret,closestPoint_ret] = objectFollowing_controller(startPosition,leftStartArea,direction,referenceDistance,lineStartPoint,lineEndPoint,recordDistances,isEndPointGiven,endpoint)
vrep=remApi('remoteApi');
global simulationHandlers_t;

% Robot parameters
V_robot = 0.7;
V_l = 0.5;
V_r = 0.5;
d = 0.415;

% Mode parameters
eps = 1; % Max range from the goal position
if isEndPointGiven
    leftStartArea = true;
    startPosition = endpoint;
    eps = 0.7;
end

% Controller parameters
ref = referenceDistance;% Reference Value
error = 0;  % Error init
Ap = 0.85; % Gain
Ki = 0.065; % Integrator Constant

% Sensor value to watch
if direction == 1
    sensor_handler = simulationHandlers_t.right_LaserSensor_front;
else
    sensor_handler = simulationHandlers_t.left_LaserSensor_front;
end

% Init values

if recordDistances
    minDist = 100;
    closestPoint = [100 100];
    [~, StartPoint]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
end

% Move the robot forward
[~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,V_l,vrep.simx_opmode_blocking);
[~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,V_r,vrep.simx_opmode_blocking);

% Stopping condition
[~,stop,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
isReturned = false;


% Read the Right laser's value - Controlled attribute [y]
[~,dState,currentDistance,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,sensor_handler,vrep.simx_opmode_blocking);

distVal= [];
prev_err= [0];
% Control loop
tic
while ~stop && (~isReturned || ~leftStartArea)
    [~,dState,currentDistance,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,sensor_handler,vrep.simx_opmode_blocking);
    if recordDistances
        [~, point]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
        [closestPoint,minDist] = closestPointFromLine(point,closestPoint,StartPoint,lineStartPoint,lineEndPoint,minDist);
%         fprintf('closest: x - %.4f, y - %.4f MinDist: %.4f \n',closestPoint(1),closestPoint(2),minDist);
    end
    tElapsed = toc;
    tic
    y = currentDistance(3); % Controlled signal
    if (y > 1.5) % Saturation
        y = 1.5;
    elseif (y < -1.5)
        y = -1.5;
    end
    error = ref - y; % Error signal
    prev_err = [prev_err tElapsed*error];
    I = sum(prev_err);
    
    u = Ap*error-Ki*I; % Controlling signal
    
    if direction == 1
        u = -u;
    end
    
    [V_l,V_r]=calculateWheelSpeed(u,d,V_robot);
%     fprintf('Error: %.4f, U: %.4f \n',error,u);
%     fprintf('U: %.4f, W_L: %.4f W_R: %.4f! \n',u,V_l,V_r);
%     fprintf('dstate: %.4f \n',dState);
    % In case of end of the wall (~90 degrees turn)
    if dState == 0
        if direction == 1
            [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,0.6,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,0.4,vrep.simx_opmode_blocking);
        else
            [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,0.4,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,0.6,vrep.simx_opmode_blocking);
        end
    else
        [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,V_l,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,V_r,vrep.simx_opmode_blocking);
    end

    
    [~,stop,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
    if leftStartArea 
        
        isReturned = isNearby(startPosition,eps);   
    else
        leftStartArea = isNearby(startPosition,eps);
        leftStartArea = ~leftStartArea; 
    end
    distVal=[distVal currentDistance(3)];

end
isReturned_ret = isReturned;
leftStartArea_ret = leftStartArea;
if recordDistances
    closestPoint_ret = closestPoint;
else
    closestPoint_ret = [0 0];
end

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
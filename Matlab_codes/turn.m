function turn(phi,isStartOrientation,startOrientation,normalToWall,direction,velocityFastArg)

vrep=remApi('remoteApi');
global simulationHandlers_t;

leftTurn = 0;
rightTurn = 0;
eps = 0.09; % For normal to wall check
eps_goal_ori = 0.4;
eps_3 = 0.0095; % For back turn
phiCheck = 0;
velocityVerySlow = 0.01;
velocitySlow = 0.02;
% velocityFast = 0.6;
velocityFast = velocityFastArg;
velocity = 0;
goalOrientation = 0;


[~, robotOrientationEuler]=vrep.simxGetObjectOrientation(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
currentOrientation = robotOrientationEuler_deg(3);
if normalToWall == 0
    if currentOrientation < 0
        currentOrientation = currentOrientation + 360;
    end


    if phi < 0 
       rightTurn = 1;
       phiCheck = 1;
    elseif phi > 0
        leftTurn = 1;
        phiCheck = 1;
    elseif phi < -360 && phi > 360 && phi == 0
       disp('Impossible turn event, the turning angle should be between this range: [-360,360]\0');
    end

    if phiCheck
    
       if isStartOrientation
           goalOrientation = startOrientation;
       else
            goalOrientation = currentOrientation + phi;
       end

       if goalOrientation > 360
           goalOrientation = goalOrientation - 360;
       elseif goalOrientation < 0
           goalOrientation = goalOrientation + 360;
       end
    

       while (abs(goalOrientation - currentOrientation) > eps_goal_ori) && (abs(goalOrientation - currentOrientation + 360) > eps_goal_ori) && (abs(goalOrientation - currentOrientation - 360)  > eps_goal_ori) 
%            fprintf('OrientationDelta: %.4f \n',abs(goalOrientation - currentOrientation));
           if (abs(goalOrientation - currentOrientation) <= 10) || (abs(goalOrientation - currentOrientation + 360) <= 10) || (abs(goalOrientation - currentOrientation - 360) <= 10)
              velocity = velocitySlow;
          else
              velocity = velocityFast;
          end        
          if rightTurn
             [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocity,vrep.simx_opmode_blocking);
             [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocity,vrep.simx_opmode_blocking);
          elseif leftTurn
             [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocity,vrep.simx_opmode_blocking);
             [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocity,vrep.simx_opmode_blocking);
          end
          [~, robotOrientationEuler]=vrep.simxGetObjectOrientation(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
          robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
          currentOrientation = robotOrientationEuler_deg(3);
          if currentOrientation < 0
             currentOrientation = currentOrientation + 360;
          end
       end
    move(0);
    end
    
% Turning normal to wall left (the wall will be on the right)
elseif normalToWall == 1
    % Start turning 
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocityFast,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocityFast,vrep.simx_opmode_blocking);
    
    % Read the sensor data buffers
    [~,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
    [~,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_blocking);
    [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
    [~,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
    [~,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_rear,vrep.simx_opmode_blocking);
    
    % If there isn't any object in the sensor range turn faster
    while (detectionStateF || detectionStateFLA || detectionStateFRA)
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocityFast,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocityFast,vrep.simx_opmode_blocking);
        [~,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
        [~,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_blocking);
        [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
    end
    
    % Now Both sensor have an object in the range turn slower
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocitySlow,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocitySlow,vrep.simx_opmode_blocking);
    
    [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
    [~,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
    [~,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_rear,vrep.simx_opmode_blocking);
    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointRFLaserSensor(3) - detectedPointRRLaserSensor(3)) > eps) || ((detectionStateRF == 0) || (detectionStateRR == 0)) || (detectionStateFRA == 1)
        [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
        [~,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
        [~,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_rear,vrep.simx_opmode_blocking);
    end
    
    move(0);
    
% Turning normal to wall right (the wall will be on the left)
elseif normalToWall == -1
    % Start turning 
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocityFast,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocityFast,vrep.simx_opmode_blocking);
    
    % Read the sensor data buffers
    [~,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
    [~,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_blocking);
    [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
    [~,detectionStateLF,detectedPointLFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.left_LaserSensor_front,vrep.simx_opmode_blocking);
    [~,detectionStateLR,detectedPointLRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.left_LaserSensor_rear,vrep.simx_opmode_blocking);

    % If there isn't any object in the sensor range turn faster
    while (detectionStateF || detectionStateFLA || detectionStateFRA)
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocityFast,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocityFast,vrep.simx_opmode_blocking);
        [~,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
        [~,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_blocking);
        [~,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_blocking);
    end
    
    % Now Both sensor have an object in the range turn slower
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocitySlow,vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocitySlow,vrep.simx_opmode_blocking);
    
    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointLFLaserSensor(3) - detectedPointLRLaserSensor(3)) > eps) || ((detectionStateLF == 0) || (detectionStateLR == 0))
        [~,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_blocking);
        [~,detectionStateLF,detectedPointLFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.left_LaserSensor_front,vrep.simx_opmode_blocking);
        [~,detectionStateLR,detectedPointLRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.left_LaserSensor_rear,vrep.simx_opmode_blocking);
    end
    
    move(0);
    
% Turning back of the wall
elseif normalToWall == 2
    % Start turning 
    if direction == 1
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocitySlow,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocitySlow,vrep.simx_opmode_blocking);
    else
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocitySlow,vrep.simx_opmode_blocking);
        [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocitySlow,vrep.simx_opmode_blocking);
    end

    
    % Read the sensor data buffers
    [~,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
    [~,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_rear,vrep.simx_opmode_blocking);
    [~,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_right,vrep.simx_opmode_blocking);
    [~,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_left,vrep.simx_opmode_blocking);

    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointBRLaserSensor(3) - detectedPointBLLaserSensor(3)) > eps_3) || ((detectionStateBR == 0) && (detectionStateBL == 0))
        if direction == 1
            [~,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_left,vrep.simx_opmode_blocking);
            [~,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_right,vrep.simx_opmode_blocking);
        else
            [~,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_right,vrep.simx_opmode_blocking);
            [~,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.back_LaserSensor_left,vrep.simx_opmode_blocking);
        end

        if ((abs(detectedPointBRLaserSensor(3) - detectedPointBLLaserSensor(3)) <= eps) && (detectionStateBR ~= 0) && (detectionStateBL ~= 0))
            velocity = velocityVerySlow;
        else
        	velocity = velocitySlow;
        end
        if direction == 1
            [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocity,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,-velocity,vrep.simx_opmode_blocking);
        else
            [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,-velocity,vrep.simx_opmode_blocking);
            [~]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocity,vrep.simx_opmode_blocking);
        end
    end
    
    move(0);
    
end


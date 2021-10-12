function turn(phi,clientID,lMotorHandler,rMotorHandler,robotHandler,fLaserSensorHandle,fLaserSensorRAHandler,fLaserSensorLAHandler,rLaserSensorFHandler,rLaserSensorRHandler,lLaserSensorFHandler,lLaserSensorRHandler,bLaserSensorRHandler,bLaserSensorLHandler,startOrientation,normalToWall,direction,velocityFastArg)

vrep=remApi('remoteApi');

leftTurn = 0;
rightTurn = 0;
eps = 0.03; % For normal to wall check
eps_2 = 0.1; % For start orientation
eps_3 = 0.001; % For back turn
phiCheck = 0;
velocityVerySlow = 0.0001;
velocitySlow = 0.02;
% velocityFast = 0.6;
velocityFast = velocityFastArg;
velocity = 0;
goalOrientation = 0;


[returnCode, robotOrientationEuler]=vrep.simxGetObjectOrientation(clientID,robotHandler,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
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
    
       if startOrientation
           goalOrientation = 0;
       else
            goalOrientation = currentOrientation + phi;
       end

       if goalOrientation > 360
           goalOrientation = goalOrientation - 360;
       elseif goalOrientation < 0
           goalOrientation = goalOrientation + 360;
       end
    

       while (abs(goalOrientation - currentOrientation) > eps_2) 
          fprintf('Goal orientation [deg]: %.4f, Current Orientation [deg]: %.4f \n', goalOrientation, currentOrientation);
        
          if (abs(goalOrientation - currentOrientation) <= 15) || (abs(goalOrientation - currentOrientation + 360) <= 15) || (abs(goalOrientation - currentOrientation - 360) <= 15)
              velocity = velocitySlow;
          elseif (abs(goalOrientation - currentOrientation) <= 5) || (abs(goalOrientation - currentOrientation + 360) <= 5) || (abs(goalOrientation - currentOrientation - 360) <= 5)
              velocity = velocityVerySlow;
          else
              velocity = velocityFast;
          end
        
          if rightTurn
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,velocity,vrep.simx_opmode_blocking);
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,-velocity,vrep.simx_opmode_blocking);
          elseif leftTurn
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,-velocity,vrep.simx_opmode_blocking);
             [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,velocity,vrep.simx_opmode_blocking);
          end
          [returnCode, robotOrientationEuler]=vrep.simxGetObjectOrientation(clientID,robotHandler,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
          robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
          currentOrientation = robotOrientationEuler_deg(3);
          if currentOrientation < 0
             currentOrientation = currentOrientation + 360;
          end
%           fprintf('Diff: %.4f \n',abs(goalOrientation - currentOrientation));
       end
    move(0,clientID,lMotorHandler,rMotorHandler);
    end
    
% Turning normal to wall left (the wall will be on the right)
elseif normalToWall == 1
    % Start turning 
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,-velocityFast,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,velocityFast,vrep.simx_opmode_blocking);
    
    % Read the sensor data buffers
    [returnCode,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorHandle,vrep.simx_opmode_blocking);
    [returnCode,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorLAHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorFHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorRHandler,vrep.simx_opmode_blocking);
%     fprintf('RF: %.4f    RR: %.4f \n',detectedPointRFLaserSensor(3), detectedPointRRLaserSensor(3));
    
    % If there isn't any object in the sensor range turn faster
    while (detectionStateF || detectionStateFLA || detectionStateFRA)
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,-velocityFast,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,velocityFast,vrep.simx_opmode_blocking);
        [returnCode,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorHandle,vrep.simx_opmode_blocking);
        [returnCode,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorLAHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
    end
    
    % Now Both sensor have an object in the range turn slower
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,-velocitySlow,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,velocitySlow,vrep.simx_opmode_blocking);
    
    [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorFHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorRHandler,vrep.simx_opmode_blocking);
    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointRFLaserSensor(3) - detectedPointRRLaserSensor(3)) > eps) || ((detectionStateRF == 0) && (detectionStateRR == 0)) || (detectionStateFRA == 1)
        [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorFHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorRHandler,vrep.simx_opmode_blocking);
%         fprintf('Diff: %.4f \n Values: %.4f %.4f \n',abs(detectedPointRFLaserSensor(3) - detectedPointRRLaserSensor(3)), detectedPointRFLaserSensor(3),detectedPointRRLaserSensor(3));
        fprintf('TURNING! \n');
    end
    
    move(0,clientID,lMotorHandler,rMotorHandler);
    
% Turning normal to wall right (the wall will be on the left)
elseif normalToWall == -1
    % Start turning 
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,velocityFast,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,-velocityFast,vrep.simx_opmode_blocking);
    
    % Read the sensor data buffers
    [returnCode,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorHandle,vrep.simx_opmode_blocking);
    [returnCode,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorLAHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateLF,detectedPointLFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,lLaserSensorFHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateLR,detectedPointLRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,lLaserSensorRHandler,vrep.simx_opmode_blocking);
%     fprintf('LF: %.4f    LR: %.4f \n',detectedPointLFLaserSensor(3), detectedPointLRLaserSensor(3));
    
    % If there isn't any object in the sensor range turn faster
    while (detectionStateF || detectionStateFLA || detectionStateFRA)
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,velocityFast,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,-velocityFast,vrep.simx_opmode_blocking);
        [returnCode,detectionStateF,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorHandle,vrep.simx_opmode_blocking);
        [returnCode,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorLAHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateFRA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorRAHandler,vrep.simx_opmode_blocking);
    end
    
    % Now Both sensor have an object in the range turn slower
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,velocitySlow,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,-velocitySlow,vrep.simx_opmode_blocking);
    
    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointLFLaserSensor(3) - detectedPointLRLaserSensor(3)) > eps) || ((detectionStateLF == 0) && (detectionStateLR == 0)) || (detectionStateFLA == 1)
        [returnCode,detectionStateFLA,~,~,~]=vrep.simxReadProximitySensor(clientID,fLaserSensorLAHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateLF,detectedPointLFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,lLaserSensorFHandler,vrep.simx_opmode_blocking);
        [returnCode,detectionStateLR,detectedPointLRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,lLaserSensorRHandler,vrep.simx_opmode_blocking);
%         fprintf('Diff: %.4f \n Values: %.4f %.4f \n',abs(detectedPointLFLaserSensor(3) - detectedPointLRLaserSensor(3)), detectedPointLFLaserSensor(3),detectedPointLRLaserSensor(3));
    end
    
    move(0,clientID,lMotorHandler,rMotorHandler);
    
% Turning back of the wall
elseif normalToWall == 2
    % Start turning 
    if direction == 1
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,velocitySlow,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,-velocitySlow,vrep.simx_opmode_blocking);
    else
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,lMotorHandler,-velocitySlow,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,rMotorHandler,velocitySlow,vrep.simx_opmode_blocking);
    end

    
    % Read the sensor data buffers
    [returnCode,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorFHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateRR,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,rLaserSensorRHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorRHandler,vrep.simx_opmode_blocking);
    [returnCode,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorLHandler,vrep.simx_opmode_blocking);
%     fprintf('BR: %.4f    BL: %.4f \n',detectedPointBRLaserSensor(3), detectedPointBLLaserSensor(3));
    

    % While the two sensors don't measure approx. the same value turn (while the robot is not normal to the wall)
    while (abs(detectedPointBRLaserSensor(3) - detectedPointBLLaserSensor(3)) > eps) || ((detectionStateBR == 0) && (detectionStateBL == 0))
        if direction == 1
            [returnCode,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorLHandler,vrep.simx_opmode_blocking);
            [returnCode,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorRHandler,vrep.simx_opmode_blocking);
        else
            [returnCode,detectionStateBR,detectedPointBRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorRHandler,vrep.simx_opmode_blocking);
            [returnCode,detectionStateBL,detectedPointBLLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,bLaserSensorLHandler,vrep.simx_opmode_blocking);
        end
        
%         fprintf('Diff: %.4f \n Values: %.4f %.4f \n',abs(detectedPointRFLaserSensor(3) - detectedPointRRLaserSensor(3)), detectedPointRFLaserSensor(3),detectedPointRRLaserSensor(3));
    end
    
    move(0,clientID,lMotorHandler,rMotorHandler);
    
end


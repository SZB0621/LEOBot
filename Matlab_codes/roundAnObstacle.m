function isReturned = roundAnObstacle(clientID,left_Motor,right_Motor,right_LaserSensor_front,right_LaserSensor_rear,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,pioneer_Robot,reference_Box,xyz_current)
%ROUNDANOBSTACLE Summary of this function goes here
%   Detailed explanation goes here
    vrep=remApi('remoteApi');

    direction= 1;
    normalToWall = 1;
    leftStartArea = 0;
    turnVelocity = 0.02;
    referenceDistance = 0.4;
    startOrientation = 0;
    isReturned_ret = false;
    leftStartArea_ret = false;

    % Read the front sensor values to decide in which direction to go
    [returnCode,detectionState_F,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_streaming);
    [returnCode,detectionState_FRA,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_rightAngle,vrep.simx_opmode_streaming);
    [returnCode,detectionState_FLA,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_leftAngle,vrep.simx_opmode_streaming);
    [returnCode,detectionState_F,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_buffer);
    [returnCode,detectionState_FRA,detectedPointFrontRALaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_rightAngle,vrep.simx_opmode_buffer);
    [returnCode,detectionState_FLA,detectedPointFrontLALaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_leftAngle,vrep.simx_opmode_buffer);

    if (detectionState_FRA && ~(detectionState_F)) % Turn left, object on the right
        direction = 1;
        normalToWall = 1;
    elseif (detectionState_FLA && ~(detectionState_F)) % Turn right, object on the left
        direction = 0;
        normalToWall = -1;
    else % Direction doesn't matter default setup is to turn right
        direction = 1;
        normalToWall = 1;
    end

    while ~(isReturned_ret && leftStartArea_ret)
        leftStartArea = leftStartArea_ret;
        turn(0,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction,turnVelocity);
        [isReturned_ret,leftStartArea_ret] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,xyz_current,leftStartArea,direction,referenceDistance);
    end
        isReturned = isReturned_ret;
    end


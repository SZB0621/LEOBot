function [isReturned_ret,closestPoint_ret] = roundAnObstacle(xyz_current,lineStartPoint,lineEndPoint,isEndPointGiven,endpoint)
    vrep=remApi('remoteApi');
    global simulationHandlers_t;

    direction= 1;
    normalToWall = 1;
    leftStartArea = false;
    turnVelocity = 0.2;
    referenceDistance = 0.5;
    isStartOrientation = 0;
    startOrientation = 0;
    isReturned = false;
    leftStartArea_ret = false;
    closestPoint = [0,0];
    recordDistances = true;
    
    if isEndPointGiven
        leftStartArea_ret = true;
        recordDistances = false;
    end

    % Read the front sensor values to decide in which direction to go
    [~,~,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_streaming);
    [~,~,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_streaming);
    [~,~,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_streaming);
    [~,detectionState_F,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_buffer);
    [~,detectionState_FRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_buffer);
    [~,detectionState_FLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_buffer);

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

    while ~(isReturned && leftStartArea_ret)
        leftStartArea = leftStartArea_ret;
        turn(0,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
        [isReturned,leftStartArea_ret,closestPoint] = objectFollowing_controller(xyz_current,leftStartArea,direction,referenceDistance,lineStartPoint,lineEndPoint,recordDistances,isEndPointGiven,endpoint);
%         fprintf('CLOSEST POINT: %.4f, %.4f \n', closestPoint_ret(1),closestPoint_ret(2));
    end
        isReturned_ret = isReturned;
        closestPoint_ret = closestPoint;
        
end


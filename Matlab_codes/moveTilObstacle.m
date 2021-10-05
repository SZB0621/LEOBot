function isObjectInFront=moveTilObstacle(clientID,left_Motor,right_Motor,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle)
vrep=remApi('remoteApi');

isObjectInFront = false;
[returnCode,detectionState_F,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_streaming);
[returnCode,detectionState_FRA,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_rightAngle,vrep.simx_opmode_streaming);
[returnCode,detectionState_FLA,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_leftAngle,vrep.simx_opmode_streaming);

move(2,clientID,left_Motor,right_Motor);

while (~detectionState_F && ~detectionState_FRA && ~detectionState_FLA)
    [returnCode,detectionState_F,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_buffer);
    [returnCode,detectionState_FRA,detectedPointFrontRALaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_rightAngle,vrep.simx_opmode_buffer);
    [returnCode,detectionState_FLA,detectedPointFrontLALaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor_leftAngle,vrep.simx_opmode_buffer);
end

move(0,clientID,left_Motor,right_Motor);
isObjectInFront = true;
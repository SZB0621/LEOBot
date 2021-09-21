function isObjectInFront=moveTilObstacle(clientID,left_Motor,right_Motor,front_LaserSensor)
vrep=remApi('remoteApi');

isObjectInFront = false;
[returnCode,detectionState,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_streaming);
move(2,clientID,left_Motor,right_Motor);

while ~detectionState
    [returnCode,detectionState,detectedPointFrontLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,front_LaserSensor,vrep.simx_opmode_buffer);
end

move(0,clientID,left_Motor,right_Motor);
isObjectInFront = true;
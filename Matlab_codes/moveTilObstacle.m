function isObjectInFront=moveTilObstacle()
vrep=remApi('remoteApi');
global simulationHandlers_t;

isObjectInFront = false;
[~,detectionState_F,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_streaming);
[~,detectionState_FRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_streaming);
[~,detectionState_FLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_streaming);

move(2);

while (~detectionState_F && ~detectionState_FRA && ~detectionState_FLA)
    [~,detectionState_F,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_buffer);
    [~,detectionState_FRA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_rightAngle,vrep.simx_opmode_buffer);
    [~,detectionState_FLA,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor_leftAngle,vrep.simx_opmode_buffer);
end

move(0);
isObjectInFront = true;
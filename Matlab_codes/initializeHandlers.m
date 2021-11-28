function initializeHandlers()
vrep=remApi('remoteApi');
% Handle Code

% Struct for the easier access of handlers
global simulationHandlers_t;

[~,simulationHandlers_t.pioneer_Robot]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking); % Robot       
[~,simulationHandlers_t.reference_Box]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'ReferenceBox',vrep.simx_opmode_blocking); % Reference box       
[~,simulationHandlers_t.left_Motor]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking); % Robot Motor Left
[~,simulationHandlers_t.right_Motor]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking); % Robot Motor Right       
[~,simulationHandlers_t.front_LaserSensor]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Front_LaserSensor',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.front_LaserSensor_leftAngle]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Front_LaserSensor_leftAngle',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.front_LaserSensor_rightAngle]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Front_LaserSensor_rightAngle',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.right_LaserSensor_front]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Right_LaserSensor_front',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.right_LaserSensor_rear]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Right_LaserSensor_rear',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.left_LaserSensor_front]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Left_LaserSensor_front',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.left_LaserSensor_rear]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Left_LaserSensor_rear',vrep.simx_opmode_blocking);
[~,simulationHandlers_t.back_LaserSensor_right]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Back_LaserSensor_right',vrep.simx_opmode_blocking); 
[~,simulationHandlers_t.back_LaserSensor_left]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Back_LaserSensor_left',vrep.simx_opmode_blocking);

% [~,camera]=vrep.simxGetObjectHandle(simulationHandlers_t.clientID,'Vision_sensor',vrep.simx_opmode_blocking); % Robot Camera (vision sensor)
      
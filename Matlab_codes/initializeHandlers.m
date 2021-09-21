function [pioneer_Robot,reference_Box,left_Motor,right_Motor,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left]=initializeHandlers(clientID)
vrep=remApi('remoteApi');
% Handle Code
[~,pioneer_Robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking); % Robot       
[~,reference_Box]=vrep.simxGetObjectHandle(clientID,'ReferenceBox',vrep.simx_opmode_blocking); % Reference box       
[~,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking); % Robot Motor Left
[~,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking); % Robot Motor Right       
% [~,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking); % Default ultrasonic sensor      
[~,front_LaserSensor]=vrep.simxGetObjectHandle(clientID,'Front_LaserSensor',vrep.simx_opmode_blocking); 
[~,front_LaserSensor_leftAngle]=vrep.simxGetObjectHandle(clientID,'Front_LaserSensor_leftAngle',vrep.simx_opmode_blocking); 
[~,front_LaserSensor_rightAngle]=vrep.simxGetObjectHandle(clientID,'Front_LaserSensor_rightAngle',vrep.simx_opmode_blocking); 
[~,right_LaserSensor_front]=vrep.simxGetObjectHandle(clientID,'Right_LaserSensor_front',vrep.simx_opmode_blocking); 
[~,right_LaserSensor_rear]=vrep.simxGetObjectHandle(clientID,'Right_LaserSensor_rear',vrep.simx_opmode_blocking); 
[~,left_LaserSensor_front]=vrep.simxGetObjectHandle(clientID,'Left_LaserSensor_front',vrep.simx_opmode_blocking); 
[~,left_LaserSensor_rear]=vrep.simxGetObjectHandle(clientID,'Left_LaserSensor_rear',vrep.simx_opmode_blocking);
[~,back_LaserSensor_right]=vrep.simxGetObjectHandle(clientID,'Back_LaserSensor_right',vrep.simx_opmode_blocking); 
[~,back_LaserSensor_left]=vrep.simxGetObjectHandle(clientID,'Back_LaserSensor_left',vrep.simx_opmode_blocking);
% [~,camera]=vrep.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking); % Robot Camera (vision sensor)
      
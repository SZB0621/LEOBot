function move(velocity)

vrep=remApi('remoteApi');
global simulationHandlers_t;

[returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.left_Motor,velocity,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(simulationHandlers_t.clientID,simulationHandlers_t.right_Motor,velocity,vrep.simx_opmode_blocking);
end
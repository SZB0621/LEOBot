function move(velocity,clientID,left_Motor,right_Motor)

vrep=remApi('remoteApi');

[returnCode]=vrep.simxSetJointTargetVelocity(clientID,left_Motor,velocity,vrep.simx_opmode_blocking);
[returnCode]=vrep.simxSetJointTargetVelocity(clientID,right_Motor,velocity,vrep.simx_opmode_blocking);

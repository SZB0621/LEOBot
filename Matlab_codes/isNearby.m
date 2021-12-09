function isReturned = isNearby(startPosition,eps)

vrep=remApi('remoteApi');
global simulationHandlers_t;

[~, robotPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);

dist = pdist([startPosition(1),startPosition(2),startPosition(3);robotPosition(1),robotPosition(2),robotPosition(3)],'euclidean');
% fprintf('dist: %.4f \n',dist);

if dist <= eps
    isReturned = true;
else
    isReturned = false;
end
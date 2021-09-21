function isReturned = isNearby(clientID,pioneer_Robot,reference_Box,startPosition,eps)

vrep=remApi('remoteApi');

[returnCode, robotPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);

dist = pdist([startPosition(1),startPosition(2),startPosition(3);robotPosition(1),robotPosition(2),robotPosition(3)],'euclidean');

if dist <= eps
    isReturned = true;
else
    isReturned = false;
end
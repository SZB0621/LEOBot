function [xy_end] = calcLineEndPosition(clientID,pioneer_Robot,reference_Box,sectionSize)
%AVOIDOBSTACLE 
vrep=remApi('remoteApi');
% Calculate the endpoint on the given line (from the start point &
% orientation and the sectionSize, which is the lenght of the wall)
[~, xyz_start]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
[~, robotOrientationEuler]=vrep.simxGetObjectOrientation(clientID,pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
line_start_orientation = robotOrientationEuler_deg(3);
x_end = xyz_start(1)+cosd(line_start_orientation)*sectionSize;
y_end = xyz_start(2)+sind(line_start_orientation)*sectionSize;
xy_end = [x_end,y_end,xyz_start(3)];
fprintf('START: %.4f, %.4f \n', xyz_start(1),xyz_start(2));
fprintf('END: %.4f, %.4f \n', x_end,y_end);

end


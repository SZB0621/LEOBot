% Robot Control main
clearvars
close all
% Init section
% vrep=remApi('remoteApi'); % Initializing the remote api file for coppelia
% vrep.simxFinish(-1); % just in case, close all opened connections

% clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % Setup connection parameters
% if (clientID>-1)
%        [~,Right_LaserSensor_front]=vrep.simxGetObjectHandle(clientID,'Right_LaserSensor_front',vrep.simx_opmode_blocking);      
%        [~,Right_LaserSensor_rear]=vrep.simxGetObjectHandle(clientID,'Right_LaserSensor_rear',vrep.simx_opmode_blocking);
%        [~,pioneer_Robot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
%        [~,reference_Box]=vrep.simxGetObjectHandle(clientID,'ReferenceBox',vrep.simx_opmode_blocking);
%        prev_min = 100;
%        closest_point = [10 10];
%     while 1
        

%        [returnCode,detectionStateRF,detectedPointRFLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,Right_LaserSensor_front,vrep.simx_opmode_blocking);
%        [returnCode,detectionStateRF,detectedPointRRLaserSensor,~,~]=vrep.simxReadProximitySensor(clientID,Right_LaserSensor_rear,vrep.simx_opmode_blocking);
%        [~,pos]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
%        line_coordinates = [2.6674809 11.2550821 11.5132065 4.3587499];
%        line_coordinates = [0 0 11.5132065 4.3587499];
%        distance = GetPointLineDistance(pos(1),pos(2),line_coordinates(1),line_coordinates(2),line_coordinates(3),line_coordinates(3));
%        fprintf('DISTANCE = %.4f \n',distance);
%        fprintf('CLOSEST POINT: X = %.4f Y = %.4f \n',closest_point(1),closest_point(2));
%        dist = pdist([line_coordinates(1),line_coordinates(2),0;pos(1),pos(2),0],'euclidean');
%        if (distance < prev_min && dist > 1)
%            prev_min = distance;
%            closest_point(1) = pos(1);
%            closest_point(2) = pos(2);
%        end
%        [returnCode, robotOrientationEuler]=vrep.simxGetObjectOrientation(clientID,pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
%        robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
%        currentOrientation = robotOrientationEuler_deg(3);
%        fprintf('X: %.4f Y: %.4f \n ORI: %.4f \n',pos(1),pos(2), currentOrientation);
%        fprintf('BR: %.4f    BL: %.4f \n',detectedPointBRLaserSensor(3), detectedPointBLLaserSensor(3));
%        fprintf('%.4f %.4f \n',detectedPointRFLaserSensor(3),detectedPointRRLaserSensor(3));

       
%        pause(0.5)
%     end
    % Post simulation clean up
%        vrep.simxFinish(-1);
% end
 
% vrep.delete();

clear
global x
x.a =0;
x.b =10;
x.c =100;

sum




% Get the distance from a point (x3, y3) to
% a line defined by two points (x1, y1) and (x2, y2);
% Reference: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
function distance = GetPointLineDistance(x3,y3,x1,y1,x2,y2)
try
	
	% Find the numerator for our point-to-line distance formula.
	numerator = abs((x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1));
	
	% Find the denominator for our point-to-line distance formula.
	denominator = sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2);
	
	% Compute the distance.
	distance = numerator ./ denominator;
catch ME
	callStackString = GetCallStack(ME);
	errorMessage = sprintf('Error in program %s.\nTraceback (most recent at top):\n%s\nError Message:\n%s',...
		mfilename, callStackString, ME.message);
	uiwait(warndlg(errorMessage))
end
return; % from GetPointLineDistance()
end
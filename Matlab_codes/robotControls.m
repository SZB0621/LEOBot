% Robot Control main
clearvars
close all

% Init section
vrep=remApi('remoteApi'); % Initializing the remote api file for coppelia
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % Setup connection parameters

% Global variables
velocity = 0.5;
startOrientation = true; % If start orientation is 1 in turn function the robot will turn into 0 postition
global_startPosition = 0; % Variable to store the start position (for the whole strategy)
local_startPosition = 0; % Variable to store the start position (only for the local primitives)
local_endPosition = 0;
leftStartArea = false;
area = [];
wallA = 0;
wallB = wallA;
Rvision = 3; % The range of the laser sensors to the sides (1.5 + 1.5)
NEcorner = 0;
SEcorner = 0;
SWcorner = 0;

% Control commands
command = 1000;
areaMeasured = false;
normalToWall = 0; % Values = [-1,0,1,2] 0 - normal turn function, -1/1 turn normal to the obstacle in front -1 turn right 1 turn left 2 back of the wall
direction = 0; % On which side is the wall -1 left, 1 right

% Fast check for the succesful connection between Matlab and Coppelia
if (clientID>-1)
       disp('Connection succesful...')
       
       while command ~= 1 && command ~= 0
           prompt = 'The script is ready for execution enter: 1 - to start the execution, 0 -to exit the script';
           command = input(prompt);
       end
       
       % Execution
       if command == 1
           % Setup Handlers
           [pioneer_Robot,reference_Box,left_Motor,right_Motor,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left]=initializeHandlers(clientID);
       
           % Turn and move into start position
           startOrientation = true; % If start orientation is 1 in turn function the robot will turn into 0 postition
           normalToWall = 0; % Values = [-1,0,1,2] 0 - normal turn function, -1/1 turn normal to the obstacle in front -1 turn right 1 turn left 2 back of the wall
           turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
           startOrientation = false;
           moveTilObstacle(clientID,left_Motor,right_Motor,front_LaserSensor);
           normalToWall = 1;
           turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
           [~, global_startPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
           direction = 1;
           [isReturned,leftStartArea] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,global_startPosition,leftStartArea,direction);
           
           % Measure the complete area
           while ~areaMeasured
               [~,local_startPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
               turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
               [isReturned,leftStartArea] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,global_startPosition,leftStartArea,direction);
               [~,local_endPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
               area = [area pdist([local_startPosition(1),local_startPosition(2),local_startPosition(3);local_endPosition(1),local_endPosition(2),local_endPosition(3)],'euclidean')];
               if isReturned && leftStartArea
                   areaMeasured = true;
                   wallA = area(1);
                   wallB = area(2);
               end
           end
           
           % Start Seed Spreading
           
           % Calculate the amount of the necessary strips, the strips will
           % be paralell to the wallB
           divisionCount = ceil((wallA/Rvision));
           stripsCount = divisionCount + 1;
           divisionSize = (wallA/divisionCount)-0.25;
           
           % Check if the stripscount is odd or even (determins the end position)
           % odd - SW
           % even - SE
           isEven = ~mod(stripsCount,2);
           
           % Get into startposition (NE - corner)
           [~,~] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,[0 0 0],leftStartArea,direction);
           % Start position's coordinates
           [~,NEcorner]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
           % Goal position's coordinates (odd - SW, even - SE)
           SWcorner = [NEcorner(1)-wallB NEcorner(2)-wallA NEcorner(3)];
           SEcorner = [NEcorner(1) NEcorner(2)-wallA NEcorner(3)];
           if isEven
               GoalCorner = SEcorner;
           else
               GoalCorner = SWcorner;
           end
           
           % Main loop
           for i=1:1:stripsCount
               % Turns back of the wall go on strip E->W
               normalToWall = 2;
               % The turning direction depends on the side sensors
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(clientID,right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                   direction = -1;
               else
                   direction = 1;
               end
               turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(clientID,right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                  direction = 1;
                  [~,~] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,[0 0 0],leftStartArea,direction); 
               else
                   moveTilObstacle(clientID,left_Motor,right_Motor,front_LaserSensor);
               end
               
               % Check if the robot reached the GoalCorner
               isArrived = isNearby(clientID,pioneer_Robot,reference_Box,GoalCorner,1);
               if isArrived
                   break;
               end
               
               % Turns normal to wall go on strip N->S
               normalToWall = 1;
               turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
               [~,local_startPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
               inDivSize = isNearby(clientID,pioneer_Robot,reference_Box,local_startPosition,divisionSize);
               while inDivSize
                   inDivSize = isNearby(clientID,pioneer_Robot,reference_Box,local_startPosition,divisionSize);
                   move(velocity,clientID,left_Motor,right_Motor);
               end
               % Turns back of the wall go on strip W->E
               normalToWall = 2;
               % The turning direction depends on the side sensors
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(clientID,right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                   direction = -1;
               else
                   direction = 1;
               end
               turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(clientID,left_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                  direction = -1;
                  [~,~] = objectFollowing_controller(clientID,left_Motor,right_Motor,right_LaserSensor_front,front_LaserSensor,left_LaserSensor_front,pioneer_Robot,reference_Box,[0 0 0],leftStartArea,direction); 
               else
                   moveTilObstacle(clientID,left_Motor,right_Motor,front_LaserSensor);
               end
               
               % Check if the robot reached the GoalCorner
               isArrived = isNearby(clientID,pioneer_Robot,reference_Box,GoalCorner,1);
               if isArrived
                   break;
               end
               
               % Turns normal to wall go on strip N->S
               normalToWall = -1;
               turn(100,clientID,left_Motor,right_Motor,pioneer_Robot,front_LaserSensor,front_LaserSensor_rightAngle,front_LaserSensor_leftAngle,right_LaserSensor_front,right_LaserSensor_rear,left_LaserSensor_front,left_LaserSensor_rear,back_LaserSensor_right,back_LaserSensor_left,startOrientation,normalToWall,direction);
               [~,local_startPosition]=vrep.simxGetObjectPosition(clientID,pioneer_Robot,reference_Box,vrep.simx_opmode_blocking);
               inDivSize = isNearby(clientID,pioneer_Robot,reference_Box,local_startPosition,divisionSize);
               while inDivSize
                   inDivSize = isNearby(clientID,pioneer_Robot,reference_Box,local_startPosition,divisionSize);
                   move(velocity,clientID,left_Motor,right_Motor);
               end
           end
           

           
           
           move(0,clientID,left_Motor,right_Motor);
       elseif command == 0
           disp('The script exits...')
       end
      
       % Post simulation clean up
       vrep.simxFinish(-1);
 end
 
 vrep.delete();
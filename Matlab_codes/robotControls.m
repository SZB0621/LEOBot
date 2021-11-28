% Robot Control main
clearvars
close all
clear all

% Init section
vrep=remApi('remoteApi'); % Initializing the remote api file for coppelia
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5); % Setup connection parameters

% Global variables
global simulationHandlers_t;
simulationHandlers_t.clientID = clientID;
simulationHandlers_t.pioneer_Robot = NaN;
simulationHandlers_t.reference_Box = NaN;
simulationHandlers_t.left_Motor = NaN;
simulationHandlers_t.right_Motor = NaN;
simulationHandlers_t.front_LaserSensor = NaN;
simulationHandlers_t.front_LaserSensor_leftAngle = NaN;
simulationHandlers_t.front_LaserSensor_rightAngle = NaN;
simulationHandlers_t.right_LaserSensor_front = NaN;
simulationHandlers_t.right_LaserSensor_rear = NaN;
simulationHandlers_t.left_LaserSensor_front = NaN;
simulationHandlers_t.left_LaserSensor_rear = NaN;
simulationHandlers_t.back_LaserSensor_right = NaN;
simulationHandlers_t.back_LaserSensor_left = NaN;

% Local variables
velocity = 0.5;
turnVelocity = 0.6;
referenceDistance = 0.9; 
isStartOrientation = true; % If start orientation is 1 in turn function the robot will turn into startOrientation postition
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
if (simulationHandlers_t.clientID>-1)
       disp('Connection succesful...')
       
       while command ~= 1 && command ~= 0
           prompt = 'The script is ready for execution enter: 1 - to start the execution, 0 -to exit the script';
           command = input(prompt);
       end
       
       % Execution
       if command == 1
           % Setup Handlers
           initializeHandlers();
       
           % Turn and move into start position
           isStartOrientation = true; % If start orientation is 1 in turn function the robot will turn into 0 postition
           normalToWall = 0; % Values = [-1,0,1,2] 0 - normal turn function, -1/1 turn normal to the obstacle in front -1 turn right 1 turn left 2 back of the wall
           startOrientation = 0;
           turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
           isStartOrientation = false;
           moveTilObstacle();
           normalToWall = 1;
           turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
           [~, global_startPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
           direction = 1;
           [isReturned,leftStartArea,~] = objectFollowing_controller(global_startPosition,leftStartArea,direction,referenceDistance,[0 0],[0 0],false,false,[0 0]);
           
           % Measure the complete area
           while ~areaMeasured
               [~,local_startPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
               turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
               [isReturned,leftStartArea,~] = objectFollowing_controller(global_startPosition,leftStartArea,direction,referenceDistance,[0 0],[0 0],false,false,[0 0]);
               [~,local_endPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
               area = [area pdist([local_startPosition(1),local_startPosition(2),local_startPosition(3);local_endPosition(1),local_endPosition(2),local_endPosition(3)],'euclidean')];
               if isReturned && leftStartArea
                   areaMeasured = true;
                   wallA = area(1);
                   wallB = area(2);
               end
           end
           fprintf('Area measured! \n');
           
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
           [~,~,~] = objectFollowing_controller([0 0 0],leftStartArea,direction,referenceDistance,[0 0],[0 0],false,false,[0 0]);
           % Start position's coordinates
           [~,NEcorner]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
           % Goal position's coordinates (odd - SW, even - SE)
           SWcorner = [NEcorner(1)-wallB NEcorner(2)-wallA NEcorner(3)];
           SEcorner = [NEcorner(1) NEcorner(2)-wallA NEcorner(3)];
           if isEven
               GoalCorner = SEcorner;
           else
               GoalCorner = SWcorner;
           end
           fprintf('Start Position reached! \n');
           
           % Main loop
           for i=1:1:stripsCount
               % Turns back of the wall go on strip E->W
               normalToWall = 2;
               % The turning direction depends on the side sensors
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                   direction = -1;
               else
                   direction = 1;
               end
               turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                  direction = 1;
                  [~,~,~] = objectFollowing_controller([0 0 0],leftStartArea,direction,referenceDistance,[0 0],[0 0],false,false,[0 0]); 
               else
                   % Start of Object avoidance
                   [~, robotOrientationEuler]=vrep.simxGetObjectOrientation(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
                   robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
                   lineStartOrientation= robotOrientationEuler_deg(3);                        
                   [~, lineStartPoint]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                   [lineEndPoint] = calcLineEndPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,wallB);
                       reachedTheWall = false;
                       while ~reachedTheWall
                            moveTilObstacle();
                            [~, roundStartPoint]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                            dist = pdist([roundStartPoint(1),roundStartPoint(2),roundStartPoint(3);lineEndPoint(1),lineEndPoint(2),lineEndPoint(3)],'euclidean');

                            if dist >= 1
                                fprintf('IT REACHED THE OBSTACLE \n');
                                isEndPointGiven = false;
                                endpoint = [0,0];
                                [isReturned,closestPoint] = roundAnObstacle(roundStartPoint,lineStartPoint,lineEndPoint,isEndPointGiven,endpoint);
% %                             Placeholder for obstacle avoidance function
                                fprintf('CIRCLE DONE \n');
                                isEndPointGiven = true;
                                endpoint = [closestPoint(1) closestPoint(2) 0];
                                isStartOrientation = true;
                                startOrientation = lineStartOrientation;
                                normalToWall = 0;
                                % The turning direction depends on the side sensors
                                [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
                                if dState
                                    phi = -1;
                                else
                                    phi = 1;
                                end
                                turn(phi,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
                                moveTilObstacle();
                                % Turn and move to obs
                                [isReturned,closestPoint] = roundAnObstacle(roundStartPoint,lineStartPoint,lineEndPoint,isEndPointGiven,endpoint);
                                fprintf('OUT POINT REACHED \n');
                                % Turn to the line end point
                                [~, outPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                                startOrientation = 180-atand((lineEndPoint(2)-outPosition(2))/(lineEndPoint(1)-outPosition(1)));
                                turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);    
                            else
                                fprintf('IT REACHED THE WALL \n');
                                reachedTheWall = true;
                            end
                       end
                       % End of object avoidance
               end
               
               % Check if the robot reached the GoalCorner
               isArrived = isNearby(GoalCorner,2);
               if isArrived
                   break;
               end
               
               % Turns normal to wall go on strip N->S
               normalToWall = 1;
               turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
               [~,local_startPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
               inDivSize = isNearby(local_startPosition,divisionSize);
               while inDivSize
                   inDivSize = isNearby(local_startPosition,divisionSize);
                   [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.front_LaserSensor,vrep.simx_opmode_blocking);
                   if dState
                       break;
                   end
                   move(velocity);
               end
               % Turns back of the wall go on strip W->E
               normalToWall = 2;
               % The turning direction depends on the side sensors
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                   direction = -1;
               else
                   direction = 1;
               end
               turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
               [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.left_LaserSensor_front,vrep.simx_opmode_blocking);
               if dState
                   direction = -1;
                   [~,~,~] = objectFollowing_controller([0 0 0],leftStartArea,direction,referenceDistance,[0 0],[0 0],false,false,[0 0]); 
               else
                   % Start of Object avoidance
                   [~, robotOrientationEuler]=vrep.simxGetObjectOrientation(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,vrep.sim_handle_parent,vrep.simx_opmode_blocking);
                   robotOrientationEuler_deg = rad2deg(robotOrientationEuler);
                   lineStartOrientation= robotOrientationEuler_deg(3);                        
                   [~, lineStartPoint]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                   [lineEndPoint] = calcLineEndPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,wallB);
                       reachedTheWall = false;
                       while ~reachedTheWall
                            moveTilObstacle();
                            [~, roundStartPoint]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                            dist = pdist([roundStartPoint(1),roundStartPoint(2),roundStartPoint(3);lineEndPoint(1),lineEndPoint(2),lineEndPoint(3)],'euclidean');

                            if dist >= 1
                                fprintf('IT REACHED THE OBSTACLE \n');
                                isEndPointGiven = false;
                                endpoint = [0,0];
                                [isReturned,closestPoint] = roundAnObstacle(roundStartPoint,lineStartPoint,lineEndPoint,isEndPointGiven,endpoint);
% %                             Placeholder for obstacle avoidance function
                                fprintf('CIRCLE DONE \n');
                                isEndPointGiven = true;
                                endpoint = [closestPoint(1) closestPoint(2) 0];
                                isStartOrientation = true;
                                startOrientation = lineStartOrientation;
                                normalToWall = 0;
                                % The turning direction depends on the side sensors
                                [~,dState,~,~,~]=vrep.simxReadProximitySensor(simulationHandlers_t.clientID,simulationHandlers_t.right_LaserSensor_front,vrep.simx_opmode_blocking);
                                if dState
                                    phi = -1;
                                else
                                    phi = 1;
                                end
                                turn(phi,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
                                moveTilObstacle();
                                % Turn and move to obs
                                [isReturned,closestPoint] = roundAnObstacle(roundStartPoint,lineStartPoint,lineEndPoint,isEndPointGiven,endpoint);
                                fprintf('OUT POINT REACHED \n');
                                % Turn to the line end point
                                [~, outPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
                                startOrientation = atand((lineEndPoint(2)-outPosition(2))/(lineEndPoint(1)-outPosition(1)));
                                turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);    
                            else
                                fprintf('IT REACHED THE WALL \n');
                                reachedTheWall = true;
                            end
                       end
                       % End of object avoidance
               end
               
               % Check if the robot reached the GoalCorner
               isArrived = isNearby(GoalCorner,2);
               if isArrived
                   break;
               end
               
               % Turns normal to wall go on strip N->S
               normalToWall = -1;
               turn(100,isStartOrientation,startOrientation,normalToWall,direction,turnVelocity);
               [~,local_startPosition]=vrep.simxGetObjectPosition(simulationHandlers_t.clientID,simulationHandlers_t.pioneer_Robot,simulationHandlers_t.reference_Box,vrep.simx_opmode_blocking);
               inDivSize = isNearby(local_startPosition,divisionSize);
               while inDivSize
                   inDivSize = isNearby(local_startPosition,divisionSize);
                   move(velocity);
               end
           end
           

           
           
           move(0);
       elseif command == 0
           disp('The script exits...')
       end
      
       % Post simulation clean up
       vrep.simxFinish(-1);
 end
 
 vrep.delete();
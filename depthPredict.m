function[depth] = depthPredict(robotPose,map,sensorOrigin,angles)
% DEPTHPREDICT: predict the depth measurements for a robot given its pose
% and the map
%
%   DEPTH = DEPTHPREDICT(ROBOTPOSE,MAP,SENSORORIGIN,ANGLES) returns
%   the expected depth measurements for a robot 
%
%   INPUTS
%       robotPose   	3-by-1 pose vector in global coordinates (x,y,theta)
%       map         	N-by-4 matrix containing the coordinates of walls in
%                   	the environment: [x1, y1, x2, y2]
%       sensorOrigin	origin of the sensor-fixed frame in the robot frame: [x y]
%       angles      	K-by-1 vector of the angular orientation of the range
%                   	sensor(s) in the sensor-fixed frame, where 0 points
%                   	forward. All sensors are located at the origin of
%                   	the sensor-fixed frame.
%
%   OUTPUTS
%       depth       	K-by-1 vector of depths (meters)
%
%
%   Cornell University
%   Autonomous Mobile Robots
%   Homework 1
% LOPEZ, DELANEY

% create variables for pose
theta = robotPose(3);
sensorX = robotPose(1)+sensorOrigin(1)*cos(theta)-sensorOrigin(2)*sin(theta);
sensorY = robotPose(2)+sensorOrigin(1)*sin(theta)+sensorOrigin(2)*cos(theta);
sensorPose = [sensorX, sensorY, theta];

% define max depth for function to register walls
max_depth = 10;
depth = max_depth*ones(length(angles),1);

% gets current angle to create a line from sensor along scan angle
for i = 1:length(angles)
    ang = theta+angles(i);
    
    x1 = sensorX;
    y1 = sensorY;
    x2 = sensorX+max_depth*cos(ang);
    y2 = sensorY+max_depth*sin(ang);

    minDepth = max_depth;

    % selects points from map data
    for m = 1:size(map,1)
    x3 = map(m,1);
    y3 = map(m,2);
    x4 = map(m,3);
    y4 = map(m,4);
    
    % checks if wall lines and sensor lines intersect
    [isect,x,y]= intersectPoint(x1,y1,x2,y2,x3,y3,x4,y4);
        
        if isect == true
            wallX = x;
            wallY = y;
        
            depth_intersect = (wallX-sensorX)*cos(ang)+(wallY-sensorY)*sin(ang);
        
            if depth_intersect > 0 && depth_intersect < minDepth
                minDepth = depth_intersect;
            end
        end
    end
    % updates vector with intersect points
    depth(i) = minDepth;
end


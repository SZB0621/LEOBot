function [closestPoint,minimalDistance] = closestPointFromLine(point,prevPoint,StartPoint,lineStartPoint,lineEndPoint,prevDist)

    closestPoint = [];
    
    DistanceFromLine = GetPointLineDistance(point(1),point(2),lineStartPoint(1),lineStartPoint(2),lineEndPoint(1),lineEndPoint(2));
    DistanceFromStartPoint = pdist([point(1),point(2),0;StartPoint(1),StartPoint(2),0],'euclidean');
%     fprintf('DistanceFromLine: %.4f, DistanceFromStartPoint: %.4f \n',DistanceFromLine,DistanceFromStartPoint);
    if (DistanceFromLine < prevDist && DistanceFromStartPoint > 2)
        minimalDistance = DistanceFromLine;
        closestPoint(1) = point(1);
        closestPoint(2) = point(2);
    else
        minimalDistance = prevDist;
        closestPoint(1) = prevPoint(1);
        closestPoint(2) = prevPoint(2);        
    end    
end

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
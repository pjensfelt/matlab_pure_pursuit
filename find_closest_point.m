% Usage: [xMin,yMin,dMin,index,s] = find_closest_point(x,y,Xp,Yp,interpolate)
% Inputs:
% x : query points x-coord
% y : query points y-coord
% Xp : waypoints x-coords
% Yp : waypoints y-coords
% interpolate : if 0 we will return the closest waypoint, otherwise also
% considering all points on edges between waypoints
% minIndex : if specified consider only points with index >= minIndex
% Outputs
% xMin : closest point x-coord
% yMin : closest point y-coord
% dMin : closest distance
% index : index of closest waypoint
% s : distance from closest waypoint to closest point towards previous
% waypoint (s<0) or towards next waypoint (s >0)
%
% This code comes as is without any guarantees that it works as intended
% (c) 2017 Patric Jensfelt
function [xMin,yMin,dMin,index,s] = find_closest_point(x,y,Xp,Yp,interpolate,minIndex)

if nargin < 6
    minIndex = 1;
end
if nargin < 5
    interpolate = 0;
end

s = 0;

dd = hypot(Xp-x,Yp-y);
[dMin,index] = min(dd(minIndex:end));
index = index + (minIndex -1);
xMin = Xp(index);
yMin = Yp(index);

% If we asked for interpolation we will look for the closest point also
% including the points on the edges between the waypoints
% We know which of the waypoints is the cloest, now we will check if there
% is a point on the edge leading in to or out from this waypoint.
if interpolate == 0
    disp('returning early')
    return;
end

% Check leading in first
if index > 1
    %disp(sprintf('Checking edge leading in to point with index %f',index))
    % Create unit vector from current to previous waypoint
    dx = Xp(index-1) - Xp(index);
    dy = Yp(index-1) - Yp(index);
    L = hypot(dx,dy);
    unitVec = [dx, dy] / L;

    % Calculate the scalar product between the above unit vector 
    % and a vector from the current waypoint to the point we 
    % are checking against
    % This gives us information about where on the line segment the
    % query point (x,y) is projected
    dx = x - Xp(index);
    dy = y - Yp(index);
    ss = unitVec(1)*dx + unitVec(2)*dy;
    if (ss > 0 && ss < L)
        % A point on the line segment will be closer to the query point
        % that the waypoint
    
        % Calculate the distance from the query point to the line segment
        % We get this using the scalar product with the unitVector rotated
        % 90 degrees which we get by swapping x and y and changing sign on
        % y
        d = abs(-unitVec(2)*dx + unitVec(1)*dy);
        if d < dMin
            %disp('New closest point, to leading in edge')
            % Store the closest point
            % We give the s parameter a minus sign to signify that the 
            % closest point is on an edge to the previos waypoint
            dMin = d;
            xMin = Xp(index) + ss*unitVec(1);
            yMin = Yp(index) + ss*unitVec(2);
            s = -ss;
        end
    end
end

% Check leading out from 
if index < length(Xp)
    %disp(sprintf('Checking edge leading out from point with index %f',index))
    % Create unit vector from current to next waypoint
    dx = Xp(index+1) - Xp(index);
    dy = Yp(index+1) - Yp(index);
    L = hypot(dx,dy);
    unitVec = [dx, dy] / L;

    % Calculate the scalar product between the above unit vector 
    % and a vector from the current waypoint to the point we 
    % are checking against
    % This gives us information about where on the line segment the
    % query point (x,y) is projected
    dx = x - Xp(index);
    dy = y - Yp(index);
    ss = unitVec(1)*dx + unitVec(2)*dy;
    if (ss > 0 && ss < L)
        % A point on the line segment will be closer to the query point
        % that the waypoint
    
        % Calculate the distance from the query point to the line segment
        % We get this using the scalar product with the unitVector rotated
        % 90 degrees which we get by swapping x and y and changing sign on
        % y
        d = abs(-unitVec(2)*dx + unitVec(1)*dy);
        if d < dMin
            %disp('New closest point, to leading out edge')
            % Store the closest point
            dMin = d;
            xMin = Xp(index) + ss*unitVec(1);
            yMin = Yp(index) + ss*unitVec(2);
            s = ss;
        end
    end
end
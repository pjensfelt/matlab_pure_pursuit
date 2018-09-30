% This code comes as is without any guarantees that it works as intended
% (c) 2017 Patric Jensfelt
function [xT,yT] = get_lookahead_point(x,y,D,Xp,Yp);

if length(Xp) < 2
    disp('Need at least 2 waypoints in get_lookahead_point!!!')
    return
end

% We iterate from waypoint to waypoint and add distances until we get to
% the desired lookahead infront of the current position. 
% We decrement D in eveyr step with the distance we moved so far
index = 1;
while true
    [xMin,yMin,junk,index,s] = find_closest_point(x,y,Xp,Yp,1,index);
    %disp(sprintf('D=%fm index=%d x=%f y=%f s=%f',D,index,x,y,s))

    % The closest point is the last waypoint
    if index == length(Xp)
        % Get unit vector from second last to the last waypoint
        dx = Xp(index) - Xp(index-1);
        dy = Yp(index) - Yp(index-1);
        L = hypot(dx,dy);
        unitVec = [dx dy]/L;
        xT = x + unitVec(1)*D;
        yT = y + unitVec(2)*D;
        %disp('Returning becase we reached last waypoint')
        return;
    end

    if s < 0 
        %disp('Dealing with s<0')
        % We are not yet at the next way point
        if D < abs(s)
            dx = Xp(index) - Xp(index-1);
            dy = Yp(index) - Yp(index-1);
            L = hypot(dx,dy);
            unitVec = [dx dy]/L;
            xT = x + unitVec(1)*D;
            yT = y + unitVec(2)*D;
            %disp(sprintf('Returning because we did not reach currently closest waypoint s=%f',s))
            return;
        else
            D = D + s;
            x = Xp(index);
            y = Yp(index);
        end
    else
        %disp('Dealing with s>=0')
        dx = Xp(index+1) - Xp(index);
        dy = Yp(index+1) - Yp(index);
        L = hypot(dx,dy);
        unitVec = [dx dy]/L;
        if D < L-s
            xT = x + unitVec(1)*D;
            yT = y + unitVec(2)*D;
            %disp(sprintf('Returning because we did not reach next waypoint, L=%f',L))
            return;
        else
            D = D - (L-s);
            index = index + 1;
            x = Xp(index);
            y = Yp(index);
        end
    end
end
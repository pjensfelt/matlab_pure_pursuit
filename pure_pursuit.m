%
% Author: Patric Jensfelt, patric@kth.se
%
function pure_pursuit(lookAhead, pathfile)

if nargin < 1
    lookAhead = 0.1;
end

if nargin < 2
    pathfile = 'path.mat';
end

% Load the path
load(pathfile, 'Xp','Yp');

% Dynamic limits
transAccMax = 2; % m/s^2
rotAccMax = 360*(pi/180); % rad/s^2

% We start from 0,0,0 with translation speed v=0m/s and rotational speed
% w=0rad/s
x = 0;
y = 0;
a = 0;
v = 0;
w = 0;
t = 0;
X = [x];
Y = [y];
A = [a];
V = [v];
W = [w];
[junk,junk,D] = find_closest_point(x,y,Xp,Yp,1);

T = 10;
dt = 0.01;
Nsim = 10;
index = 1;
% We run the simulation for at most T long or until the closest point is the last waypoint
while true
    % Get the closest point in the path
    [xMin,yMin,d,index,s] = find_closest_point(x,y,Xp,Yp,1,index);
    
    if index == length(Xp)
        disp(sprintf('Reached the last waypoint at t=%fs with max lateral error %fm',t(end),max(D)))
        break;
    end

    % Calculate the control signals, ie the desired translation and 
    % rotation speeds. We cannot set the actual translation and rotation
    % speeds because accelerations are finite
    [vRef, wRef] = calcCtrl(lookAhead);
    
    % Run the actual simulation where we incorporate limitations in 
    % accelerations
    ddt = dt / Nsim;
    for k = 1:Nsim
        accV = (vRef - v) / ddt;
        if accV > transAccMax; accV = transAccMax; end
        if accV < -transAccMax; accV = -transAccMax; end
        v = v + accV*ddt;
        
        accW = (wRef - w) / ddt;
        if accW > rotAccMax; accW = rotAccMax; end
        if accW < -rotAccMax; accW = -rotAccMax; end
        w = w + accW*ddt;
        
        x = x + v*cos(a)*ddt;
        y = y + v*sin(a)*ddt;
        a = a + w*dt;
    end
    
    % Store data for plotting
    A = [A a];
    X = [X x];
    Y = [Y y];
    t = [t t(end)+dt];
    V = [V v];
    W = [W w];
    D = [D,d];
    
    figure(1)
    plot(Xp,Yp,xTarget,yTarget,'o');
    hold on
    display_robot(x,y,a,'k',1);
    aa = (0:360)*pi/180;
    plot(x+lookAhead*cos(aa), y+lookAhead*sin(aa), 'r')
    hold off
    axis equal
    drawnow
    
    if t(end) >= T
        disp(sprintf('Reached end of simualtion T=%fs',t(end)))
        break;
    end
end

figure(2)
plot(Xp,Yp,'k',X,Y,'r')
figure(3)
subplot(3,1,1),plot(t,V),legend('speed [m/s]')
subplot(3,1,2),plot(t,W*180/pi),legend('rotation speed [deg/s]')
subplot(3,1,3),plot(t,D),legend('Distance to path [m]')

function [vRef, wRef] = calcCtrl(lookAhead)
    % We calculate a point xTraget,yTarget which is a distance lookAhead 
    % further along the path and steer towards it.
    % The further our current angle is from the desired angle the slower
    % We move
    vMax = 1;
    kP = 1;
    [xTarget,yTarget] = get_lookahead_point(xMin,yMin,lookAhead,Xp,Yp);
    aRef = atan2(yTarget - y, xTarget - x);
    aErr = aRef - a;
    if (aErr > pi); aErr = aErr - 2*pi; end
    if (aErr <-pi); aErr = aErr + 2*pi; end
    vRef = vMax*exp(-0.5*(aErr/(60*pi/180))^2);
    wRef = kP*aErr;
    [vRef wRef];
end
end

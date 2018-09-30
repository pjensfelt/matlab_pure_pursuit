function [h] = display_robot(x,y,a,c,drawDirectionVector)

if nargin < 4
    c = 'r';
end

if nargin < 5
    drawDirectionVector = 0;
end

R = 0.2;
aa = (0:360)*pi/180;
X = [R*cos(aa);R*sin(aa)];

h = plot(x+X(1,:),y+X(2,:),c,'LineWidth',2);
h = [h plot(x,y,'x')];
if drawDirectionVector
    h = [h plot([x+cos(a)*[R (R+0.5)]],[y+sin(a)*[R (R+0.5)]],c,'LineWidth',2)];
end
function samplepath(fname)

if nargin < 1
    fname = 'path.mat';
end

path(path, 'freehanddraw/');

disp('Draw the path, start in the center (will be corrected oytherwise)')
figure(1), clf;
hold on
display_robot(0,0,0,'k',true);
axis(3*[-1 1 -1 1])
disp('Middle click to stop input')
[tmp,Xp,Yp]=freehanddraw();
hold off

% Remove first and last point as they seem to be duplicated
Xp = Xp(2:(end-1));
Yp = Yp(2:(end-1));


% Remove all points where there is not enough movements (<0.01m)
dd = hypot(diff(Xp),diff(Yp));
pointsToUse = find(dd>0.01);
Xp = Xp(pointsToUse);
Yp = Yp(pointsToUse);

% Make sure the trajectory start in the center
Xp = Xp - Xp(1);
Yp = Yp - Yp(1);

disp(sprintf('Sampled %d points', length(Xp)))
save(fname, 'Xp', 'Yp')

clf
hold on
display_robot(0,0,0,'k',true);
plot(Xp,Yp,'r',Xp,Yp,'bx')
hold off
axis equal
%TestingLine Publishing:
h = GazeboMatlabSimulator;
%%
t = 0:0.01:1;
omega = 1;
r = 1.5;
m = MarkerInfo;
m.action = m.ADD;
m.color = [0,0,1,1];
d = [r*cos(2*pi*omega*t);r*sin(2*pi*omega*t);0.1*ones(1,length(t))];%Somehow these are in centimeters
for count = 1:size(d,2)
 h.PublishTrajectory(d(:,count),m);
pause(0.01);
end
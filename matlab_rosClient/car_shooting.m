function f = gazebocar_shooting()
% Example of trajectory optimization for a 
% WAM Model using shooting and least-squares Gauss Newton method
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 5;
S.N = 8;
S.h = tf/S.N;
S.m = 2;
S.n = 4;

% cost function parameters
S.Q = .0*diag([5, 5, 1, 1]);
S.R = .01*diag([1, 1]);
S.Qf = diag([10, 10, 10, 1]);

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

S.sim = Gazebo_MatlabSimulator;
S.sim.Configure(0.001,20);
S.steps = uint32(round((0:S.h:tf)/S.sim.physxtimestep));

x0 = [0; 0; pi/2; 0];
S.xf = [2.5; 2; pi/2; 0];%Posn(x,y),Angle, Body Velocity
S.x0 = x0;

% initial control sequence
us = zeros(2,S.N);
us(1,:) = 0.2;
us(2,:) = 0.01;
xs = zeros(4,S.N+1);

figure(1),clf,
S.phandle(1) = plot(0,0);
hold on, S.phandle(2) = plot(0,0,'r');
hold on, S.phandle(3) = plot(0,0,'g');
set(S.phandle(1),'XData',S.steps);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.steps);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',S.steps);
set(S.phandle(3),'YDataSource','xs(3,:)');%Tip Positions
figure(2),clf,
S.phandle(4) = plot(0,0);
hold on, S.phandle(5) = plot(0,0,'r');
set(S.phandle(4),'XData',(S.steps(1:end-1)));
set(S.phandle(4),'YDataSource','us(1,:)');
set(S.phandle(5),'XData',(S.steps(1:end-1)));
set(S.phandle(5),'YDataSource','us(2,:)');

m = size(us,1);
N = S.N;
lb=repmat([-2; -1], N, 1); % can incorporate upper and lower bound
ub=repmat([2; 1], N, 1);
options = optimset('FinDiffRelStep',0.001, 'TolX',1e-2,...
                    'TolFun',1e-3, 'MaxFunEvals',200);%Will change later
clc
us = lsqnonlin(@(us)arm_cost(us, S), us, lb, ub,options);
disp('us');
disp(us);
S.sim.Configure(0.001,1);
pause;
% % update trajectory
xs = sys_traj(x0, us, S);
% 
disp('xs');
disp(xs);
% figure(3),
% plot(S.steps, xs(1,:), '-b');
% hold on, plot(S.steps, xs(2,:), '-r');
% 
% y = arm_cost(us, S);
% J=y'*y/2;
% 
% disp(['cost=' num2str(J)])
% 
% xlabel('x')
% ylabel('y')
% 
% figure(4),
% plot(0:S.h:tf-S.h, us(1,:),0:S.h:tf-S.h, us(2,:));
% xlabel('sec.')
% legend('u_1','u_2')


function y = arm_cost(us, S)
% this the arm costs in least-squares form,
% i.e. the residuals at each time-step

us = reshape(us, S.m, S.N);

xs = sys_traj(S.x0, us, S);

N=size(us,2);

%y = zeros(S.N*(S.n+S.m), 1);
y=[];
for k=1:N
  y = [y; S.Rs*us(:,k)];
end

for k=2:N
  y = [y; S.Qs*xs(:,k)];
end
y = [y; S.Qfs*(xs(:,N+1)-S.xf)];
disp('Cost: ');
disp(0.5*y'*y);
 
function xs = sys_traj(x0, us, S)

N = size(us, 2);
%xs(:,1) = x0;
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);
%Set car to initial posn:
modelposeandtwist = [x0(1:2)' 0.05 rpy2quat([x0(3), 0, 0])' x0(4) zeros(1,5)];%13x1
mex_mmap('setmodelstate',S.sim.Mex_data,'Unicycle',modelposeandtwist);
pause(0.01);
jointids = 1:4;%All Joints are actuated
us = [us;us];%Copy us two times for both left and right hand side controls
[LinkData,~] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, us, ...
                                                [], [], S.steps);
xs(1:2,:) = LinkData(1:2,:);
xs(3,:) = atan2(2*(LinkData(4,:).*LinkData(7,:) + LinkData(5,:).*LinkData(6,:)), 1 - ...
                    2*(LinkData(6,:).^2 + LinkData(7,:).^2));%Yaw computation
xs(4,:) = LinkData(8,:);

if norm(xs(:,1) - S.x0(:,1))>1e-3
    disp('xs is not starting from x0');
end
refreshdata(S.phandle,'caller');
pause(0.01);


function f = wam_shooting()
% Example of trajectory optimization for a 
% WAM Model using shooting and least-squares Gauss Newton method
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 1;
S.N = 10;
S.h = tf/S.N;
S.m = 7;%7 Joints
S.n = 6;%The tip position and tip velocity (Linear only)

% cost function parameters
S.Q = diag([0, 0, 0, 1, 1, 1, 1, 1, 1]);%Penalize tip velocity
S.R = 0*diag([0.1, 0.05, 0.025, 0.0125, 0.0125, 0.0125, 0.0125]);%Joint Torque costs (EXPERIMENT: Farther the joint easier to torque it)
S.Qf = diag([1000, 1000, 1000, 1, 1, 1, 1, 1, 1]);%Final tip position and velocity

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

S.sim = Gazebo_MatlabSimulator;
S.sim.Configure(0.001,20);
S.steps = uint32(round((0:S.h:tf)/S.sim.physxtimestep));

% initial state
%x0 = [-5; -2; -1.2; 0; 0];
x0 = [1.2462; 0; 0.7810; 0; 0; 0; 0; 0; 0];%Tip position for reset
S.xf = [0.9; 0.2; 0.7810; 0; 0; 0; 0; 0; 0];%Goal tip position (Linear and Angular Vel)

%S.xf = [0.8; 0.2; 0.5; 0; 0; 0; 0; 0; 0];%Goal tip position (Linear and Angular Vel)

S.x0 = x0;

% initial control sequence
%us = 0*ones(2,S.N);
us = zeros(7,S.N);%7 Joint Torques (Initial Guess)
%xs = zeros(6,S.N+1);

figure(1),
S.phandle(1) = plot(0,0);
hold on, S.phandle(2) = plot(0,0,'r');
hold on, S.phandle(3) = plot(0,0,'g');
set(S.phandle(1),'XData',S.steps);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.steps);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',S.steps);
set(S.phandle(3),'YDataSource','xs(3,:)');%Tip Positions
% figure(2),
% S.phandle(3) = plot(0,0);
% hold on, S.phandle(4) = plot(0,0,'r');
% set(S.phandle(3),'XData',(S.steps(1:end-1)));
% set(S.phandle(3),'YDataSource','us(1,:)');
% set(S.phandle(4),'XData',(S.steps(1:end-1)));
% set(S.phandle(4),'YDataSource','us(2,:)');

%Update Trajectory
%xs = sys_traj(x0, us, S);

%subplot(1,2,1)

%plot(xs(1,:), xs(2,:), '-b')
hold on

m = size(us,1);
N = S.N;
lb=repmat(-30*ones(7,1), N, 1); % can incorporate upper and lower bound
ub=repmat(30*ones(7,1), N, 1);
options = optimset('FinDiffRelStep',0.0001, 'TolX',1e-2,...
                    'TolFun',1e-3, 'MaxFunEvals',400);%Will change later
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
mex_mmap('stringreq',S.sim.Mex_data,'worldreset');
pause(0.01);%Testing
%mex_mmap('setjointstate',S.sim.Mex_data,2,[x0(2),0,0,x0(4),0,0]);
%mex_mmap('setjointstate',S.sim.Mex_data,1,[x0(1),0,0,x0(3),0,0]);%TODO Combine into one call
jointids = 1:7;%All Joints are actuated
[LinkData, ~] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, us, ...
                                                [], [], S.steps);
xs = LinkData([1:3 8:13],:);%Tip Position and Tip Velocities
%xs = sd_round(xs,4);%Round upto N significant digits
if norm(xs(:,1) - S.x0(:,1))>1e-3
    disp('xs is not starting from x0');
end
refreshdata(S.phandle,'caller');
pause(0.01);


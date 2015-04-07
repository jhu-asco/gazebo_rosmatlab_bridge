function f = quadcopter_shooting()
%QUADCOPTER_SHOOTING Example of trajectory optimization for a quadrotor using GN shooting
% car-like model using shooting and least-squares Gauss Newton method
%
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 3;
S.N = 10;
S.h = tf/S.N;
S.m = 4;
S.n = 7;

% cost function parameters
S.Q = .0*diag([0, 0, 0, 1, 1, 1, 0]);
S.R = .01*diag([0.1, 0.1, 0.1, 0.1]);
S.Qf = diag([10, 10, 10, 5, 5, 5, 1]);

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

S.f = @arm_f;
S.sim = GazeboMatlabSimulator;
S.sim.Configure(0.001,100);
S.steps = uint32(round((0:S.h:tf)/S.sim.PhysicsTimeStep));

% initial state
%x0 = [-5; -2; -1.2; 0; 0];
x0 = [0; 0; 0.182466; 0; 0; 0; 0;];%x y z vx vy vz yaw
S.xf = [0.2; 0.3; 1; 0.2; 0.3; 0; 1];%Final state

S.x0 = x0;

% initial control sequence
us = [zeros(3,S.N); 15*ones(1,S.N)];

figure(1),
S.phandle(1) = plot(0,0,'r');
hold on, S.phandle(2) = plot(0,0,'g');
hold on, S.phandle(3) = plot(0,0,'b');
set(S.phandle(1),'XData',S.steps);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.steps);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',S.steps);
set(S.phandle(3),'YDataSource','xs(3,:)');

%Update Trajectory
%xs = sys_traj(x0, us, S);

%subplot(1,2,1)

%plot(xs(1,:), xs(2,:), '-b')
hold on

m = size(us,1);
N = S.N;
lb=repmat([-0.1; -0.1; -0.1; 0], N, 1); % can incorporate upper and lower bound
ub=repmat([0.1; 0.1; 0.1; 16], N, 1);
options = optimset('FinDiffRelStep',0.0001, 'TolX',1e-2,'TolFun',1e-4);
us = lsqnonlin(@(us)arm_cost(us, S), us, lb, ub,options);
disp('us');
disp(us);
pause;%Lets u run the optimal one
S.sim.Configure(0.001,1);
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
%Can also set x0 for quadcopter
N = size(us, 2);
%xs(:,1) = x0;
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);%Testing
%mex_mmap('setjointstate',S.sim.Mex_data,2,[x0(2),0,0,x0(4),0,0]);
%mex_mmap('setjointstate',S.sim.Mex_data,1,[x0(1),0,0,x0(3),0,0]);%TODO Combine into one call
LinkWrench = [zeros(2,N); us(4,:); us(1:3,:)];
[LinkData, ~] = mex_mmap('runsimulation',S.sim.Mex_data,[],[], ...
                                                    uint32(0), LinkWrench, S.steps);
xs(1:6,:) = LinkData([1:3 8:10],:);%Only 1 link
xs(7,:) = atan2(2*(LinkData(4,:).*LinkData(7,:) + LinkData(5,:).*LinkData(6,:)), 1 - ...
                    2*(LinkData(6,:).^2 + LinkData(7,:).^2));%Yaw computation
if norm(xs(:,1) - S.x0(:,1))>1e-3
    disp('xs is not starting from x0');
end
refreshdata(S.phandle,'caller');
pause(0.01);


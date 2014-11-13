function f = arm_shooting()
% Example of trajectory optimization for a 
% car-like model using shooting and least-squares Gauss Newton method
%
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 1;
S.N = 4;
S.h = tf/S.N;
S.m = 2;
S.n = 4;

% cost function parameters
S.Q = .0*diag([5, 5, 1, 1]);
S.R = .01*diag([1, 0.1]);
S.Qf = diag([100, 100, 20, 20]);

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

S.f = @arm_f;
S.sim = Gazebo_MatlabSimulator;
S.sim.Configure(0.001,100);
S.steps = uint32(round((0:S.h:tf)/S.sim.physxtimestep));

% initial state
%x0 = [-5; -2; -1.2; 0; 0];
x0 = [-1; 0; 0; 0;];
S.xf = [1; 0; 0; 0];

S.x0 = x0;

% initial control sequence
us = 0*ones(2,S.N);

% us = [0.9008   10.4779   13.3308    2.9975;
%     9.3994    0.0811   -6.0542    5.9867];
%us =[25*ones(1,S.N); 5*ones(1,S.N)];
xs = zeros(4,S.N+1);

figure(1),
S.phandle(1) = plot(0,0);
hold on, S.phandle(2) = plot(0,0,'r');
set(S.phandle(1),'XData',S.steps);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.steps);
set(S.phandle(2),'YDataSource','xs(2,:)');
figure(2),
S.phandle(3) = plot(0,0);
hold on, S.phandle(4) = plot(0,0,'r');
set(S.phandle(3),'XData',(S.steps(1:end-1)));
set(S.phandle(3),'YDataSource','us(1,:)');
set(S.phandle(4),'XData',(S.steps(1:end-1)));
set(S.phandle(4),'YDataSource','us(2,:)');

%Update Trajectory
%xs = sys_traj(x0, us, S);

%subplot(1,2,1)

%plot(xs(1,:), xs(2,:), '-b')
hold on

m = size(us,1);
N = S.N;
lb=repmat([-50; -50], N, 1); % can incorporate upper and lower bound
ub=repmat([50; 50], N, 1);
options = optimset('FinDiffRelStep',0.01, 'TolX',1e-2,'TolFun',1e-4);
us = lsqnonlin(@(us)arm_cost(us, S), us, lb, ub,options);
disp('us');
disp(us);
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
jointids = [1 2];
%xs(:,1) = x0;
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);
modelposeandtwist = [zeros(3,1);1;zeros(9,1)];
mex_mmap('setmodelstate',S.sim.Mex_data,'double_pendulum_with_base',modelposeandtwist,...
    uint32(jointids)-1,[x0(1:2);x0(3:4)]);

[~, JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, us, ...
                                                    [], [], S.steps);
xs([1,3],1:(N+1)) = JointData(:,1:2:(2*(N+1)));
xs([2,4],1:(N+1)) = JointData(:,2:2:(2*(N+1)));
A = rem(xs(1:2,:),2*pi);
A(A>pi) = A(A>pi)-2*pi;
A(A<-pi) = A(A<-pi)+2*pi;
xs(1:2,:)= A;
%xs = sd_round(xs,4);%Round upto N significant digits
if norm(xs(:,1) - S.x0(:,1))>1e-3
    disp('xs is not starting from x0');
end
refreshdata(S.phandle,'caller');
pause(0.01);


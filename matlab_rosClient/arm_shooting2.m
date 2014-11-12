function f = arm_shooting2()
% Example of trajectory optimization for a 
% car-like model using shooting and least-squares Gauss Newton method
%
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 1;
S.N = 2;
S.h = 0.001;
S.m = 2;
S.n = 4;

%System Parameters:
S.m1 = 1;
S.m2 = 1;
S.l1 = 1;
S.l2 = 1;
S.lc1 = .5;
S.lc2 = .5;
S.I1 = S.m1*(S.l1)/12;
S.I2 = S.m2*(S.l2)/12;
S.g = 9.81;

% cost function parameters
S.Q = .0*diag([5, 5, 1, 1]);
S.R = .1*diag([1, 5]);
S.Qf = diag([100, 100, 1, 1]);

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);
S.physxtimestep = 0.001;%Time step for propagating dynamics

S.f = @arm_f;
S.steps = uint32(round((0:(tf/S.N):tf)/S.h));

% initial state
%x0 = [-5; -2; -1.2; 0; 0];
x0 = [0; 0; 0; 0;];

S.x0 = x0;
S.xf = [1.57; 0; 0; 0];
% initial control sequence
us = 0*ones(2,S.N);
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

us = lsqnonlin(@(us)arm_cost(us, S), us, lb, ub);

% % update trajectory
% xs = sys_traj(x0, us, S);
% 
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
% plot(S.steps(1:end-1), us(1,:),S.steps(1:end-1), us(2,:));
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

function [x, A, B] = arm_f(k, x, u, S)
% arm discrete dynamics
% set jacobians A, B to [] if unavailable
%
% the following parameters should be set:
% S.m1  : mass of first body
% S.m2  : mass of second body
% S.l1  : length of first body
% S.l2  : length of second body
% S.lc1 : distance to COM
% S.lc2 : distance to COM
% S.I1  : inertia 1
% S.I2  : inertia 2
% S.g   : gravity
%
% S.h : time-step

q = x(1:2);
v = x(3:4);

c1 = cos(q(1));
c2 = cos(q(2));
s2 = sin(q(2));
c12 = cos(q(1) + q(2));

% coriolis matrix
C = -S.m2*S.l1*S.lc2*s2*[v(2), v(1) + v(2);
                    -v(1), 0] + diag([.2;.2]);

% mass elements
m11 = S.m1*S.lc1^2 + S.m2*(S.l1^2 + S.lc2^2 + 2*S.l1*S.lc2*c2) + ...
      S.I1 + S.I2;

m12 = S.m2*(S.lc2^2 + S.l1*S.lc2*c2) + S.I2;

m22 = S.m2*S.lc2^2 + S.I2;

% mass matrix
M = [m11, m12;
     m12, m22];

% gravity vector
fg = [(S.m1*S.lc1 + S.m2*S.l1)*S.g*c1 + S.m2*S.lc2*S.g*c12;
      S.m2*S.lc2*S.g*c12];

% acceleration
a = inv(M)*(u - C*v - fg);
v = v + S.h*a;

x = [q + S.h*v;
     v];

% leave empty to use finite difference approximation
A= [];
B= [];
function xs = sys_traj(x0, us, S)
%N = size(us, 2);
xs(:,1) = x0;
xtemp = x0;
ind = 1;
for k=1:(S.steps(end)-1)
  xtemp = S.f(k*S.physxtimestep, xtemp, us(:,ind), S);
  if (k+1) == S.steps(ind+1)
        xs(:,ind+1) = xtemp;
        ind = ind+1;
  end
end
refreshdata(S.phandle,'caller');
pause(0.01);


function f = car_shooting()
% Example of trajectory optimization for a 
% car-like model using shooting and least-squares Gauss Newton method
%
% Marin Kobilarov marin(at)jhu.edu


% time horizon and segments
tf = 5;
S.N = 16;
S.h = tf/S.N;
S.m = 2;
S.n = 5;

% cost function parameters
S.Q = .0*diag([5, 5, 1, 1, 1]);
S.R = .1*diag([1, 5]);
S.Qf = diag([10, 10, 10, 1, 1]);

S.Qs = sqrt(S.Q);
S.Rs = sqrt(S.R);
S.Qfs = sqrt(S.Qf);

S.f = @arm_f;

% initial state
%x0 = [-5; -2; -1.2; 0; 0];
x0 = [3; -2; -2.2; 0; 0];

S.x0 = x0;

% initial control sequence
us = zeros(2,S.N);
xs = sys_traj(x0, us, S);

subplot(1,2,1)

plot(xs(1,:), xs(2,:), '-b')
hold on

m = size(us,1);
N = S.N;
lb=repmat([-1; -.4], N, 1); % can incorporate upper and lower bound
ub=repmat([1; .4], N, 1);

us = lsqnonlin(@(us)car_cost(us, S), us, lb, ub);

% update trajectory
xs = sys_traj(x0, us, S);

plot(xs(1,:), xs(2,:), '-b');

y = car_cost(us, S);
J=y'*y/2;

disp(['cost=' num2str(J)])

xlabel('x')
ylabel('y')

subplot(1,2,2)

plot(0:S.h:tf-S.h, us(1,:),0:S.h:tf-S.h, us(2,:));
xlabel('sec.')
legend('u_1','u_2')


function y = car_cost(us, S)
% this the car costs in least-squares form,
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
y = [y; S.Qfs*xs(:,N+1)];

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

function [x, A, B] = car_f(k, x, u, S)
% car dynamics and jacobians

h = S.h;
c = cos(x(3));
s = sin(x(3));
v = x(4);
w = x(5);

A = [1 0 -h*s*v h*c 0;
     0 1 h*c*v h*s 0;
     0 0 1 0 h;
     0 0 0 1 0;
     0 0 0 0 1];

B = [0 0;
     0 0;
     0 0;
     h 0;
     0 h];

x = [x(1) + h*c*v;
     x(2) + h*s*v;
     x(3) + h*w;
     v + h*u(1);
     w + h*u(2)];


function xs = sys_traj(x0, us, S)

N = size(us, 2);
xs(:,1) = x0;

for k=1:N,
  xs(:, k+1) = S.f(k, xs(:,k), us(:,k), S);
end

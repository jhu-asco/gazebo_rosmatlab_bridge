function f = arm_sddp_analytic()
% Trajectory Optimization for 2DOF pendulum using SDDP Optimization (Analytic Model)

% model parameters
S.m1 = 1;
S.m2 = 1;
S.l1 = .5;
S.l2 = .5;
S.lc1 = .25;
S.lc2 = .25;
S.I1 = S.m1*S.l1/12;
S.I2 = S.m2*S.l2/12;
S.g = 9.8;

% time horizon and number of segments
tf = 2;
S.N = 128;
S.h = tf/S.N;
ts = 0:S.h:tf;

% Cost function parameters
S.Q = diag([1, 1, .1, .1]);
S.R = diag([.05, .05]);
S.Pf = diag([5, 5, 1, 1]);

S.f = @sys_traj;
S.L = @arm_L;
S.mu = 0;

% initial state 
x0 = [0; 0; 0; 0];
%Setup Optimization Problem:
Ds = zeros(2,4,S.N);
% final desired state
S.xf = [pi/2; 0; 0; 0];

% initial controls 
us = zeros(2, S.N);

%us = [repmat([.1;0.1], 1, S.N/2), repmat(-[.1;0.1], 1, S.N/2)]/5;
%us = [repmat([.1;0], 1, N/2), repmat(-[.1;0], 1, N/2)]/5;

%%% Setup the Visualization of the states and controls [May Not be obvious how to set this up but very useful for debugging]
figure(1),clf,
S.phandle(1) = plot(0,0);
hold on, S.phandle(2) = plot(0,0,'r');
xlabel('time');
ylabel('Joint Angles');
set(S.phandle(1),'XData',ts);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',ts);
set(S.phandle(2),'YDataSource','xs(2,:)');
figure(2),clf,
S.phandle(3) = plot(0,0);
hold on, S.phandle(4) = plot(0,0,'r');
xlabel('time');
ylabel('Controls');
set(S.phandle(3),'XData',(ts(1:end-1)));
set(S.phandle(3),'YDataSource','us(1,:)');
set(S.phandle(4),'XData',(ts(1:end-1)));
set(S.phandle(4),'YDataSource','us(2,:)');
hold on


%First Iteration no initial gains and no initial trajectory. Only initial controls given.
[V, Vn, dV, xs, us, Ds] = sddp(x0, us, [], Ds, S);
disp('V: ');
disp(V);
disp('Vn: ');
disp(Vn);
disp('dV');
disp(dV);

%For future iterations pass in xs:
for it = 1:25
  [V, Vn, dV, xs, us, Ds] = sddp(x0, us, xs, Ds, S);
  disp('V: ');
  disp(V);
  disp('Vn: ');
  disp(Vn);
  disp('dV');
  disp(dV);
  disp('xn:');
  disp(xs(:,end)');
  pause;
end
% 
% xs = ddp_traj(x0, us, S);
% 
% Jmin = ddp_cost(xs, us,  S)
% pause;
% subplot(1,2,1)
% 
% plot(xs(1,:), xs(2,:), '-b')
% hold on
% plot(S.xf(1),S.xf(2),'*g')
% axis equal
% 
% S.a = 1;
% 
% for i=1:20
%   [dus, V, Vn, dV, a] = ddp(x0, us, S);
% 
%   % update controls
%   us = us + dus;
% 
%   S.a = a;   %reuse old step-size for efficiency
%   
%   % update trajectory using new controls
%   xs = ddp_traj(x0, us, S);
% 
%   plot(xs(1,:), xs(2,:), '-b');
%   
% %  keyboard
% end
% plot(xs(1,:), xs(2,:), '-g','LineWidth',3);
% 
% J = ddp_cost(xs, us, S)
% 
% xlabel('q_1')
% ylabel('q_2')
% 
% subplot(1,2,2)
% 
% plot(0:S.h:tf-S.h, us(1,:),0:S.h:tf-S.h, us(2,:));
% xlabel('sec.')
% ylabel('Nm')
% legend('u_1','u_2')
% 

function [L, Lx, Lxx, Lu, Luu] = arm_L(k, x, u, S)
% arm cost function

dx = x - S.xf;

if (k==S.N+1)
  L = dx'*S.Pf*dx/2;
  Lx = S.Pf*dx;
  Lxx = S.Pf;
  Lu = [];
  Luu = [];
else
  L = S.h/2*(dx'*S.Q*dx + u'*S.R*u);
  Lx = S.h*S.Q*dx;
  Lxx = S.h*S.Q;
  Lu = S.h*S.R*u;
  Luu = S.h*S.R;
end


function [xs, us] = sys_traj(x0, us0, xds, cs, Ds, S)
%This function evaluates the system trajectory for a given nominal trajectory and control gains and optional perturbations
% If no nominal trajectory is provided, the perturbations are propagated in openloop.
% xds: nx(N+1) matrix of desired states
% cs: mxN matrix of control intercepts
% Ds: mxnxN matrix of control gains
% us0: mxN matrix of control perturbations/previous controls; If no desired trajectory, it is the openloop controls
% x0: nx1 vector of initial state

%Check args:
if isempty(xds)
  %Openloop:
  N = size(us0,2);
  xs = zeros(length(x0), N+1);
  xs(:,1) = x0;
  %Run the loop
  for i = 1:N
    %Run the simulation for given step size and given controls.
    xs(:,i+1) = arm_f(i,xs(:,i),us0(:,i), S);
  end
  return;
end

%%%% xds is not empty so using feedback loop:

%Convenience variables:
N = size(cs,2);
m = size(cs,1);
n = size(xds,1);

%Check if perturbations are provided
if isempty(us0)
  us0 = zeros(m,N);
end

%Using Feedback and running the control loop:
xs = zeros(n, N+1);
us = zeros(m,N);
xs(:,1) = x0;
for i = 1:N
    % dx:
    dx = xs(:,i) - xds(:,i);%This can be some manifold difference #TODO
    %Find feedback control and perturb it by dus
    us(:,i) = cs(:,i) + Ds(:,:,i)*dx + us0(:,i);
    %Run the simulation for given step size and given controls.
    xs(:,i+1) = arm_f(i,xs(:,i),us(:,i), S);
end

refreshdata(S.phandle,'caller');
pause(0.01);

function [x, A, B] = arm_f(~, x, u, S)
% arm discrete dynamics
% set jacobians A, B to [] if unavailable
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
a = M\(u - C*v - fg);
v = v + S.h*a;

x = [q + S.h*v;
     v];

% leave empty to use finite difference approximation
A= [];
B= [];

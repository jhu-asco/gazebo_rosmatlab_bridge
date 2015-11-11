function f = car_sddp_analytic()
% Trajectory Optimization for rccar using SDDP Optimization (Analytic Model)
% time horizon and number of segments
tf = 20;
S.N = 32;
S.h = tf/S.N;
ts = 0:S.h:tf;

% Cost function parameters
S.Q = .0*diag([5, 5, 1, 1, 1]);
S.R = diag([1, 5]);
S.Qf = diag([5, 5, 1, 1, 1]);

S.f = @sys_traj;
S.L = @car_L;
S.mu = 1;

% initial state 
x0 = [-5; -2; -1.2; 0; 0];
%Setup Optimization Problem:
Ds = zeros(2,5,S.N);
% final desired state
S.xf = [0; 0; 0; 0; 0];

% initial controls 
us = [repmat([.1;0.1], 1, S.N/2), repmat(-[.1;0.1], 1, S.N/2)]/5;
%us = zeros(2, S.N);
%us = [repmat([.1;0.1], 1, S.N/2), repmat(-[.1;0.1], 1, S.N/2)]/5;
%us = [repmat([.1;0], 1, N/2), repmat(-[.1;0], 1, N/2)]/5;

%%% Setup the Visualization of the states and controls [May Not be obvious how to set this up but very useful for debugging]
figure(1),clf,
subplot(1,3,1), xlabel('Time'), ylabel('x');
hold on, S.phandle(1) = plot(0,0);
subplot(1,3,2), xlabel('Time'), ylabel('y');
hold on, S.phandle(2) = plot(0,0,'r');
subplot(1,3,3), xlabel('Time'), ylabel('theta');
hold on, S.phandle(3) = plot(0,0,'g');
set(S.phandle(1),'XData',ts);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',ts);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',ts);
set(S.phandle(3),'YDataSource','xs(3,:)');%States
figure(2),clf,
subplot(2,1,1), xlabel('Time'), ylabel('drive torque');
hold on, S.phandle(4) = plot(0,0);
subplot(2,1,2), xlabel('Time'), ylabel('steer torque');
hold on, S.phandle(5) = plot(0,0,'r');
set(S.phandle(4),'XData',(ts(1:end-1)));
set(S.phandle(4),'YDataSource','us(1,:)');
set(S.phandle(5),'XData',(ts(1:end-1)));
set(S.phandle(5),'YDataSource','us(2,:)');
figure(3), clf;
S.phandle(6) = plot(0,0);
xlabel('x'), ylabel('y');
set(S.phandle(6),'XDataSource','xs(1,:)');
set(S.phandle(6),'YDataSource','xs(2,:)');

%First Iteration no initial gains and no initial trajectory. Only initial controls given.
[V, Vn, dV, xs, us, Ds] = sddp(x0, us, [], Ds, S);
disp('V: ');
disp(V);
disp('Vn: ');
disp(Vn);
disp('dV');
disp(dV);

%For future iterations pass in xs:
for it = 1:10
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

function [L, Lx, Lxx, Lu, Luu] = car_L(k, x, u, S)
% car cost function

dx = x - S.xf;

if (k==S.N+1)
  L = dx'*S.Qf*dx/2;
  Lx = S.Qf*dx;
  Lxx = S.Qf;
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
    xs(:,i+1) = car_f(i,xs(:,i),us0(:,i), S);
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
    xs(:,i+1) = car_f(i,xs(:,i),us(:,i), S);
end

refreshdata(S.phandle,'caller');
pause(0.01);

function [x, A, B] = car_f(~, x, u, S)
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

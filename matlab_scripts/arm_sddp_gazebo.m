function arm_sddp_gazebo()
%ARM_SDDP Example of trajectory optimization for 2 link arm using SDDP
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

% time horizon and segments
tf = 2; %Final Time
S.N = 200; %Number of control segments
S.h = tf/S.N; %Step size
S.m = 2; %Control size
S.n = 4; % State size
S.ts = 0:S.h:tf;%Converts the time into physics time steps

% cost function parameters
S.Q = diag([0.1, 0.1, 0.5, 0.5]);
S.R = 0.01*diag([1, 0.1]);
S.Qf = diag([20, 20, 4, 4]);

%Step size parameters
S.a = 1; %Default step size for DDP

%Physics Engine Parameters
S.sim = GazeboMatlabSimulator;%Creates a Matlab Bridge using a helper class
S.sim.Configure(0.001,100);%Configure the physics engine to have a time step of 1 milli second and real time rate is 100
S.sim.ActuatedJoints = [1 2]; %These are the joints that are being actuated

% initial state
x0 = [-1.54; 0.0; 0; 0;];%State is Joint angles(1,2) and Joint Velocities(1,2)
%x0 = [-1.54; 0; 0; 0;];%State is Joint angles(1,2) and Joint Velocities(1,2)
%S.xf = [1; 0; 0; 0];%Final desired state
 S.xf = [1.54; 0.0; 0; 0];%Final desired state

%Setup Optimization Problem:
Ds = zeros(S.m,S.n,S.N);
%us = zeros(S.m, S.N);%For now zero controls to start
u0 = [0; 0];
us = repmat(u0, 1,S.N);

%Trajectory function:
S.f = @sys_traj;%Trajectory generation function with/out feedback
S.L = @sys_cost;%Cost function for the system

%%% Setup the Visualization of the states and controls [May Not be obvious how to set this up but very useful for debugging]
figure(1),clf,
S.phandle(1) = plot(0,0);
hold on, S.phandle(2) = plot(0,0,'r');
xlabel('time');
ylabel('Joint Angles');
set(S.phandle(1),'XData',S.ts);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.ts);
set(S.phandle(2),'YDataSource','xs(2,:)');
figure(2),clf,
S.phandle(3) = plot(0,0);
hold on, S.phandle(4) = plot(0,0,'r');
xlabel('time');
ylabel('Controls');
set(S.phandle(3),'XData',(S.ts(1:end-1)));
set(S.phandle(3),'YDataSource','us(1,:)');
set(S.phandle(4),'XData',(S.ts(1:end-1)));
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
  while (1)
      user_input = input('Press (a) to animate; (q) to quit; Any other key to iterate\n','s');
      if user_input == 'a'
          S.sim.Configure(0.001,1);
          S.f(x0, us, [],[],[],S);%Openloop integration of controls
          S.sim.Configure(0.001,100);
      elseif user_input == 'q'
          return;
      else
          break;
      end
  end
%   refreshdata(S.phandle,'caller');
%   pause;
end


function [L, Lx, Lxx, Lu, Luu] = sys_cost(k, x, u, S)
%system cost function for each node in the trajectory
% Trajectory cost is the sum of the nodes

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
  %Reset System
  S.sim.Reset;
  pause(0.01);
  S.sim.SetModelState('double_pendulum_with_base',[],[x0(1:2)';x0(3:4)']);
  %Run the loop
  for i = 1:N
    %Run the simulation for given step size and given controls.
    [~, JointData] = S.sim.Step((S.ts(i+1) - S.ts(i)), us0(:,i));
    %Set the state based on joint data
    xs([1,3],i+1) = JointData(:,1);
    xs([2,4],i+1) = JointData(:,2);
    %Map the joint angles to -pi to pi range
    A = rem(xs(1:2, i+1),2*pi);
    A(A>pi) = A(A>pi)-2*pi;
    A(A<-pi) = A(A<-pi)+2*pi;
    xs(1:2,i+1)= A;
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

%Reset System
S.sim.Reset;
pause(0.01);
S.sim.SetModelState('double_pendulum_with_base',[],[x0(1:2)';x0(3:4)']);

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
    [~, JointData] = S.sim.Step((S.ts(i+1) - S.ts(i)), us(:,i));
    %Set the state based on joint data
    xs([1,3],i+1) = JointData(:,1);
    xs([2,4],i+1) = JointData(:,2);
    %Map the joint angles to -pi to pi range
    A = rem(xs(1:2, i+1),2*pi);
    A(A>pi) = A(A>pi)-2*pi;
    A(A<-pi) = A(A<-pi)+2*pi;
    xs(1:2,i+1)= A;
end

refreshdata(S.phandle,'caller');
pause(0.01);

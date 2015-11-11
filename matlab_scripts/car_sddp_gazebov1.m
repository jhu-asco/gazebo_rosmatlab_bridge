function f = car_sddp_gazebov1()
%CAR_SDDP Example of trajectory optimization for car model in gazebo
%
% This is the first version of rccar optimization with 6 states for the car and two control inputs
% 
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu
% time horizon and number of segments

tf = 4;
S.N = 20;
S.h = tf/S.N;
S.ts = 0:S.h:tf;

%Physics Engine params
S.sim = GazeboMatlabSimulator;
S.sim.Configure(0.001,20);
S.sim.ActuatedJoints = 1:4;%We are actuating all the joints

%Physics Parameters
S.l = 0.3; %Length of the car
S.wheelradius = 0.05;%Wheel radius

% Cost function parameters
S.Q = .0*diag([5, 5, 1, 1, 1, 1]);
S.R = 0.01*diag([1, 5]);
S.Qf = 10*diag([5, 5, 2, 1, 1, 1]);

S.f = @sys_traj;
S.L = @sys_cost;
S.mu = 1;

% initial state 
x0 = [0; 0; 0; 0; 0; 0];
S.xf = [1; 1; pi/4; 0; 0; 0];%Posn(x,y),Angle, Body Velocity, steering angle and velocity

%Setup Optimization Problem:
Ds = zeros(2,6,S.N);
% us = zeros(2,S.N);% initial control sequence
% us(1,:) = 0.3;
% us(2,:) = 0.02;
us = [repmat([.3;0.8], 1, S.N/2), repmat(-[-.3;0.7], 1, S.N/2)];


%Trajectory function:
S.f = @sys_traj;%Trajectory generation function with/out feedback
S.L = @sys_cost;%Cost function for the system

%%% Setup the Visualization of the states and controls [May Not be obvious how to set this up but very useful for debugging]
figure(1),clf,
subplot(1,3,1), xlabel('Time'), ylabel('x');
hold on, S.phandle(1) = plot(0,0);
subplot(1,3,2), xlabel('Time'), ylabel('y');
hold on, S.phandle(2) = plot(0,0,'r');
subplot(1,3,3), xlabel('Time'), ylabel('theta');
hold on, S.phandle(3) = plot(0,0,'g');
set(S.phandle(1),'XData',S.ts);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.ts);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',S.ts);
set(S.phandle(3),'YDataSource','xs(3,:)');%States
figure(2),clf,
subplot(2,1,1), xlabel('Time'), ylabel('drive torque');
hold on, S.phandle(4) = plot(0,0);
subplot(2,1,2), xlabel('Time'), ylabel('steer torque');
hold on, S.phandle(5) = plot(0,0,'r');
set(S.phandle(4),'XData',(S.ts(1:end-1)));
set(S.phandle(4),'YDataSource','us(1,:)');
set(S.phandle(5),'XData',(S.ts(1:end-1)));
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
end

function [L, Lx, Lxx, Lu, Luu] = sys_cost(k, x, u, S)
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
  
  reset_car(x0, S);%Reset the car
  
  %Run the loop
  for i = 1:N
    %Run the simulation for given step size and given controls.
    xs(:,i+1) = car_f(i,us0(:,i), S);
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

reset_car(x0, S);%Reset the car

for i = 1:N
    % dx:
    dx = xs(:,i) - xds(:,i);%This can be some manifold difference #TODO
    %Find feedback control and perturb it by dus
    us(:,i) = cs(:,i) + Ds(:,:,i)*dx + us0(:,i);
    %Run the simulation for given step size and given controls.
    xs(:,i+1) = car_f(i,us(:,i),S);
end

refreshdata(S.phandle,'caller');
pause(0.01);

function x = car_f(k,u,S)
%Single step in Feedback loop:
us = [u;u];%Copy us two times for both left and right hand side joints
[LinkData, JointData] = S.sim.Step((S.ts(k+1) - S.ts(k)), us);%Run Gazebo Dynamics
x = zeros(6,1);
%Write new state x based on link and joint data:
LinkState = LinkData{1};
rpy = quat2rpy(LinkState.orientation);
x(3) = rpy(3); %Yaw of body
x(1:2) = LinkState.position(1:2); %Posn of Body
x(4) = LinkState.linearvelocity(1)*cos(x(3)) + LinkState.linearvelocity(2)*sin(x(3));%Body velocity of car
x(5) = (JointData(1,2) + JointData(1,4))/2;
x(6) = (JointData(2,2) + JointData(2,4))/2;

function reset_car(x0, S)
S.sim.Reset;
pause(0.01);
%Set car to initial posn:
modelstate = MatlabRigidBodyState;%Helper class to store rigidbody state
modelstate.position = [ x0(1); x0(2); S.wheelradius];%Initial position
modelstate.orientation = rpy2quat([x0(3), 0, 0]);%Initial orientation
modelstate.linearvelocity(1) = x0(4);%Initial velocity
joint_states = [0 x0(5) 0 x0(5);0 x0(6) 0 x0(6)];%1 and 3 are driving wheels; 2 and 4 steering
S.sim.SetModelState('Unicycle',modelstate, joint_states);%Set the model state. Joints are not modified.

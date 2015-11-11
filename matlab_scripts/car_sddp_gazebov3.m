function f = car_sddp_gazebov3()
%CAR_SDDP Example of trajectory optimization for car model in gazebo
% 
%This uses velocity servo on both steering and driving. The state of the
%car is x,y,theta,steering angle. controls = desired steering vel, body vel
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu
% time horizon and number of segments

%tf = 4;
tf = 4;
% S.N = 100;
% S.N = 50;
S.N = 20;
S.h = tf/S.N;
S.ts = 0:S.h:tf;

%Physics Engine params
S.sim = GazeboMatlabSimulator;
S.sim.Configure(0.001,20);
S.sim.ActuatedJoints = 1:4;%We are actuating all the joints
S.sim.AttachServo([1,3],[2.0;0.0;0.0],[2;-2;5;-5],1);%Joints for which servos should be attached, gains(PID), limits(integral limits and command limits), control type: position(0)/velocity control(1)
S.sim.AttachServo([2,4],[50.0;10.0;0.0],[100;-100;500;-500],1);%Attach a servo to control steering angle velocities

%Physics Parameters
S.l = 0.3; %Length of the car
S.wheelradius = 0.05;%Wheel radius

% Cost function parameters
S.Q = 0.1*diag([5, 5, 1, 0.1]);
S.R = diag([1, 5]);
S.Qf = 30*diag([5, 5, 1, 0]);

S.f = @sys_traj;
S.L = @sys_cost;
S.mu = 1;

% initial state 
x0 = [0; 0; 0; 0];
%  S.xf = [4; 1.5; 0; 0];%Posn(x,y),Angle, Steering angle
S.xf = [1.5; 0.2; 0; 0];%Posn(x,y),Angle, Steering angle
% S.xf = [2; 1; 0; 0];%Posn(x,y),Angle, Steering angle

%Setup Optimization Problem:
Ds = zeros(2,4,S.N);
% us = zeros(2,S.N);% initial control sequence
% us(1,:) = 0.3;
% us(2,:) = 0.02;
us = [repmat([.2;0.2], 1, S.N/2), repmat([.2;-0.4], 1, S.N/2)];
% us = [repmat([0.5;0], 1, S.N/2), repmat([0.5;0], 1, S.N/2)];

%Trajectory function:
S.f = @sys_traj;%Trajectory generation function with/out feedback
S.L = @sys_cost;%Cost function for the system

%%% Setup the Visualization of the states and controls [May Not be obvious how to set this up but very useful for debugging]
figure(1),clf,
subplot(2,2,1), xlabel('Time'), ylabel('x');
hold on, S.phandle(1) = plot(0,0);
subplot(2,2,2), xlabel('Time'), ylabel('y');
hold on, S.phandle(2) = plot(0,0,'r');
subplot(2,2,3), xlabel('Time'), ylabel('theta');
hold on, S.phandle(3) = plot(0,0,'g');
subplot(2,2,4), xlabel('Time'), ylabel('Steer');
hold on, S.phandle(4) = plot(0,0,'g');
set(S.phandle(1),'XData',S.ts);
set(S.phandle(1),'YDataSource','xs(1,:)');
set(S.phandle(2),'XData',S.ts);
set(S.phandle(2),'YDataSource','xs(2,:)');
set(S.phandle(3),'XData',S.ts);
set(S.phandle(3),'YDataSource','xs(3,:)');
set(S.phandle(4),'XData',S.ts);
set(S.phandle(4),'YDataSource','xs(4,:)');%States
figure(2),clf,
subplot(2,1,1), xlabel('Time'), ylabel('drive torque');
hold on, S.phandle(5) = plot(0,0);
subplot(2,1,2), xlabel('Time'), ylabel('steer torque');
hold on, S.phandle(6) = plot(0,0,'r');
set(S.phandle(5),'XData',(S.ts(1:end-1)));
set(S.phandle(5),'YDataSource','us(1,:)');
set(S.phandle(6),'XData',(S.ts(1:end-1)));
set(S.phandle(6),'YDataSource','us(2,:)');
figure(3), clf;
S.phandle(7) = plot(0,0);
xlabel('x'), ylabel('y');
set(S.phandle(7),'XDataSource','xs(1,:)');
set(S.phandle(7),'YDataSource','xs(2,:)');

%First Iteration no initial gains and no initial trajectory. Only initial controls given.
[V, Vn, dV, xs, us, Ds] = sddp(x0, us, [], Ds, S);
disp('V: ');
disp(V);
disp('Vn: ');
disp(Vn);
disp('dV');
disp(dV);
disp('xn:');
disp(xs(:,end)');

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
%Multiply the input by wheel radius to get body velocity:
us([1,3]) = us([1,3])*(1/S.wheelradius);
[LinkData, JointData] = S.sim.Step((S.ts(k+1) - S.ts(k)), us);%Run Gazebo Dynamics
x = zeros(4,1);
%Write new state x based on link and joint data:
LinkState = LinkData{1};
rpy = quat2rpy(LinkState.orientation);
x(3) = rpy(3); %Yaw of body
x(1:2) = LinkState.position(1:2); %Posn of Body
x(4) = (JointData(1,2) + JointData(1,4))/2;%Steering angle
%x(4) = LinkState.linearvelocity(1)*cos(x(3)) + LinkState.linearvelocity(2)*sin(x(3));%Body velocity of car
%x(6) = (JointData(2,2) + JointData(2,4))/2;

function reset_car(x0, S)
S.sim.Reset;
pause(0.01);
%Set car to initial posn:
modelstate = MatlabRigidBodyState;%Helper class to store rigidbody state
modelstate.position = [ x0(1); x0(2); S.wheelradius];%Initial position
modelstate.orientation = rpy2quat([x0(3), 0, 0]);%Initial orientation
modelstate.linearvelocity(1) = 0;%Initial velocity same as required from control
joint_states = zeros(2,4);
joint_states(1,[2,4]) = [x0(4), x0(4)];%Same steering angle as required
S.sim.SetModelState('Unicycle',modelstate, joint_states);%Set the model state. Joints are not modified.

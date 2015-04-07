function f = arm_test()
% EN530.678 HW#3 supplementary 
% simulation of a two-link manipulator 
%
% M. Kobilarov
%
% Modified by G.Gowtham.

% model parameters
S.m1 = 1;
S.m2 = 1;
S.l1 = 1.0;
S.l2 = 1.0;
S.lc1 = .5;
S.lc2 = .5;
S.I1 = 1.0;
S.I2 = 1.0;
S.g = 9.81;

% initial state with high-velocity
x0 = [0; 0; 0; 0];

% desired state
S.xd = [1; -1; 0; 0];

% desired accelration -- only applicable for trajectory tracking
S.ad = [0; 0];

T = 3; % simulation time
frequency = 50;%Hz


S.kp = 20; S.kd = 10;%Gain parameters

h = GazeboMatlabSimulator;
h.ActuatedJoints = [1 2]; %These are set based on Available Joint names. Can be viewed using S.sim.AvailableNames{2}
%Reset the Physics World
h.Reset;

%Set Initial State:
h.SetModelState('double_pendulum_with_base',[],[x0(1:2)';x0(3:4)']);

%Feedback Loop:
x = x0;
N = frequency*T;%Number of steps to run
xs = zeros(4,N);%All the states
for i = 1:N
    stepsize = 1/frequency;
    u = arm_ctrl((i-1)*stepsize, x, S);%Compute Control
    [~, JointData] = h.Step(stepsize, u);
    x([1,3]) = JointData(:,1);
    x([2,4]) = JointData(:,2);
    %Map the joint angles to -pi to pi range
    A = rem(x(1:2),2*pi);
    A(A>pi) = A(A>pi)-2*pi;
    A(A<-pi) = A(A<-pi)+2*pi;
    x(1:2)= A;
    xs(:,i) = x;
    disp('u:');
    disp(u);
    disp('xs:');
    disp(x');
end
figure; subplot(2,1,1), plot((1:N)*(1/frequency),xs(1,:),'b','DisplayName','Joint1Angle');
subplot(2,1,1), hold on, plot((1:N)*(1/frequency),xs(2,:),'r','DisplayName','Joint2Angle');

subplot(2,1,2), plot((1:N)*(1/frequency),xs(3,:),'b','DisplayName','Joint1velocity');
subplot(2,1,2), hold on, plot((1:N)*(1/frequency),xs(4,:),'r','DisplayName','Joint2velocity');


function u = arm_ctrl(t, x, S)
% standard computed torque law

% current
q = x(1:2);
v = x(3:4);

% desired
qd = S.xd(1:2);
vd = S.xd(3:4);

[M, C, N] = arm_dyn(t, x, S);
u = M*(S.ad - S.kp*(q - qd) - S.kd*(v - vd)) + C*v + N;

% alternatively without coriolis/centripetal
% u = M*(S.ad - S.kp*(q - qd) - S.kd*(v - vd)) + N;




function [M, C, N] = arm_dyn(t, x, S)
% compute the dynamical terms of the manipulator
% M - mass matrix
% C - Corilois matrix
% N - gravity/damping forces vector

q = x(1:2);
q(2) = q(2) - pi/6;% Offset due to model joint in Gazebo being offset
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

% gravity, damping, etc...
N = [(S.m1*S.lc1 + S.m2*S.l1)*S.g*c1 + S.m2*S.lc2*S.g*c12;
      S.m2*S.lc2*S.g*c12];



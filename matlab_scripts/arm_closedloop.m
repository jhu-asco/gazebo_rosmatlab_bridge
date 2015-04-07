function f = arm_closedloop()
%ARM_CLOSEDLOOP Example for closed loop PID control of Double Pendulum 2 link arm
% The dynamics are obtained from Gazebo MATLAB bridge
%
% Gowtham garimella ggarime1(at)jhu.edu

h = GazeboMatlabSimulator;%Creates a Matlab Bridge using a helper class
h.Configure(0.001,1);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1

%%% Problem Setting %%%%
tf = 5;%Time to run the Controller for
frequency = 100;%100 Hz freq of controller
N = frequency*tf;%Number of steps to run
x0 = [0, 0, 0, 0];%State: Joint angles(1,2) Joint Velocities(1,2) %Initial velocity has to be zero
S.xf = [1,1,0,0];%Final State

S.int_offset = zeros(1,2);%Used by controller to add intergration offset (Optional Params)
h.ActuatedJoints = [1 2]; %These are set based on Available Joint names. Can be viewed using S.sim.AvailableNames{2}

%Reset the Physics World
h.Reset;

%Set the initial state. This sets the joint angles of a given model without
%modifying the model state. Model state can also be modified by giving the
%second argument as model input.
h.SetModelState('double_pendulum_with_base',[],[x0(1:2);x0(3:4)]);

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
for i = 1:N
    [u,S] = ArmPID(x,S);%Run the control law
    %Run the simulation for given step size and given controls.
    [~, JointData] = h.Step((1/frequency), u);
    %Set the state based on joint data
    x([1,3]) = JointData(:,1);
    x([2,4]) = JointData(:,2);
    %Map the joint angles to -pi to pi range
    A = rem(x(1:2),2*pi);
    A(A>pi) = A(A>pi)-2*pi;
    A(A<-pi) = A(A<-pi)+2*pi;
    x(1:2)= A;
    disp('xs:');
    disp(x');
end


function [u,S]  = ArmPID(x,S)%Linkdata and time are not used right now
%STEERPID PID controller for Steering
%Error is used for integrating (ki term)
kp = 50;
kd = 50;
ki = 0.05;
error = S.xf - x;
S.int_offset = S.int_offset + ki*error(1:2);
u = kp*error(1:2) + kd*error(3:4) + S.int_offset;

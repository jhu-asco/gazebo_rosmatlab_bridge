function f = arm_closedloop()
% Example for closed loop PID control of Double Pendulum
% The dynamics are obtained from Gazebo MATLAB bridge
%
% Gowtham garimella ggarime1(at)jhu.edu

S.sim = Gazebo_MatlabSimulator;%Creates a Matlab Bridge using a helper class
S.sim.Configure(0.001,1);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1

%%% Problem Setting %%%%
tf = 10;%Time to run the Controller for
x0 = [-1, 0, 0, 0];%State: Joint angles(1,2) Joint Velocities(1,2) %Initial velocity has to be zero
S.xf = [1,0,0,0];%Final State
S.N = 100*tf;%Feedback Freq = S.N/tf = 100 Hz
S.nofsteps = round(tf/(S.N*S.sim.physxtimestep));%Internal computation of nof physics steps for one feedback step 
%For example feedback running at 100 Hz (time step 0.01 sec) has 10 physics steps in it, since physics time step is 1 millisecond (configured above)

S.int_offset = zeros(1,2);%Used by controller to add intergration offset (Optional Params)
jointids = [1 2]; %These are set based on Available Joint names. Can be viewed using S.sim.AvailableNames{2}

%Reset the Physics World
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);% Wait for it take effect may not be necessary

mex_mmap('setmodelstate',S.sim.Mex_data,'double_pendulum_with_base',[],...
    uint32(jointids)-1,[x0(1:2);x0(3:4)]); %Set the Joint Angles without worrying about the pendulum position

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
for i = 1:S.N
  [u,S] = ArmPID(x,S);
  [~, JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, u, ...
                                                    [], [], uint32([0,S.nofsteps]));
if i == 1 %Simple Verification to ensure the actual starting position is same as the specified initial position (Can be removed in future #DEBUG )
    checkx = [JointData(1,1:2) JointData(2,1:2)];
    if norm(x0 - checkx) > 1e-3
        disp('Model Did not start from initial condition');
        disp(checkx);
    end
end
x([1,3]) = JointData(:,3);
x([2,4]) = JointData(:,4);
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

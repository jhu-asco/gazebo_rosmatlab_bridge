function f = arm_closedloop()
S.sim = Gazebo_MatlabSimulator;
S.sim.Configure(0.001,1);
S.int_offset = zeros(1,2);%Used by controller to add intergration offset
tf = 5;%Time to run the Controller for
x0 = zeros(1,4);
S.xf = [1,1,0,0];%Final State
S.N = 500;%Freq = S.N/S.t = 100 Hz
S.nofsteps = round(tf/(S.N*S.sim.physxtimestep));
jointids = [1 2];
%Reset the state
mex_mmap('stringreq',S.sim.Mex_data,'worldreset');
x = x0;%Temporary state
%Start Feedback loop:
for i = 1:S.N
  u = ArmPID(x,S);
  [~, JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, u, ...
                                                    [], [], uint32([0,S.nofsteps]));
x([1,3]) = JointData([1,4],3);
x([2,4]) = JointData([1,4],4);
A = rem(x(1:2),2*pi);
A(A>pi) = A(A>pi)-2*pi;
A(A<-pi) = A(A<-pi)+2*pi;
x(1:2)= A;%Mapping to -pi to pi
disp('xs:');
disp(x');
end


function u  = ArmPID(x,S)%Linkdata and time are not used right now
%STEERPID PID controller for Steering
%Error is used for integrating (ki term)
kp = 50;
kd = 50;
ki = 0.04;
error = S.xf - x;
S.int_offset = S.int_offset + ki*error(1:2);
u = kp*error(1:2) + kd*error(3:4) + S.int_offset;
%refreshdata;
%figure(1), subplot(2,2,1), hold on, plot(jointdata(1,1).k
%disp(h.time);
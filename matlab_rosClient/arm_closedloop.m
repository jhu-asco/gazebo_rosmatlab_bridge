function f = arm_closedloop()
S.sim = Gazebo_MatlabSimulator;
S.sim.Configure(0.001,1);
S.int_offset = zeros(1,2);%Used by controller to add intergration offset
tf = 10;%Time to run the Controller for
x0 = [-1, 0, 0, 0];%Initial velocity has to be zero
S.xf = [1,0,0,0];%Final State
S.N = 100*tf;%Freq = S.N/S.t = 100 Hz
S.nofsteps = round(tf/(S.N*S.sim.physxtimestep));
jointids = [1 2];
%Reset the state
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);
modelposeandtwist = [zeros(3,1);1;zeros(9,1)];
mex_mmap('setmodelstate',S.sim.Mex_data,'double_pendulum_with_base',modelposeandtwist,...
    uint32(jointids)-1,[x0(1:2);x0(3:4)]);
x = x0;%Temporary state
%Start Feedback loop:
for i = 1:S.N
  [u,S] = ArmPID(x,S);
  [~, JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, u, ...
                                                    [], [], uint32([0,S.nofsteps]));
if i == 1
    checkx = [JointData(1,1:2) JointData(2,1:2)];
    if norm(x0 - checkx) > 1e-3
        disp('Model Did not start from initial condition');
        disp(checkx);
    end
end
x([1,3]) = JointData(:,3);
x([2,4]) = JointData(:,4);
A = rem(x(1:2),2*pi);
A(A>pi) = A(A>pi)-2*pi;
A(A<-pi) = A(A<-pi)+2*pi;
x(1:2)= A;%Mapping to -pi to pi
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
%refreshdata;
%figure(1), subplot(2,2,1), hold on, plot(jointdata(1,1).k
%disp(h.time);
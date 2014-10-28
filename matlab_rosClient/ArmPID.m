function Optionalparams  = ArmPID(h,Optionalparams)%Linkdata and time are not used right now
%STEERPID PID controller for Steering
%Optionalparams should be [desiredjointangles error]
%Error is used for integrating (ki term)
kp = 20;
kd = 20;
ki = 0.01;
error = [Optionalparams(1)-h.JointData(1,1) Optionalparams(2)-h.JointData(1,2)];
Optionalparams(3:4) = Optionalparams(3:4) + ki*error;
torque = kp*(error) - kd*h.JointData(4,1:2) + Optionalparams(3:4);
mex_mmap('setjointeffort',h.Mex_data,[1,2], torque);%Use Joint indices directly
%refreshdata;
%figure(1), subplot(2,2,1), hold on, plot(jointdata(1,1).k
%disp(h.time);
end


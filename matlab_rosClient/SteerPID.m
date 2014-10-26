function Optionalparams  = SteerPID(h,Optionalparams)%Linkdata and time are not used right now
%STEERPID PID controller for Steering
%kp = 50;
%kd = 60;
kp = 10;
kd = 10;
%disp(jointdata);
%disp(t);
ki = 0.005;
error = [Optionalparams(1)-h.JointData(1,1) Optionalparams(1)-h.JointData(1,2)];
Optionalparams(2:3) = Optionalparams(2:3) + ki*error;
Steer_torque = kp*(error) - kd*h.JointData(4,1:2) + Optionalparams(2:3);
mex_mmap('seteffort',h.Mex_data,[2,3], Steer_torque);
%refreshdata;
%figure(1), subplot(2,2,1), hold on, plot(jointdata(1,1).k
%disp(h.time);
end


function Optionalparams = DrivePID(h,Optionalparams)
%DRIVEPID PID controller for Driving the car
kp = 0.5;
%[~,jointdata] = mex_mmap('readlinkjoint',A,{[],[2,3]});%Inds can be changed into names later
%disp(jointdata);
%disp(t);
ki = 0.01;
%disp(jointdata(4,1));
%disp(jointdata(4,2));
error = [Optionalparams(1)-h.JointData(4,3) Optionalparams(1)-h.JointData(4,4)];
Optionalparams(2:3) = Optionalparams(2:3) + ki*error;
Drive_torque = kp*error + Optionalparams(2:3);
mex_mmap('seteffort',h.Mex_data,[4,5],Drive_torque);
%refreshdata;
%figure(1), subplot(2,2,1), hold on, plot(jointdata(1,1).k
%disp(h.time);
end


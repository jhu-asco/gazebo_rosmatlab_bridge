%Servo Check:
h = GazeboMatlabSimulator;
%% For Arm
h.AttachServo(1,[100.0;20.0;100.0],[10;-10;30;-30]);
h.AttachServo(2,[50.0;10.0;50.0],[10;-10;30;-30]);
%mex_mmap('AttachServo',h.MexData, 0,[100.0;20.0;100.0],[10;-10;30;-30])
%mex_mmap('AttachServo',h.MexData, 1,[50.0;10.0;50.0],[10;-10;30;-30])
h.ActuatedJoints = 1:2;
us = [1.57;1.57];
[LinkData,JointData] = h.Step(10.0, us);
%% For WAM Arm:
h.AttachServo(1:5,[100.0;20.0;100.0],[10;-10;50;-50]); 
h.AttachServo(6:7,[100.0;20.0;100.0],[10;-10;30;-30]);
h.ActuatedJoints = 1:7;
us = [1;-1;1;-1;1;-1;1];%Controls
profile on;
[LinkData,JointData] = h.Step(0.01,us);
profile viewer
profile off;
%% Verify StepTrajectory in GazeboMatlabSimulator:
nofpoints = 20;
h.ActuatedJoints = 1:7;
ts = linspace(0,1, nofpoints);
us = repmat([1;-1;1;-1;1;-1;1],1,nofpoints-1);
profile on;
[LinkData, JointData] = h.StepTrajectory(ts,us);
profile viewer
profile off;
%% Verify StepTrajectory with linkwrenches for WAM:
h.ActuatedJoints = [];
h.ActuatedLinks = 1;
h.AttachServo(1:5,[100.0;20.0;100.0],[30;-30;50;-50]); 
h.AttachServo(6:7,[100.0;20.0;100.0],[30;-30;30;-30]);
nofpoints = 20;
ts = linspace(0,7, nofpoints);
u1 = MatlabLinkInput;
u1.torque(3) = -3.0;
us = repmat({u1},1,nofpoints-1);
%profile on;
[LinkData, JointData] = h.StepTrajectory(ts,[],us);
%profile viewer
%profile off;
%% For Car:
mex_mmap('AttachServo',h.MexData, 0,[2.0;0.1;0.0],[2;-2;5;-5],1)
mex_mmap('AttachServo',h.MexData, 2,[2.0;0.1;0.0],[2;-2;5;-5],1)
mex_mmap('AttachServo',h.MexData, 1,[20.0;5.0;20.0],[10;-10;30;-30])
mex_mmap('AttachServo',h.MexData, 3,[20.0;5.0;20.0],[10;-10;30;-30])
jointid = uint32(0:3);
u1 = [5.0; 0.3];
us = [u1; u1];


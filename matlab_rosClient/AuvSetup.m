function AuvSetup( )
%AuvSetup Sets up the auv vehicle to be controlled by Matlab
%Right of Auv is x axis, forward of Auv is y axis and up is z axis
h = GazeboMatlabSimulator;
h.Reset;
h.Configure(0.001,1);
h.AttachServo([1,2],[50.0;20.0;50.0],[10;-10;30;-30]);%Attach servos to the two fingers;
h.AttachServo(3,[2.0;0.1;0.0],[2;-2;5;-5],1); %Attach Velocity Servo to riser (Note: Position Servo does not work with prismatic links)
h.ActuatedJoints =1:3;
%h.ActuatedLinks = 1;
us_joints = [-0.5; 0.5; 0.3];%Joint Controls
%us_links{1} = MatlabLinkInput;
%us_links{1}.force(3) = 41*9.81;
%Run simulation using step function:
h.Step(2,us_joints);







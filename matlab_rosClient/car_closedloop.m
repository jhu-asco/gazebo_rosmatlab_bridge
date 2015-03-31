function f = car_closedloop()
% Example of trajectory optimization for a 
% WAM Model using shooting and least-squares Gauss Newton method
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

h = GazeboMatlabSimulator;%Creates a Matlab Bridge using a helper class
h.Configure(0.001,1);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1

%%% Problem Setting %%%%
tf = 6;%Time to run the Controller for
x0  = [0;0;0;0.02;0;0]; %Posn(x,y),Angle, Body Velocity of car, steering angle position, steering angle velocity
S.xf = [2.5; -3; 0; 0.02];%Goal Posn(x,y),Angle, Body Velocity of car
S.xi = x0(4);%Dynamic compensator for car controller 
S.l = 0.5; %Length of the car
frequency = 50;%Hz
h.ActuatedJoints = 1:4;%We are actuating all the joints
S.h = (1/frequency);

N = frequency*tf;%Feedback Freq = S.N/tf



%Reset the Physics World
h.Reset();%Reset gazebo world

%Attach servo to all the joints:
h.AttachServo([1,3],[2.0;0.1;0.0],[2;-2;5;-5],1);
h.AttachServo([2,4],[100.0;20.0;100.0],[20;-20;100;-100]);

%Set car to initial posn:
modelstate = MatlabRigidBodyState;
modelstate.position = [ x0(1); x0(2); 0.05];
modelstate.orientation = rpy2quat([x0(3), 0, 0]);
modelstate.linearvelocity(1) = x0(4);
h.SetModelState('Unicycle',modelstate);
pause(0.01);

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
for i = 1:N
  [u,S] = CarController(x,S); % u is the car driving torque and steering torque
  us = [u;u];%Copy us two times for both left and right hand side controls[4x1]
  [LinkData, JointData] = h.Step((1/frequency), us);
  
  %Write new x based on link and joint data:
  LinkState = LinkData{1};
  x(1:2) = LinkState.position(1:2); %Posn of Body
  rpy = quat2rpy(LinkState.orientation);
  x(3) = rpy(3); %Yaw of body
  x(4) = LinkState.linearvelocity(1);%Body velocity 
  x(5:6) = JointData(:,6);%Steering angle and velocity 
  %Map joint angles to -pi to pi:
  A = rem(x(5),2*pi);
  A(A>pi) = A(A>pi)-2*pi;
  A(A<-pi) = A(A<-pi)+2*pi;
  x(5)= A;
  disp('xs:');
  disp(x');
end

function [u,S] = CarController(x,S)
    %x is the current state (x,y,theta,vel,steering angle, steering vel)
    % feedback linearized control-law
    %Virtual output is [xdoubledot, ydoubledot]
    %dynamic compensator is body velocity
    %controls for this kinematic model is steering angle, body velocity of
    %car
    %Output of the car is the position of wheels between axis:
    %Gains for virtual inputs:
    kpv = 0.2;
    kpvd = 1.0;
    y = x(1:2);
    ydot = [x(4)*cos(x(3)); x(4)*sin(x(3))];
    yd = S.xf(1:2);
    ydotd = [S.xf(4)*cos(S.xf(3)); S.xf(4)*sin(S.xf(3))];
    v = kpv*(yd - y) + kpvd*(ydotd - ydot);%Gives the virtual input
    if abs(S.xi) > 0.01
        temp = (S.l/(S.xi^2));
    else
        temp = (S.l*1e4);
    end
    B = [-temp*sin(x(3))    temp*cos(x(3));
           cos(x(3))        sin(x(3))];
    temp = B*v;%temp is xidot(u1dot) and u2
    S.xi = temp(2)*S.h + S.xi;%Integrate the compensator 
    phi = atan(temp(1));%Desired Steering angle
    %Bind steering angle:
    if (phi > pi/4)
        phi = pi/4;
    elseif (phi < -pi/4)
        phi = -pi/4;
    end
    disp('steer: ');
    disp(phi);
    disp('xbar: ');
    disp(atan2(v(2),v(1)));
    u = [20*S.xi; phi];%20 is the conversion from body velocity to joint velocity






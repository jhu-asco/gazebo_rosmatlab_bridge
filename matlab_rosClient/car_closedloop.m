function f = car_closedloop()
% Example of trajectory optimization for a 
% WAM Model using shooting and least-squares Gauss Newton method
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

S.sim = Gazebo_MatlabSimulator;%Creates a Matlab Bridge using a helper class
S.sim.Configure(0.001,1);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1

%%% Problem Setting %%%%
tf = 6;%Time to run the Controller for
x0  = [0;0;0;0.02;0;0]; %Posn(x,y),Angle, Body Velocity of car, steering angle position, steering angle velocity
S.xf = [2.5; -3; 0; 0.02];%Goal Posn(x,y),Angle, Body Velocity of car
S.xi = x0(4);%Dynamic compensator for car controller 
S.l = 0.5; %Length of the car
frequency = 50;%Hz
S.N = frequency*tf;%Feedback Freq = S.N/tfv c
S.h = (tf/S.N);%Timestep
S.nofsteps = round(tf/(S.N*S.sim.physxtimestep));%Internal computation of nof physics steps for one feedback step 
%For example feedback running at 100 Hz (time step 0.01 sec) has 10 physics steps in it, since physics time step is 1 millisecond (configured above)

%Reset the Physics World
mex_mmap('reset',S.sim.Mex_data);
pause(0.01);% Wait for it take effect may not be necessary

%Attach servo to all the joints:
mex_mmap('attachservo',S.sim.Mex_data, 0,[2.0;0.1;0.0],[2;-2;5;-5],1)
mex_mmap('attachservo',S.sim.Mex_data, 2,[2.0;0.1;0.0],[2;-2;5;-5],1)
mex_mmap('attachservo',S.sim.Mex_data, 1,[100.0;20.0;100.0],[20;-20;100;-100])
mex_mmap('attachservo',S.sim.Mex_data, 3,[100.0;20.0;100.0],[20;-20;100;-100])

%Set car to initial posn:
modelposeandtwist = [x0(1:2)' 0.05 rpy2quat([x0(3), 0, 0])' x0(4) zeros(1,5)];%13x1
mex_mmap('setmodelstate',S.sim.Mex_data,'Unicycle',modelposeandtwist);
pause(0.01);

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
jointids = 1:4; %All the joints are actuated
for i = 1:S.N
  [u,S] = CarController(x,S); % u is the car driving torque and steering torque
  us = [u;u];%Copy us two times for both left and right hand side controls[4x1]
  [LinkData, JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(jointids)-1, us, ...
      [], [], uint32([0,S.nofsteps]));
  %Write new x based on link and joint data:
  x(1:2) = LinkData(1:2,2); %Posn of Body
  rpy = quat2rpy(LinkData(4:7,2));
  x(3) = rpy(3); %Yaw of body
  x(4) = LinkData(8,2);%Body velocity 
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
    %Since servos are attached to joints the commands are directly velocity
    %and steering angle
%     %Use PD Controllers to achieve desired steering angle and desired
%     %velocity:
%     u(1,1) = 5.0*(S.xi - x(4));%drivetorque = kp *error_velocity (P)
%     u(2,1) = 1.0*(phi - x(5)) + 1.0*(-x(6));%steeringtorque using pid
%     disp('x: ');
%     disp(x);
%     disp('u: ');
%     disp(u);






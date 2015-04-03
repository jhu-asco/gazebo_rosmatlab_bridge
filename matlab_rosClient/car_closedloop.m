function f = car_closedloop()
% Example of trajectory optimization for a 
% WAM Model using shooting and least-squares Gauss Newton method
%
% Gowtham Garimella ggarime1(at)jhu.edu
% Marin Kobilarov marin(at)jhu.edu

h = GazeboMatlabSimulator;%Creates a Matlab Bridge using a helper class
h.Configure(0.001,2);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1
viz = true; %Set visualization to true
if viz == true
    markerinfo1 = MarkerInfo;%Current Trajectory
    markerinfo1.color = [0;0;1;1];%Blue
    markerinfo1.id = 1;%Unique id to mark the trajectory
    markerinfo2 = MarkerInfo;%Desired Trajectory
end

%%% Problem Setting %%%%
tf = 25;%Time to run the Controller for
x0  = [0;0;0;0.02]; %Posn(x,y),Angle, Body Velocity of car
%S.xf = [2.5; -3; 0; 0.02];%Goal Posn(x,y),Angle, Body Velocity of car
S.center = [0;0];
S.radius = 2;
S.skew = 1.0;
S.omega = 0.3; %Angular velocity
S.xi = x0(4);%Dynamic compensator for car controller 
S.l = 0.5; %Length of the car
S.wheelradius = 0.05;
frequency = 50;%Hz
h.ActuatedJoints = 1:4;%We are actuating all the joints
S.h = (1/frequency);

N = frequency*tf;%Feedback Freq = S.N/tf



%Reset the Physics World
h.Reset();%Reset gazebo world

%Attach servo to all the joints:
h.AttachServo([1,3],[2.0;0.1;0.0],[2;-2;5;-5],1);
h.AttachServo([2,4],[1000.0;100.0;500.0],[100;-100;500;-500]);

%Set car to initial posn:
 modelstate = MatlabRigidBodyState;
 modelstate.position = [ x0(1); x0(2); 0.05];
 modelstate.orientation = rpy2quat([x0(3), 0, 0]);
 modelstate.linearvelocity(1) = x0(4);
 if viz == true
     markerinfo1.action = markerinfo1.MODIFY;
     h.PublishTrajectory(modelstate.position,markerinfo1);
     markerinfo1.action = markerinfo1.ADD;
     S = Finalgoal(0,S);
     markerinfo2.action = markerinfo2.MODIFY;
     h.PublishTrajectory([S.xf(1);S.xf(2);S.wheelradius],markerinfo2);
     markerinfo2.action = markerinfo2.ADD;
 end
 h.SetModelState('Unicycle',modelstate);
 pause(0.01);

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
xs = zeros(4,N);
uall = zeros(2,N);
steer = zeros(2,N);
for i = 1:N
  S = Finalgoal((i-1)*S.h,S);
  [u,S] = CarController(x,S); % u is the car driving torque and steering torque
  uall(:,i) = u;
  u(1) = (1/S.wheelradius)*u(1);%Convert body velocity to joint velocity (wheel velocity)
%  x = Cardynamics(x,u,S);
   us = [u;u];%Copy us two times for both left and right hand side controls[4x1]
   [LinkData, JointData] = h.Step((1/frequency), us);
%   
%   %Write new x based on link and joint data:
   LinkState = LinkData{1};
   x(1:2) = LinkState.position(1:2); %Posn of Body
   rpy = quat2rpy(LinkState.orientation);
   x(3) = rpy(3); %Yaw of body
   x(4) = LinkState.linearvelocity(1)*cos(x(3)) + LinkState.linearvelocity(2)*sin(x(3));%Body velocity of car
   steer(:,i) = JointData(:,2);
   if viz == true
       if rem(i,5) == 0
           h.PublishTrajectory(LinkState.position,markerinfo1);
           h.PublishTrajectory([S.xf(1);S.xf(2);S.wheelradius],markerinfo2); 
       end
   end
%   x(5:6) = JointData(:,6);%Steering angle and velocity 
  %Map joint angles to -pi to pi:
  xs(:,i) = x;
end
ts = (0:N-1)*(S.h);
figure;
subplot(2,2,1),hold on, plot(ts, xs(1,:),'b'), legend('x');
subplot(2,2,2),hold on, plot(ts, xs(2,:),'r'), legend('y');
subplot(2,2,3),hold on, plot(ts,xs(3,:)*(180/pi),'g'), legend('theta(^o)');
subplot(2,2,4),hold on, plot(ts, xs(4,:),'b'), plot(ts, uall(1,:),'r'), legend('vel','veld');
%figure; hold on, plot(xs(1,:),xs(2,:),'b'), plot(S.center(1) + S.radius*cos(S.omega*ts), S.center(2) + S.radius*sin(S.omega*ts),'r'), axis equal, legend('xvsy','xdvsyd');
figure; hold on, plot(ts, steer(1,:),'r'), plot(ts,uall(2,:),'b'), legend('phi','phid');

% function xnew = Cardynamics(x,u,S)
%     xnew = zeros(4,1);
%     xnew(4) = u(1);%Car velocity is equal to whats commanded
%     xnew(1) = x(1) + (S.h)*x(4)*cos(x(3));
%     xnew(2) = x(2) + (S.h)*x(4)*sin(x(3));
%     xnew(3) = x(3) + (S.h)*x(4)*tan(u(2))/S.l;
%     


function [u,S] = CarController(x,S)
    %x is the current state (x,y,theta,vel,steering angle, steering vel)
    % feedback linearized control-law
    %Virtual output is [xdoubledot, ydoubledot]
    %dynamic compensator is body velocity
    %controls for this kinematic model is steering angle, body velocity of
    %car
    %Output of the car is the position of wheels between axis:
    %Gains for virtual inputs:
    kpv = 3.0;
    kpvd = 6.0;
    y = x(1:2);
    ydot = [x(4)*cos(x(3)); x(4)*sin(x(3))];
    yd = S.xf(1:2);
    ydotd = [S.xf(4)*cos(S.xf(3)); S.xf(4)*sin(S.xf(3))];
    v = kpv*(yd - y) + kpvd*(ydotd - ydot);%Gives the virtual input
    %disp('v:')
    %disp(v);
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
%     disp('steer: ');
%     disp(phi);
%     disp('xbar: ');
%     disp(atan2(v(2),v(1)));
     u = [S.xi; phi]; %Only for car model

    
 function S = Finalgoal(t,S)
     S.xf = zeros(4,1);
     S.xf(4) = S.radius*S.omega;%Constant angular velocity
     theta = rem(S.omega*t,2*pi);
     S.xf(3) = rem(theta + (pi/2),2*pi);
     S.xf(1:2) = S.center + S.radius*[cos(theta);S.skew*sin(theta)];






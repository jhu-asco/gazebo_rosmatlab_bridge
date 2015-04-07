function f = car_closedloop()
%CAR_CLOSEDLOOP Example of trajectory optimization for an rccar using feedback linearization of kinematic model
%
% Author:
% Marin Kobilarov marin(at)jhu.edu
%
% Gowtham Garimella ggarime1(at)jhu.edu


h = GazeboMatlabSimulator;%Creates a Matlab Bridge using a helper class
h.Configure(0.001,1);%Configure the physics engine to have a time step of 1 milli second and real time rate is 1
viz = true; %Set visualization to true for plotting trajectories in Gazebo
if viz == true
    markerinfo1 = MarkerInfo;%Convenience class to store information about the trajectory being published
    markerinfo1.color = [0;0;1;1];%Blue Color is provided as RGBA A is the transparency. The numbers should be between 0 and 1
    markerinfo1.id = 1;%Unique id to mark the trajectory. This is used to add points to previous trajectories
    markerinfo2 = MarkerInfo;%Desired Trajectory
end

%%% Problem Setting %%%%
tf = 25;%Time to run the Controller for
x0  = [0;-0.3;pi/6;0.02]; %Posn(x,y),Angle, Body Velocity of car
%Desired trajectory setting:
S.center = [0;0];
S.radius = 2;
S.skew = 1.0;
S.omega = 0.3; %Angular velocity
%Helper variables;
S.xi = x0(4);%Dynamic compensator for car controller 
S.l = 0.3; %Length of the car
S.wheelradius = 0.05;%Wheel radius
frequency = 50;%Controller frequency in Hz
h.ActuatedJoints = 1:4;%We are actuating all the joints
S.h = (1/frequency);

N = frequency*tf;%Feedback Freq = S.N/tf



%Reset the Physics World
h.Reset();%Reset gazebo world

%Attach servo to all the joints:
h.AttachServo([1,3],[2.0;0.0;0.0],[2;-2;5;-5],1);%Joints for which servos should be attached, gains(PID), limits(integral limits and command limits), control type: position(0)/velocity control(1)
h.AttachServo([2,4],[100.0;100.0;50.0],[100;-100;500;-500]);

%Set car to initial posn:
 modelstate = MatlabRigidBodyState;%Helper class to store rigidbody state
 modelstate.position = [ x0(1); x0(2); S.wheelradius];%Initial position
 modelstate.orientation = rpy2quat([x0(3), 0, 0]);%Initial orientation
 modelstate.linearvelocity(1) = x0(4);%Initial velocity
 if viz == true
     markerinfo1.action = markerinfo1.MODIFY;%Add initial point to trajectory
     h.PublishTrajectory(modelstate.position,markerinfo1);
     markerinfo1.action = markerinfo1.ADD;
     S = Finalgoal(0,S);
     markerinfo2.action = markerinfo2.MODIFY;%Add initial desired point to trajectory
     h.PublishTrajectory([S.xf(1);S.xf(2);S.wheelradius],markerinfo2);
     markerinfo2.action = markerinfo2.ADD;
 end
 h.SetModelState('Unicycle',modelstate);%Set the model state. Joints are not modified.
 pause(0.01);

%%% Feedback Loop Starts %%%%
x = x0;%Temporary state
xs = zeros(4,N);
uall = zeros(2,N);
steer = zeros(2,N);
for i = 1:N
    S = Finalgoal((i-1)*S.h,S);%Get the current desired state
    [u,S] = CarController(x,S); % u is the car driving velocity and steering angle
    uall(:,i) = u;%Store for visualization
    u(1) = (1/S.wheelradius)*u(1);%Convert body velocity to joint velocity (wheel velocity)
    us = [u;u];%Copy us two times for both left and right hand side joints
    [LinkData, JointData] = h.Step((1/frequency), us);%Run Gazebo Dynamics
    %Write new state x based on link and joint data:
    LinkState = LinkData{1};
    rpy = quat2rpy(LinkState.orientation);
    x(3) = rpy(3); %Yaw of body
    x(1:2) = LinkState.position(1:2); %Posn of Body
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
    kpvd = 3.0;
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
     u = [S.xi; phi]; %Only for car model

    
 function S = Finalgoal(t,S)
     S.xf = zeros(4,1);
     S.xf(4) = S.radius*S.omega;%Constant angular velocity
     theta = rem(S.omega*t,2*pi);
     S.xf(3) = rem(theta + (pi/2),2*pi);
     S.xf(1:2) = S.center + S.radius*[cos(theta);S.skew*sin(theta)];






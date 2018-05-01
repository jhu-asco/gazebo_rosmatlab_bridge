function cyton_servo_control(  )
%cyton_servo_control Control Cyton arm using servos.
sim = GazeboMatlabSimulator;
sim.Reset;
tf = 4;
h = 0.01;%Step for only visualization as servo control is happening at 1Khz in plugin
N = round(tf/h);%Number of steps to run
%For 10 seconds:
% sim.AttachServo(1,[4.0;0.2;0.0],[50;-50;1000;-1000]); %Attach servos for joint angle control
% sim.AttachServo(2,[4.0;1.0;0.0],[20;-20;100;-100]);
% sim.AttachServo(3,[4.0;1.0;0.0],[20;-20;100;-100]);
% sim.AttachServo(4:7,[2.0;0.5;0.0],[5;-5;100;-100]);
% sim.AttachServo(8:9,[0.1;0.0;0.0],[2;-2;30;-30],1);
% For 2 seconds:
sim.AttachServo(1,[10.0;5.0;1.0],[50;-50;1000;-1000]); %Attach servos for joint angle control
sim.AttachServo(2,[5.0;5.0;1.0],[20;-20;100;-100]);
sim.AttachServo(3,[5.0;5.0;1.0],[20;-20;100;-100]);
sim.AttachServo(4:7,[5.0;5.0;0.0],[5;-5;100;-100]);
sim.AttachServo(8:9,[0.5;0.0;0.0],[2;-2;30;-30],1);

sim.ActuatedJoints = 1:9;
us = [1;-1;1;-1;1;-1;1;-0.05;0.05];%Desired Joint Angles

%Visualization:
m = MarkerInfo;
sim.PublishTrajectory([0.639485; 0; 1.0],m);%Initialposition from foward kin
m.action = m.ADD;
uall = zeros(9,N);
for i = 1:N
    [LinkData,JointData] = sim.Step(h,us);%Run simulation. Servo control is happening with desired joint angles as us
    sim.PublishTrajectory(LinkData{1}.position,m);%Publish the current end effector position.
    uall(:,i) = JointData(1,:)';
end
figure; 
subplot(4,2,1), plot((1:N)*h,uall(1,:)), hold on, plot((1:N)*h,us(1)*ones(1,N),'r'),xlabel('t'), ylabel('u1');
subplot(4,2,2), plot((1:N)*h,uall(2,:)), hold on, plot((1:N)*h,us(2)*ones(1,N),'r'),xlabel('t'), ylabel('u2');
subplot(4,2,3), plot((1:N)*h,uall(3,:)), hold on, plot((1:N)*h,us(3)*ones(1,N),'r'),xlabel('t'), ylabel('u3');
subplot(4,2,4), plot((1:N)*h,uall(4,:)), hold on, plot((1:N)*h,us(4)*ones(1,N),'r'),xlabel('t'), ylabel('u4');
subplot(4,2,5), plot((1:N)*h,uall(5,:)), hold on, plot((1:N)*h,us(5)*ones(1,N),'r'),xlabel('t'), ylabel('u5');
subplot(4,2,6), plot((1:N)*h,uall(6,:)), hold on, plot((1:N)*h,us(6)*ones(1,N),'r'),xlabel('t'), ylabel('u6');
subplot(4,2,7), plot((1:N)*h,uall(7,:)), hold on, plot((1:N)*h,us(7)*ones(1,N),'r'),xlabel('t'), ylabel('u7');
figure;
subplot(4,2,1), plot((1:N)*h,uall(8,:)), hold on, plot((1:N)*h,us(8)*ones(1,N),'r'),xlabel('t'), ylabel('u8');
subplot(4,2,2), plot((1:N)*h,uall(9,:)), hold on, plot((1:N)*h,us(9)*ones(1,N),'r'),xlabel('t'), ylabel('u9');

end


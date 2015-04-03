function wam_servo_control(  )
%wam_servo_control Control WAM arm using servos.
sim = GazeboMatlabSimulator;
sim.Reset;
tf = 10;
h = 0.1;%Step for only visualization as servo control is happening at 1Khz in plugin
N = round(tf/h);
sim.AttachServo(1:5,[100.0;20.0;100.0],[10;-10;50;-50]); 
sim.AttachServo(6:7,[100.0;20.0;100.0],[10;-10;30;-30]);
sim.ActuatedJoints = 1:7;
us = [1;-1;1;-0.5;1;-1;1];%Desired Joint Angles
%Visualization:
m = MarkerInfo;
sim.PublishTrajectory([1.2462; 0; 0.7810],m);%Initialposition from foward kin
m.action = m.ADD;
uall = zeros(7,N);
for i = 1:N
    [LinkData,JointData] = sim.Step(h,us);
    sim.PublishTrajectory(LinkData{1}.position,m);
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


end


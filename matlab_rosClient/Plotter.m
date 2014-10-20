% Feedback loop. Addition arguments can be specified inside optional
% parameters in h.
h.times = h.times(1:h.count_msgs(1));
id1 = 1;
id2 = 2;
figure(1), subplot(2,2,1), hold on, plot(h.times, h.LinkPoses{1}(1,1:h.count_msgs(1)),'b*-');%X
figure(1), subplot(2,2,2), hold on, plot(h.times, h.LinkPoses{1}(2,1:h.count_msgs(1)),'b*-');%Y
figure(1), subplot(2,2,3), hold on, plot(h.times, h.LinkPoses{1}(3,1:h.count_msgs(1)),'b*-');%Z
figure(2), subplot(2,2,1), hold on, plot(h.times, h.JointAngles{id1}(1,1:h.count_msgs(1)), 'b*-');
figure(2), subplot(2,2,2), hold on, plot(h.times, h.JointVelocities{id1}(1,1:h.count_msgs(1)),'r*-');
figure(2), subplot(2,2,3), hold on, plot(h.times, h.JointAngles{id2}(1,1:h.count_msgs(1)), 'b*-');
figure(2), subplot(2,2,4), hold on, plot(h.times, h.JointVelocities{id2}(1,1:h.count_msgs(1)),'r*-');
figure(3), subplot(2,2,1), hold on, plot(h.times, h.LinkTwists{1}(1,1:h.count_msgs(1)),'r*-');%VX or some close relative of that
figure(3), subplot(2,2,2), hold on, plot(h.times, h.LinkTwists{1}(2,1:h.count_msgs(1)),'r*-');%VY
figure(3), subplot(2,2,3), hold on, plot(h.times, h.LinkTwists{1}(3,1:h.count_msgs(1)),'r*-');%VZ
%Will add yaw
%Add Control Law here
%Use available names to set torques etc



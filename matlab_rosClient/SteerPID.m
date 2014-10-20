function SteerPID(h,evtdata)
%STEERPID PID controller for Steering
%kp = 50;
%kd = 60;
kp = 8;
kd = 8;
%ki = 0.05;
if h.count_msgs(1) == 0%if count_msgs is too small
    return
end
id1 = 1;
id2 = 2;
steering_angle1 = h.JointAngles{id1}(1,h.count_msgs(1));
steering_vel1 = h.JointVelocities{id1}(1,h.count_msgs(1));
steering_angle2 = h.JointAngles{id2}(1,h.count_msgs(1));
steering_vel2 = h.JointVelocities{id2}(1,h.count_msgs(1));
Steer_torque1 = kp*(h.Optional_params{1}-steering_angle1) - kd*(steering_vel1);
Steer_torque2 = kp*(h.Optional_params{1}-steering_angle2) - kd*(steering_vel2);
h.setEffort(h.Joint_names{1},Steer_torque1,h.step);
h.setEffort(h.Joint_names{2},Steer_torque2,h.step);
%disp(h.time);
end


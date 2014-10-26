buffer = StateBuffer({[],[1,2]},300);
buffer.updatepoint = 10;%0.1 secs
h = Gazebo_MatlabSimulator;
h.inds = {[],[1,2]};
%%
h.feedback_handles(1:2,1) = {@buffer.add_data,[]};
h.feedback_handles(1:2,2) = {@ArmPID,[0.2 0.2 0 0]};
%h.feedback_handles(1:2,3) = {@DrivePID,[0.5 0 0]};
buffer = StateBuffer({[],[1,2]},300);
buffer.updatepoint = 10;%0.1 secs
h = Gazebo_MatlabSimulator;
h.inds = {[],[1,2]};
%%
h.feedback_handles(1:2,1) = {@buffer.add_data,[]};
h.feedback_handles(1:2,2) = {@ArmPID,[0.2 0.2 0 0]};
%%
h.starttimer(0.01);
%mex_mmap('setjointstate',h.Mex_data,1,[1,0,0,0,0,0]);
%mex_mmap('setmodelstate',h.Mex_data,'double_pendulum_with_base',[2,8,0.3, 1,0,0,0, zeros(1,6)]')
%mex_mmap('setgazebostate',h.Mex_data,'reset');
%
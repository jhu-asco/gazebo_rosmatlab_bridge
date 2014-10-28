buffer = StateBuffer({[1],[1,3]},200);
buffer.updatepoint = 5;%0.1 secs
h = Gazebo_MatlabSimulator;
h.inds = {[1],[1,2,3,4]};
%%
h.feedback_handles(1:2,1) = {@buffer.add_data,[]};
h.feedback_handles(1:2,2) = {@SteerPID,[0.2 0 0]};
%pause(0.5);
h.feedback_handles(1:2,3) = {@DrivePID,[5 0 0]};
%%
h.starttimer(0.01);
%mex_mmap('setmodelstate',h.Mex_data,'Unicycle',[2,-2,0.05, 1,0,0,0,
%zeros(1,6)]');
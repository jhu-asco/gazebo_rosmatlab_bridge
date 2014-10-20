% Example Script to use the Gazebo Matlab Simulator class:
h = Setup_Matlabsimulator;
%%
my_jointnames = {'Unicycle::rear_left_wheel_joint','Unicycle::rear_right_wheel_joint','Unicycle::base_to_steeringblock1','Unicycle::base_to_steeringblock2'};
%Set Links to get information about links:
%h.Link_names = {'Unicycle::carbody'};%Links we need information about. %If we need information about Joints we can specify in h.Joint_names
%h.Joint_names = {my_jointnames{3},my_jointnames{4}};
%h.runsimulation(1);
%listener = addlistener(h,'ClockEvent',@CarFeedback);
h.Optional_params{1} =  0;%Desired Steering angle
h.callback_handles = {'SteerPID'};
h.runsimulation(1);
pause(1);
Plotter;
%listener1 = addlistener(h,'ClockEvent',@SteerPID);
%%
pause(1);
t = 10;
torque = 0.2;
%torque_steer = -0.1;
h.Optional_params{1} =  0.2;%Desired  Steering angle
h.setEffort(my_jointnames{1},torque,t/2);
h.setEffort(my_jointnames{2},torque,t/2);
%h.setEffort(my_jointnames{3},torque_steer,t/2);
%h.setEffort(my_jointnames{4},torque_steer,t/2);
h.runsimulation(t/2);
 pause(t/2);
 Plotter;
h.Optional_params{1} = -0.2;
h.setEffort(my_jointnames{1},-torque/2,t/2);
h.setEffort(my_jointnames{2},-torque/2,t/2);
% h.setEffort(my_jointnames{3},-torque_steer,t/2);
% h.setEffort(my_jointnames{4},-torque_steer,t/2);
h.runsimulation(t/2);
pause(t/2);
Plotter;
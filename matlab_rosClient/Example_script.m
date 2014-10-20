% Example Script to use the Gazebo Matlab Simulator class:

h = Gazebo_MatlabSimulator;
h.runsimulation(0.01);
h.resetworld;
h.Available_Names
%Set Joints to get angles and velocities of:
h.Joint_names = {'Wamwitharm::iri_wam::j2_joint'};
%Set Links to get information about links:
%h.Link_names = {}
global count;
count = 0;
listener = addlistener(h,'NewJointData',Response_to_joint_data);
%%
delay(1);
h.runsimulation(1);


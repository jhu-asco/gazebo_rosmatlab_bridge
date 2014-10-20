function [ Class_handle ] = Setup_Matlabsimulator()
%SETUP_MATLABSIMULATOR Creates the Gazebo class and provides the available
%names.
Class_handle = Gazebo_MatlabSimulator;
Class_handle.runsimulation(0.01);
Class_handle.resetworld;
end


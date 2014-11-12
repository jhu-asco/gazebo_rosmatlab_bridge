classdef Gazebo_MatlabSimulator < handle
    %Gazebo client which handles the communication with gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        Available_Names = cell(1,2);%Helper Property which provides the available names to use
        count_msgs = zeros(2,1);% Number of messages received so far in the order: Links, Models, Clock. 
        physxtimestep = 0.001;
        Mex_data; %Stored data for mex
        %mode = 'closedloop';%Can be either openloop or closedloop
    end
    methods (Access = public)
        function h = Gazebo_MatlabSimulator()
            %Starting Mex File:
            h.Mex_data = mex_mmap('new');
            [h.Available_Names{1}, h.Available_Names{2}] = mex_mmap('availablenames',h.Mex_data);
        end
        function delete(h)
            % Destructor.
            mex_mmap('delete',h.Mex_data);
        end
        
        function [LinkData, JointData] = RunSimulation(h,steps,function_handle)
            time = h.physxtimestep*steps;
            [linkids, bodywrenches, jointids, jointefforts] = feval(function_handle,time);
            [LinkData, JointData] = mex_mmap('runsimulation',h.Mex_data, uint32(jointids)-1, jointefforts, ...
                                                    uint32(linkids)-1, bodywrenches, uint32(steps));
        end
        function Configure(h,timestep,rate)
            if nargin == 2
                rate = 20;
            end
            rate = rate/timestep;%Frequency of physics engine
            mex_mmap('configurephysics',h.Mex_data,timestep,rate);
            h.physxtimestep = timestep;
        end
    end
end

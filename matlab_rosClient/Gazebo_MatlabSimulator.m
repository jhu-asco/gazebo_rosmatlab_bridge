classdef Gazebo_MatlabSimulator < handle
    %Gazebo client which handles the communication with gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        Available_Names = cell(1,2);%Helper Property which provides the available names to use
        %count_msgs = zeros(2,1);% Number of messages received so far in the order: Links, Models, Clock. 
        physxtimestep = 0.001;
        Mex_data; %Stored data for mex
        LinkData;%Link Data is array of rigid body states
        JointData;%Joint Data is 2xn with joint positions and velocities
        Servos;
        Escs;
        ActuatedJoints;%Index array
        ActuatedLinks;%Index array
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
%         function attachservo(h, jointindex, servogains, limits, control_type)
%             mex_mmap('attachservo',h.Mex_data, jointindex, servogains, limits, control_type);
%         end
        %function Step(h, ts, us_joints, us_links)
            %Run the simulation of gazebo with given stepsize and tf.
            %us_joints should be of size as ts with number of rows =
            %nofactuated joints
            %us_links should be a cell array of link wrenches applied.
            % if us_joints or us_links is empty, then they are not applied
        %end
        
%         function [LinkData, JointData] = RunSimulation(h,steps,function_handle)
%             time = h.physxtimestep*steps;
%             [linkids, bodywrenches, jointids, jointefforts] = feval(function_handle,time);
%             [LinkData, JointData] = mex_mmap('runsimulation',h.Mex_data, uint32(jointids)-1, jointefforts, ...
%                                                     uint32(linkids)-1, bodywrenches, uint32(steps));
%         end
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

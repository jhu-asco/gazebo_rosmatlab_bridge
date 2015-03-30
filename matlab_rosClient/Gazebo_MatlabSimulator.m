classdef Gazebo_MatlabSimulator < handle
    %Gazebo client which handles the communication with gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        Available_Names = cell(1,2);%Helper Property which provides the available names to use
        %count_msgs = zeros(2,1);% Number of messages received so far in the order: Links, Models, Clock. 
        physxtimestep = 0.001;
        Mex_data; %Stored data for mex
        %LinkData;%Link Data is array of rigid body states
        %JointData;%Joint Data is 2xn with joint positions and velocities
        ActuatedJoints  = [];%Index array
        ActuatedLinks = [];%Index array
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
         function attachservo(h, jointindices, servogains, limits, control_type)
             %Attaches a servo to given joint indices 
             %servogains: PID gains as 3x1 vector
             %limits: [Integralterm_max; Integralterm_min; max_torque;
             %min_torque]
             %control_type: Position control(0), velocity control(1). In velocity
             %control Derivative gain does not have any effect
             if nargin <= 3
                 control_type = 0;
             end
             if nargin <= 2
                 limits = zeros(4,1);
             end
             for i = 1:length(jointindices)
                mex_mmap('attachservo',h.Mex_data, jointindices(i)-1, servogains, limits, control_type);
             end
         end
        function [LinkStates, JointState] = Step(h, deltat, us_joints, us_links)
            %Run a single step of size (stepsize) of gazebo
            % us_joints = torques[nofactuatedjointsx1];
            % If servo is attached to the joint, the us_joint(i) becomes
            % commanded position/velocity to servo
            % us_links = cellarrayoflinkinputs[nofactuatedlinks];
            jointids = uint32(h.ActuatedJoints-1);
            if nargin == 4
                linkids = uint32(h.ActuatedLinks-1);
                linkinputs = zeros(6,length(h.ActuatedLinks));
                for count = 1:size(linkinputs,2)
                    linkinputs(count,:) = us_links{count}.getdata();
                end
            else
                linkids = [];
                linkinputs = [];
            end
            steps = uint32([0 round(deltat/h.physxtimestep)]);
            [LinkData, JointState] = mex_mmap('runsimulation',h.Mex_data, jointids, us_joints, ...
                                                     linkids, linkinputs, steps);
            LinkStates = cell(size(LinkData,2));
            for i = 1:size(LinkdData,2)
                LinkStates{i} = MatlabRigidBodyState(LinkData(i,:));
            end
        end
        function [LinkStates, JointStates] = StepTrajectory(h, ts, us_joints, us_links)
            %This runs multiple steps at once
            jointids = uint32(h.ActuatedJoints-1);
            N = length(ts)-1;%Number of segments or nofcontrols
            if nargin == 4
                linkids = uint32(h.ActuatedLinks-1);
                linkinputs = zeros(6,length(h.ActuatedLinks)*N);
                for count = 1:size(linkinputs,2)
                    linkinputs(count,:) = us_links{count}.getdata();
                end
            else
                linkids = [];
                linkinputs = [];
            end
            steps = uint32(ts);
            [LinkData, JointStates] = mex_mmap('runsimulation',h.Mex_data, jointids, us_joints, ...
                linkids, linkinputs, steps);
            LinkStates = cell(size(LinkData,2));
            
            for i = 1:size(LinkdData,2)
                LinkStates{i} = MatlabRigidBodyState(LinkData(i,:));
            end
        end
        
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

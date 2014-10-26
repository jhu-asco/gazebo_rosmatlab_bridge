classdef Gazebo_MatlabSimulator < handle
    %Gazebo client which handles the communication with gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        Available_Names = {};%Helper Property which provides the available names to use
        count_msgs = zeros(2,1);% Number of messages received so far in the order: Links, Models, Clock.
        feedback_handles = {};%Array for timer to run callback handles and optional params as [2xn] cell array(Make this protected #TODO)
        LinkData = []; %Link Data
        JointData = []; %Joint Data;
        t  = []; %Time from link and Joint Data
        inds = {};%Can make these into names and internally convert into inds and move this into private member
        Mex_data; %Stored data for mex
        duration = Inf;%How much time the timer should run
        mode = 'closedloop';%Can be either openloop or closedloop
    end
    properties (Access = protected)
        feedback_timer;%Timer for running Feedback handles
        timerstart;%Simulation time when the timer started
    end
%     events
%         ClockEvent
%     end
    methods (Access = public)
        function h = Gazebo_MatlabSimulator()
            %Starting Mex File:
            h.Mex_data = mex_mmap('new');
            h.Available_Names = mex_mmap('availablenames',h.Mex_data);
            h.feedback_timer = timer('TimerFcn',{@(~,~,x)mytimerFcn(x),h}...
                                ,'Period',0.01,'ExecutionMode','fixedRate');
            %start(h.feedback_timer);%Start the timer;
        end
        function delete(h)
            % Destructor.
            stop(h.feedback_timer);
            h.feedback_handles = {};
            clear h.feedback_timer;
            mex_mmap('delete',h.Mex_data);
        end
        function stoptimer(h)
            stop(h.feedback_timer);%Stopping feedback timer
        end
        function starttimer(h,timeperiod,duration)
            %Duration is optional
            stop(h.feedback_timer);
            if nargin >= 2
                set(h.feedback_timer,'Period',timeperiod);
            end
            if nargin == 3
                h.duration = duration;
            end
            h.timerstart = mex_mmap('readtime',h.Mex_data);%Read the current simulation time
            start(h.feedback_timer);
        end
    end
    methods (Access = private)
        function mytimerFcn(h) 
            %Check if timer should be stopped:
            
            %Collect Data
            if ~isempty(h.inds)
                if strcmp(h.mode,'closedloop')
                    [h.LinkData,h.JointData,h.t] = mex_mmap('readlinkjoint',h.Mex_data,h.inds);%Inds can be changed into names later
                elseif strcmp(h.mode,'openloop')
                    h.t = mex_mmap('readtime',h.Mex_data);%Read the current simulation time
                end
                if (h.t - h.timerstart) >= h.duration
                    disp('h.t, h.timerstart');
                    disp(h.t);
                    disp(h.timerstart);
                    h.stoptimer;%Stop the timer;
                end
                for i = 1:size(h.feedback_handles,2)
                    if ~isempty(h.feedback_handles{2,i})
                        h.feedback_handles{2,i} = feval(h.feedback_handles{1,i},h,h.feedback_handles{2,i});
                    else
                        feval(h.feedback_handles{1,i},h);
                    end
                end
            end
        end
    end
end

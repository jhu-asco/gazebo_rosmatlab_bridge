classdef GazeboMatlabSimulator < handle
    %Matlab client which handles the communication with Gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        AvailableNames = cell(1,2);%Helper Property which provides the available names to use. The first cell provides links available, second cell provides joints available
        PhysicsTimeStep = 0.001;%Time step for running physics engine should be configured using Configure function
        MexData; %Stored data for mex (Internal)
        ActuatedJoints  = [];%Index of joints being actuated. (The index starts with 1 and is based on available names provided above) 
        ActuatedLinks = [];%Index links being actuated. (The index is based on Availableindices)
    end
    methods (Access = public)
        function h = GazeboMatlabSimulator()
            %Starts Matlab Gazebo bridge and provides available links and joints
            h.MexData = mex_mmap('new');
            [h.AvailableNames{1}, h.AvailableNames{2}] = mex_mmap('availablenames',h.MexData);
        end
        function delete(h)
            % Destructor. Should be called before running another bridge.
            mex_mmap('delete',h.MexData);
        end
        function AttachServo(h, jointindices, servogains, limits, control_type)
            %Attaches a servo to given joint indices
            %servogains: PID gains as 3x1 vector
            %limits: [Integralterm_max; Integralterm_min; max_torque;
            %min_torque]
            %control_type: Position control(0), velocity control(1). In velocity
            %control Derivative gain does not have any effect
            if nargin <= 4
                control_type = 0;
            end
            if nargin <= 3
                limits = zeros(4,1);
            end
            if nargin <= 2
                servogains = [1;0;1];
            end
            for i = 1:length(jointindices)
                mex_mmap('attachservo',h.MexData, jointindices(i)-1, servogains, limits, control_type);
            end
        end
        function [LinkStates, JointState] = Step(h, deltat, us_joints, us_links)
            %Run a single step of size (stepsize) of gazebo with given controls
            % us_joints = torques[nofactuatedjointsx1];
            % If servo is attached to the joint, the us_joint(i) becomes
            % commanded position/velocity to servo
            % us_links = cellarrayoflinkinputs[nofactuatedlinks];
            % us_links{i} should be of type MatlabLinkInput
            % LinkStates{i} is of type MatlabRigidBodyState
            % see also MATLABLINKINPUT (class) MATLABRIGIDBODYSTATE (class)
            jointids = uint32(h.ActuatedJoints-1);
            if nargin == 4
                linkids = uint32(h.ActuatedLinks-1);
                linkinputs = zeros(6,length(h.ActuatedLinks));
                for count = 1:size(linkinputs,2)
                    linkinputs(:,count) = us_links{count}.getdata();
                end
            else
                linkids = [];
                linkinputs = [];
            end
            steps = uint32([0 round(deltat/h.PhysicsTimeStep)]);
            [LinkData, JointState] = mex_mmap('runsimulation',h.MexData, jointids, us_joints, ...
                linkids, linkinputs, steps);
            nofjoints = length(h.AvailableNames{2});
            JointState = JointState(:,(nofjoints+1):(2*nofjoints));
            noflinks = length(h.AvailableNames{1});
            LinkStates = cell(noflinks,1);
            for i = 1:noflinks
                LinkStates{i} = MatlabRigidBodyState(LinkData(:,i+noflinks));
            end
        end
        function [LinkStates, JointStates] = StepTrajectory(h, ts, us_joints, us_links)
            %This runs multiple steps at once and provides the link and joint states at all the times ts
            % trajectory is represented (ts, xs) 
            % controls us are given as (us_joints, us_links)
            % If a trajectory has N segments then ts and xs are of size (N+1)
            % xs is LinkStates a cell array of (nofavailablelinksx(N+1));
            % and JointStates (2x(nofavailablejointsx(N+1)));
            % us_joints is 1x(nofavailablejointsxN) torques
            % us_links is a cell array of noflinksxN with each element
            % being a linkinput Class (force and torque)
            % 
            jointids = uint32(h.ActuatedJoints-1);
            N = length(ts)-1;%Number of segments or nofcontrols
            if nargin == 4
                linkids = uint32(h.ActuatedLinks-1);
                linkinputs = zeros(6,length(h.ActuatedLinks)*N);
                for count = 1:size(linkinputs,2)
                    row = rem(count-1,h.ActuatedLinks)+1;
                    col = round((count - row)/h.ActuatedLinks)+1;
                    linkinputs(:,count) = us_links{row,col}.getdata();
                end
            else
                linkids = [];
                linkinputs = [];
            end
            steps = uint32(round(ts/h.PhysicsTimeStep));
            [LinkData, JointStates] = mex_mmap('runsimulation',h.MexData, jointids, us_joints, ...
                linkids, linkinputs, steps);
            noflinks = length(h.AvailableNames{1});
            LinkStates = cell(noflinks,N+1);
            
            for j = 1:(N+1)
                for i = 1:noflinks
                    LinkStates{i,j} = MatlabRigidBodyState(LinkData(:,i+(j-1)*noflinks));
                end
            end
        end

        function Configure(h,timestep,rate)
            % Configure the physics engine to run with time step 
            % The rate is the how fast the physics engine should run
            % compared to real time.
            % For example rate = 5 implies, the physics engine tries to run
            % 5 times faster than real time. Although this is bounded by
            % the resources available.
            if nargin == 2
                rate = 1;
            end
            rate = rate/timestep;%Frequency of physics engine
            mex_mmap('configurephysics',h.MexData,timestep,rate);
            h.PhysicsTimeStep = timestep;
        end
        function SetModelState(h,model_name, modelstate_, jointstates, jointinds)
            %Set the Model state(Pose) and joint angles of the joints
            %If joints are not set, then set only modelstate
            % If only joints are set, set modelstate = [];
            % if jointinds are same as actuated joints, then leave out
            % jointinds argument
            if nargin <=3
                jointstates = [];
                jointinds = [];
            elseif nargin == 4
                jointinds = uint32(h.ActuatedJoints-1);
            else
               jointinds = uint32(jointinds-1); 
            end
            
            if ~isempty(modelstate_)
                modelstate = modelstate_.getdata();
            else
                modelstate = [];
            end
            
            mex_mmap('setmodelstate',h.MexData,model_name,modelstate,...
    jointinds,jointstates); %Set the Joint Angles without worrying about the pendulum position
        end
        function PublishTrajectory(h,data,markerinfo)
            %Publish the trajectory:
            if markerinfo.action == markerinfo.DELETE
                %Delete the line:
                mex_mmap('publishtrajectory',h.MexData,[],markerinfo.id,markerinfo.MODIFY);%Clear the line points
                mex_mmap('publishtrajectory',h.MexData,[],markerinfo.id,markerinfo.DELETE);%Then Delete line
            else
                data(1:2,:) = 0.01*data(1:2,:);%Some scaling issue
                mex_mmap('publishtrajectory',h.MexData, data, markerinfo.id, markerinfo.action, markerinfo.color);
            end
        end
        function Reset(h)
            %Reset the Gazebo world
            mex_mmap('reset',h.MexData)
        end
    end
end

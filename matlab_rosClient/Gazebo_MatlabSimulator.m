classdef Gazebo_MatlabSimulator < handle
    %Gazebo client which handles the communication with gazebo.
    %Author Gowtham Garimella (ggarime1@jhu.edu)
    properties (Access = public)
        Available_Names = {};%Helper Property which provides the available names to use
        Link_names = {}; %Array of links to find the poses and twists of. We dont care about others
        %Full name of the link must be given for example coke_can::link or
        %double_pendulum_base::base etc
        Model_names = {};%Array of Models to find the poses and twists of.
        Joint_names = {};%Array of joint_names to find the angular rates etc
        ImuOrientation = [];%Imu Data in Quaternion
        OdomPose = [];%Odometry pose as [x,y,z Qw, Qx, Qy, Qz]
        LaserData = {};%Laser Data range and angle vectors as an array
        %Odometr
    end
    properties (Access = protected)
        Node = []; %Ros node for communication
        % Default Publishers:
        bodywrench_pub = [];% apply_bodywrench (geometry_msgs/WrenchStamped)
        jointeffort_pub = [];% apply_jointeffort (geometry_msgs/Vector3Stamped)
        modelstate_pub = [];% set_modelstate (gazebo_msgs/ModelState)
        runsimulation_pub = [];% run_simulation (std_msgs/Duration)
        resetworld_pub = [];% reset_world (std_msgs/Empty)
        clearbodywrench_pub = []; %Clear Bodywrench on a body
        %Default Subscribers:
        linkstates_sub = [];% link_states (gazebo_msgs/LinkStates)
        modelstates_sub = [];% model_states (gazebo_msgs/ModelStates)
        ImuSub = [];                % ROS subscriber for IMU data.
        OdomSub = [];               % ROS subscriber for odometry data.
        LaserScanSub = [];          % ROS subscriber for laser scans.
        %Messages:
        BodyWrench_msg = [];
        JointEffort_msg = [];
        ModelState_msg = [];
        runsimulation_msg = [];
        resetworld_msg = [];
        clearwrench_msg = [];
        %Internal Pose data of the Link names specified
        LinkPoses = {};%array of 7x1 vectors [x,y,z,qw, qx, qy, qz]
        LinkTwists = {}; %Array of 6x1 vectors [ vx, vy, vz, wx, wy, wz]
        ModelPoses = {};%array of 7x1 vectors [x,y,z,qw, qx, qy, qz]
        ModelTwists = {};%Array of 6x1 vectors [ vx, vy, vz, wx, wy, wz]
        JointAngles = {};%Array of vectors of 3x1 (Actual interpretation depends on the joint)
        JointVelocities = {};% Array of vectors of 3x1 (Actual interpretation depends on joint)
       % count = 0;
    end
    events
        %Add More events for other sensors if needed
        NewLinkData
        NewModelData
        NewJointData
        NewOdomData
        NewLaserScanData
        NewImuData
        %Add any other sensor events you want to listen here
        %Can also combine all the sensors into a single(NewSensorData) event to reduce
        %clutter if needed
    end
    methods (Access = public)
        function h = Gazebo_MatlabSimulator()
            h.Node = rosmatlab.node('matlab_client', 'localhost',11311);
            % Add publishers:
            h.bodywrench_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/apply_bodywrench', 'geometry_msgs/WrenchStamped');
            h.jointeffort_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/apply_jointeffort', 'geometry_msgs/Vector3Stamped');
            h.modelstate_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/set_modelstate', 'gazebo_msgs/ModelState');
            h.runsimulation_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/run_simulation', 'std_msgs/Duration');
            h.resetworld_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/reset_world', 'std_msgs/Empty');
            h.clearbodywrench_pub = h.Node.addPublisher('/gazebo_rosmatlab_bridge/clear_bodywrench','std_msgs/String');
            disp('Setting Publishers done');
            % Add subscribers:
            h.linkstates_sub = h.Node.addSubscriber('/gazebo/link_states', 'gazebo_msgs/LinkStates', 100);
            h.modelstates_sub = h.Node.addSubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates',100);
            h.ImuSub = h.Node.addSubscriber('/mobile_base/sensors/imu_data', 'sensor_msgs/Imu', 25);%put the correct topic name or rename in launch file
            h.OdomSub = h.Node.addSubscriber('/odom', 'nav_msgs/Odometry', 25);
            h.LaserScanSub = h.Node.addSubscriber('/scan', 'sensor_msgs/LaserScan', 5);
            disp('Setting Subscribers done');
            %Set Listeners:
            h.linkstates_sub.setOnNewMessageListeners({@h.linkstateCallback});
            h.modelstates_sub.setOnNewMessageListeners({@h.modelstateCallback});
            h.LaserScanSub.setOnNewMessageListeners({@h.laserCallback});
            h.OdomSub.setOnNewMessageListeners({@h.odometryCallback});
            h.ImuSub.setOnNewMessageListeners({@h.imuDataCallback});
            %Create Messages for the above Publishers:
            h.BodyWrench_msg = rosmatlab.message('geometry_msgs/WrenchStamped', h.Node);
            h.JointEffort_msg = rosmatlab.message('geometry_msgs/Vector3Stamped',h.Node);
            h.ModelState_msg = rosmatlab.message('gazebo_msgs/ModelState',h.Node);
            h.runsimulation_msg = rosmatlab.message('std_msgs/Duration',h.Node);
            h.resetworld_msg = rosmatlab.message('std_msgs/Empty',h.Node);
            h.clearwrench_msg = rosmatlab.message('std_msgs/String',h.Node);
            disp('Created Messages needed');
        end
        function delete(h)
            % Destructor.
            delete(h.Node);
        end
        function setEffort(h,joint_name, Effort, duration)
            %setEffort sets the specified Effort(Force/Torque in SI units) on the joint
            %starting immediately for specified duration. Later start time
            %will be added TODO
            header = h.JointEffort_msg.getHeader();
            header.setFrameId(joint_name);
            Vector = h.JointEffort_msg.getVector();
            Vector.setX(Effort);
            Vector.setY(duration);
            h.jointeffort_pub.publish(h.JointEffort_msg);
        end
        function setWrench(h, body_name, Wrench) 
            %Wrench is 6 by 1 vector of [force, torque]
            % By default it applies at the origin of the body and for
            % infinite duration until you explicitly clear it using
            % clearWrench
            header = h.BodyWrench_msg.getHeader();
            Force = h.BodyWrench_msg.getWrench().getForce();
            Torque = h.BodyWrench_msg.getWrench().getTorque();
            header.setFrameId(body_name);
            Force.setX(Wrench(1));
            Force.setY(Wrench(2));
            Force.setZ(Wrench(3));
            Torque.setX(Wrench(4));
            Torque.setX(Wrench(5));
            Torque.setX(Wrench(6));
            h.bodywrench_pub.publish(h.BodyWrench_msg);
        end
        function clearWrench(h, body_name)
            h.clearwrench_msg.setData(body_name);
            h.clearbodywrench_pub.publish(h.clearwrench_msg);
        end
        function runsimulation(h,duration)
            %Run the simulation for specified duration
            duration_data = h.runsimulation_msg.getData();
            duration_data.secs = floor(duration);%Set the duration of simulation
            duration_data.nsecs = round(1e9*(duration - floor(duration)));
            %h.runsimulation_msg.setData(duration);
            h.runsimulation_pub.publish(h.runsimulation_msg);
        end
        function resetworld(h)
            h.resetworld_pub.publish(h.resetworld_msg);
        end
        function Linkpose_ind = getLinkPose(h,index) %Can remove this function and move the LinkPoses into public props if needed
            %Index as listed in the Link names
            %Can add some assertions if needed TODO
            Linkpose_ind = h.LinkPoses{index};
        end
        function LinkTwist_ind = getLinkTwist(h, index)
            %Index as listed in the Link names
            %Can add some assertions if needed TODO
            LinkTwist_ind = h.LinkTwists{index};
        end
        function JointAngle_ind = getJointAngle(h, index)
            %Index as listed in Joint names
            JointAngle_ind = h.JointAngles{index};
        end
        function JointVelocity_ind = getJointVelocity(h, index)
            %Index as listed in Joint names
            JointVelocity_ind = h.JointVelocities{index};
        end
    end
    methods (Access = protected)
        %Add Callback functions here
        function linkstateCallback(h,message)
            %Link States are dealt with in this callback function
           % h.count = h.count + 1/100;
           % disp(h.count);       
            Names = message.getName();
            h.Available_Names = cell(Names.toArray);%List all the available names
            size_Linknames = size(h.Link_names);
            Poses = message.getPose();
            Twists = message.getTwist();
            found_newlinkdata = false;
            for i = 1:size_Linknames(1)
                %Then it is a member %Can make this faster by storing the indices in a slower loop if needed
                ind = Names.indexOf(h.Link_names{i});
                if ind >= 0 %Java numbering starts from 0
                    posei = Poses.get(ind);%Can open it up into vectors if needed TODO
                    posi = posei.getPosition();
                    orienti = posei.getOrientation();
                    h.LinkPoses{i} = [posi.getX(); posi.getY(); posi.getZ(); orienti.getW(); orienti.getX(); orienti.getY(); orienti.getZ()];
                    twisti = Twists.get(ind);
                    lineari = twisti.getLinear();
                    angulari = twisti.getAngular();
                    h.LinkTwists{i} = [lineari.getX(); lineari.getY(); lineari.getZ(); angulari.getX(); angulari.getY(); angulari.getZ()];
                    found_newlinkdata = true;
                end
            end
            %%%%%%%%%%%%Joint Finding %%%%%%%%%%%%%%%
            size_Jointnames = size(h.Joint_names);
            found_newjointdata = false;
            for i = 1:size_Jointnames(1)
                ind = Names.indexOf(h.Joint_names{i});
                if ind >= 0 %Found joint
                    twisti = Twists.get(ind);
                    lineari = twisti.getLinear();
                    angulari = twisti.getAngular();
                    h.JointAngles{i} = [angulari.getX(); angulari.getY(); angulari.getZ()];
                    h.JointVelocities{i} = [lineari.getX(); lineari.getY(); lineari.getZ()];
                    found_newjointdata = true;
                end
            end
            if found_newlinkdata %Notify only if we have new link data 
                notify(h,'NewLinkData'); %Broadcast event for other feedback functions to listen to it
            end
            if found_newjointdata
                notify(h,'NewJointData'); %Broadcast event for other feedback functions to listen to it
            end
        end
        function modelstateCallback(h,message)
            %Link States are dealt with in this callback function
            %h.count = h.count + 1/100;
            %disp(h.count);
            Names = message.getName();
            size_Modelnames = size(h.Model_names);
            Poses = message.getPose();
            Twists = message.getTwist();
            for i = 1:size_Modelnames(1)%Assuming row cell array
                %Then it is a member %Can make this faster by storing the indices in a slower loop if needed
                ind = Names.indexOf(h.Model_names{i});
                if ind >= 0  %Java numbering starts from 0
                    posei = Poses.get(ind);
                    posi = posei.getPosition();
                    orienti = posei.getOrientation();
                    h.ModelPoses{i} = [posi.getX(); posi.getY(); posi.getZ(); orienti.getW(); orienti.getX(); orienti.getY(); orienti.getZ()];
                    twisti = Twists.get(ind);
                    lineari = twisti.getLinear();
                    angulari = twisti.getAngular();
                    h.ModelTwists{i} = [lineari.getX(); lineari.getY(); lineari.getZ(); angulari.getX(); angulari.getY(); angulari.getZ()];
                end
            end
            notify(h,'NewModelData');
        end
        function imuDataCallback(h, message)%Copied from TurtlebotCommunicator
            % IMUDATACALLBACK - Execute tasks when new orientation data
            % (sensor_msgs/Imu) is received.
            
            orient = message.getOrientation();
            h.ImuOrientation = [orient.getW() orient.getX() orient.getY() orient.getZ()];
            notify(h,'NewImuData');
            % Convert to Euler angles.
            %angles = h.quat2angleZYX([orient.getW() orient.getX() orient.getY() orient.getZ()]);
        end
         function odometryCallback(h, message)%Copied from TurtlebotCommunicator
            % ODOMETRYCALLBACK - Execute tasks when new odometry data
            % (nav_msgs/Odometry) is received.
            
            pose = message.getPose();%PosewithCovariance
            pos = pose.getPose().getPosition();
            orient = pose.getPose().getOrientation();
            h.OdomPose = [pos.getX() pos.getY() pos.getZ() orient.getW() orient.getX() orient.getY() orient.getZ()];
            notify(h,'NewOdomData');
          %  Covar = pose.getCovariance();%Covariance can add this to the props if needed      
         end
         function laserCallback(h, message) %Copied from TurtlebotCommunicator
            % LASERCALLBACK - Execute tasks when a new laser scan
            % (sensor_msgs/LaserScan) is received.
            h.LaserData{1} = message.getRanges();
            h.LaserData{2} = (pi/2+message.getAngleMin():message.getAngleIncrement():pi/2+message.getAngleMax()-0.001)';
            notify(h,'NewLaserScanData');
            % Convert to Cartesian coordinates.
%             x = cos(angles).*ranges(:,1);
%             y = sin(angles).*ranges(:,1);
%           Plotting data            
%             if (size(x) == size(y))
%                 plot(h.LaserAxesHandle, x, y, 'LineWidth', 3); 
%                 xlim(h.LaserAxesHandle, [-1.5 1.5]);
%                 ylim(h.LaserAxesHandle, [0 3]);
%                 grid(h.LaserAxesHandle, 'on');
%                 title(h.LaserAxesHandle, 'Laser Scan');
%             end
        end
    end
end
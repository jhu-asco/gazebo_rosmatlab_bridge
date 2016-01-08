% PQP Testing using mex interface:
% Loads two mesh files and gives the distance between meshes
% The distance should be negative if one is inside other.
% TODO: Add visualization.

%Create PQP Interface
h = MatlabPqpManager;
%Load mesh and a point with tolerance = 0.1
%Modify this to your local path
meshpath = '/home/gowtham/indigo_workspace/src/gazebo_rosmatlab_bridge/models/modelcity/meshes/city_corrected_normals.stl';
h.Loadmesh(meshpath,'modelcity');
h.Loadmesh(0.4,'sphere');
pointstate = MatlabRigidBodyState;
%Create Gazebo Interface for visualization:
sim = GazeboMatlabSimulator;%Gazebo Matlab Interface:
markerinfo = MarkerInfo;%Current Trajectory
markerinfo.color = [0;0;1;1];%Blue
markerinfo.action = markerinfo.MODIFY;%Add initial state to trajectory
markerinfo.scale = 1.0;%Scaling for modelcity is 1 (Visual scaling)
%%
%profile on;
for i = 1:200
    pointstate.position = rand(3,1).*[150;150;20];%Randomly generate positions
    h.Setmeshstate('sphere',pointstate);%set point state
    %Compute distance:
    [distance,points]  = h.Computedistance('sphere','modelcity');
    %Turn off plotting for profiler
    %Plot the points: 
    markerinfo.id = i;%Unique id to mark the trajectory
    if distance > 0
        markerinfo.color = [0;0;1;0];%red
        sim.PublishTrajectory(points,markerinfo);
        %plot3(points(1,1),points(2,1),points(3,1),'b*');
    else
        markerinfo.color = [1;0;0;1];%Blue
        sim.PublishTrajectory(points,markerinfo);
        %plot3(points(1,2),points(2,2),points(3,2),'r*');
    end
    %plot3(points(1,:),points(2,:),points(3,:),'g');  
end
%profile off;
%profile viewer;

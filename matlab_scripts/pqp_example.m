% PQP Testing using mex interface:
% Loads two mesh files and gives the distance between meshes
% The distance should be negative if one is inside other.
% TODO: Add visualization.

%Create PQP Interface
h = MatlabPqpManager;
%Load mesh and a point with tolerance = 0.1
meshpath = '/home/gowtham/hydro_workspace/src/gazebo_rosmatlab_bridge/models/modelcity/meshes/city_corrected_normals.stl';
h.Loadmesh(meshpath,'modelcity');
h.Loadmesh(0.4,'sphere');
pointstate = MatlabRigidBodyState;
%%
%profile on;
for i = 1:200
    pointstate.position = rand(3,1).*[150;150;30];%Randomly generate positions
    h.Setmeshstate('sphere',pointstate);%set point state
    %Compute distance:
    [distance,points]  = h.Computedistance('sphere','modelcity');
    %Turn off plotting for profiler
    %Plot the points: 
    if distance > 0
        plot3(points(1,1),points(2,1),points(3,1),'b*');
    else
        plot3(points(1,1),points(2,1),points(3,1),'r*');
    end
    plot3(points(1,:),points(2,:),points(3,:),'g');  
    %plot3(points(1,2),points(2,2),points(3,2),'r*');
end
%profile off;
%profile viewer;
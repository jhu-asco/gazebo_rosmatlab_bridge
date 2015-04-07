function f = satellite_test( )
%Satellite_test Applies forces to thrusters to 
% verify if satellite is moving
%Thruster configuration:
%-ve x : 2,5
%-ve y: 1,8
%    z: 9,10
%+ve y: 3,6
%+ve x: 4,7
%1,2,3,4 top ring; 5,6,7,8, bottom ring
sim = GazeboMatlabSimulator;
sim.Reset;
sim.ActuatedLinks = 1:10;
us_links = cell(10,1);
for i = 1:10
    us_links{i} = MatlabLinkInput;
end
%Move in -x direction
% us_links{2}.force(3) = 1.0;%N
% us_links{5}.force(3) = 1.0;
%Move in -y direction
% us_links{1}.force(3) = 1.0;%N
% us_links{8}.force(3) = 1.0;
%Rotate around z axis:
for i =5:8
    us_links{i}.force(3) = 0.5;
end
[LinkData] = sim.Step(1,[],us_links);
disp('Satellite state:')
disp(LinkData{11}.getdata);
end


classdef MatlabRigidBodyState < handle
    %Pose Convenience class which stores position, orientation etc  
    properties
        position = zeros(3,1);%x,y,z
        orientation = [1;0;0;0];%w,x,y,z %Can also make it rpy
        linearvelocity = zeros(3,1);%vx,vy,vz
        angularvelocity = zeros(3,1);%wx,wy,wz
    end
    methods (Access = public)
        %Constructor
        function h  = MatlabRigidBodyState(input)
            if (nargin == 1)
                if size(input,1) ~= 13
                    input = input';
                end
                h.position = input(1:3);
                h.orientation = [input(4:7)];
                h.linearvelocity = input(8:10);
                h.angularvelocity = input(11:13);
            end
        end
        function output = getdata(h)
            output = [h.position; h.orientation; h.linearvelocity; h.angularvelocity];
        end
    end
end


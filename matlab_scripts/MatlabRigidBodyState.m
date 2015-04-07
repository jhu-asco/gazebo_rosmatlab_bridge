classdef MatlabRigidBodyState < handle
    %Convenience class which stores position, orientation etc of Rigid body state  
    properties
        position = zeros(3,1);%x,y,z
        orientation = [1;0;0;0];%w,x,y,z
        linearvelocity = zeros(3,1);%vx,vy,vz
        angularvelocity = zeros(3,1);%wx,wy,wz
    end
    methods (Access = public)
        %Constructor
        function h  = MatlabRigidBodyState(input)
            %Create a rigid body class from 13x1 vector of [position;
            %orientation; linearvelocity; angularvelocity]
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
            %Output as [13x1] data from existing data
            output = [h.position; h.orientation; h.linearvelocity; h.angularvelocity];
        end
    end
end


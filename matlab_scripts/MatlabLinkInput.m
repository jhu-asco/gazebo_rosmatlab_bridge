classdef MatlabLinkInput < handle
    %MatlabLinkInput Creates a struct for storing link data force and torque
    
    properties
        force = zeros(3,1); %Force applied to link
        torque = zeros(3,1); %Torque applied to link
        %reference_frame = ''; %reference frame wrt which the input is applied
    end
   
    methods
        function h = MatlabLinkInput(input)
            %Create a struct from a vector(force;torque)
            if (nargin == 1)
                if size(input,1) ~= 6
                    input = input';
                end
                h.force = input(1:3);
                h.torque = input(4:6);
            end
        end
        function output = getdata(h)
            %Output as 6x1 vector of [force; torque]
            %Ensure that the vectors force and torque are 3x1
            output = [h.force; h.torque];
        end
    end
    
end


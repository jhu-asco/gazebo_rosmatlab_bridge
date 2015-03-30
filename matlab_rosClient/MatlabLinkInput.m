classdef MatlabLinkInput < handle
    %MatlabLinkInput Creates a struct for storing link data force and
    %torque
    %   Detailed explanation goes here
    
    properties
        force = zeros(3,1); %Force applied to link
        torque = zeros(3,1); %Torque applied to link
        %reference_frame = ''; %reference frame wrt which the input is applied
    end
   
    methods
        function h = MatlabLinkInput(input)
            if size(input,1) ~= 6
                input = input';
            end
            if (nargin == 1)
                h.force = input(1:3);
                h.torque = input(4:6);
            end
        end
        function output = getdata(h)
            output = [h.force; h.torque];
        end
    end
    
end


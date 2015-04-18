classdef MarkerInfo <handle
    %MarkerInfo Provides information about color, id etc of the line being published
    properties (Constant)
        MODIFY=0;
        ADD=1;
        DELETE=2;
    end
    properties
        color = [1;0;0;0]%Default red (rgba)
        id = 0;%Default id = 0
        action=0;%Default Modify;
        scale = 0.01;%Some scaling issue needs to be adjusted based on what you see in gazebo
    end
end


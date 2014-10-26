classdef StateBuffer < handle
    properties (Access = public)
        Linkbuffer = [];%[13xn1xn2] n1 is the buffer size  n2 referers to number of links
        Jointbuffer = [];%[2xn1]  n2 is the buffer size 
        tbuffer =[]; %Time;
        updatepoint = 0;
        Plothandles = [];%Figure handles for refreshing data
        subsample_rate = 2;%Subsampling of the actual data freq
    end
    properties (Access = protected)
        buffersize ;%1 second of data
        visualization_inds = {}; %Visualization Indices constructed with
        run_index = 0; %Index for adding data into buffer in a circular manner
        subsample_count = 1;
    end
    methods (Access = public)
        function h = StateBuffer(viz_inds,buffersize)
            h.buffersize = buffersize;
            h.updatepoint = round(buffersize/2);
            h.run_index = h.updatepoint + 1;%Avoids a subtle problem in the first round
            h.visualization_inds = viz_inds;
            h.Linkbuffer = zeros(13,buffersize,length(viz_inds{1}));
            h.Jointbuffer = zeros(6,buffersize,length(viz_inds{2}));
            h.tbuffer = 1:buffersize;
%             figure; p1 = plot(0,0);
%             set(p1,'YDataSource','h.tbuffer');
%             set(p1,'XData',h.tbuffer);%Just 1:buffersize
%             h.Plothandles(end+1) = p1; %#DEBUG For Tbuffer
            for j = 1:length(viz_inds{1})
                figure, clf;
                for i = 1:6
                    subplot(2,3,i), p1 =  plot(0,0);
                    set(p1,'XDataSource','h.tbuffer');
                    if i > 3
                        set(p1,'YDataSource',strcat('h.Linkbuffer(',num2str(i+3),',:,',num2str(j),')'));
                    else
                        set(p1,'YDataSource',strcat('h.Linkbuffer(',num2str(i),',:,',num2str(j),')'));
                    end
                    h.Plothandles(end+1) = p1;%Store plot handles for calling refresh later
                end
            end
            for j = 1:length(viz_inds{2})
                figure, clf;
                for i = 1:2
                    subplot(1,2,i), p1 =  plot(0,0);
                    set(p1,'XDataSource','h.tbuffer');
                    if i > 1
                        set(p1,'YDataSource',strcat('h.Jointbuffer(',num2str(i+2),',:,',num2str(j),')'));
                    else
                        set(p1,'YDataSource',strcat('h.Jointbuffer(',num2str(i),',:,',num2str(j),')'));
                    end
                    h.Plothandles(end+1) = p1;%Store plot handles for calling refresh later
                end
            end
            %refreshdata(h.Plothandles,'caller');
        end
        function add_data(h,Gazebo_class)%Should be called at Data collection frequency
            % Add Subsampling 
            if h.subsample_count < h.subsample_rate
                h.subsample_count = h.subsample_count+1;
                return;
            else
                h.subsample_count  = 1;
            end
            if ~isempty(h.visualization_inds{1})
                h.Linkbuffer(:,h.run_index+1,:) = Gazebo_class.LinkData(:,h.visualization_inds{1});
            end
            if ~isempty(h.visualization_inds{2})
                h.Jointbuffer(:,h.run_index+1,:) = Gazebo_class.JointData(:,h.visualization_inds{2});
            end% This can be modified for collecting data in open loop case
            h.tbuffer(h.run_index+1) = Gazebo_class.t;
            
            if h.run_index == h.updatepoint%Every half buffer refresh data
                h.tbuffer = circshift(h.tbuffer,-h.run_index-1,2);
                if ~isempty(h.visualization_inds{1})
                    h.Linkbuffer = circshift(h.Linkbuffer,-h.run_index-1,2);
                end
                if ~isempty(h.visualization_inds{2})
                    h.Jointbuffer = circshift(h.Jointbuffer,-h.run_index-1,2);
                end
                h.run_index = -1;%After Shifting
                %Visualize the data:
                refreshdata(h.Plothandles,'caller');%Refresh Data
            end
            
            h.run_index = rem(h.run_index+1,h.buffersize);%Can also subsample data (Later) #TODO
        end
    end
end

function Response_to_link_data(source)
%Response_to_link_data(h) : Sample response function to link data
%Can add more parameters using the normal method:
%@(h,params)Response_to_link_data(h,params)
%Assumes there is atleast one Link name available
%disp(h.getLinkPose(1));
global count;
count = count + 1;
Jointangle  = source.getJointAngle(1);
Jointvel = source.getJointVelocity(1);
figure(1), subplot(2,1,1), hold on, plot(count, Jointangle(1),'b*');
figure(1), subplot(2,1,2), hold on, plot(count, Jointvel(1),'r*');
end


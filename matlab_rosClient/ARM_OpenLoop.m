function [linkids, bodywrenches, jointids, jointefforts] = ARM_OpenLoop(t)
%Assumes t is 1xn vector
linkids = [];
bodywrenches = [];%No Link and body wrenches
jointids = [1 2];
jointefforts = [0.5; 1] * sin(t);%[2x1][1xn]
%jointefforts_reshape = reshape(jointefforts,1,2,length(t));%Reshape into 1x2xN
end
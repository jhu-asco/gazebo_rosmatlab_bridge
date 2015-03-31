function f = sopt_car

S.n =4;
S.c=2;


S.L = @car_L;
S.Lf = @car_Lf;
S.xf = [1; 2; pi/2; 0];%Posn(x,y),Angle, Body Velocity

S.Q = 0.1*diag([5, 5, 1, 1]);
S.R = .01*diag([1, 1]);
S.Qf = diag([5, 5, 1, 1]);
S.xf = [1; 2; pi/2; 0];%Posn(x,y),Angle, Body Velocity



pxs(1).m = [0; 0; 0; 1];
pxs(1).S = diag([.0001; .0001; 0.0001; 0.0001]);

%pu.m = [.5; .5];
N = 10;
tf = 3;
S.dt = tf/N;

S.sim = GazeboMatlabSimulator;
S.sim.Configure(0.001,20);
S.jointids = 1:4;
for i=1:N
  pus(i).m = [0.5; 0.1];
  pus(i).S = diag([0.1; 0.2]);
  pus(i).K=[];
  pus(i).Sux=[];
end

for it=1:1000
  [pxs, pus] = sopt7(pxs, pus, S);
%  pu.m
%  pu.S
%  K
input('Press Enter to continue')
%pause(1)
end


function L = car_L(x, u, S)
% car cost (just standard quadratic cost)
L = S.dt/2*(x'*S.Q*x + u'*S.R*u);

function L = car_Lf(x, S)
L = (x-S.xf)'*S.Qf*(x-S.xf)/2;


% function [xs, us] = traj(x0, pu, K, S)
% 
% N = S.T/S.dt;
% n = length(x0);
% xs = zeros(n, N+1);
% us = zeros(2,N);
% mex_mmap('stringreq',S.sim.MexData,'worldreset');
% pause(0.01);
% xs(:,1) = x0;
% nofsteps = S.dt/S.physxtimestep;
% %Try setting car to x0 here
% for i=1:N
%   us(:,i) = mvnrnd(pu.m, pu.S)' + K*(xs(:,i) - px.m);
%   %Constant interpolation and repmat:
%   us1 = repmat(us(:,i),2,1);%For both left and right wheels
%   [LinkData,~] = mex_mmap('runsimulation',S.sim.MexData, uint32(jointids)-1, us1, ...
%                                                 [], [], uint32([0,nofsteps]));
%   xs(1:2,i+1) = LinkData(1:2,2);
%   xs(3,i+1) = atan2(2*(LinkData(4,2).*LinkData(7,2) + LinkData(5,2).*LinkData(6,2)), 1 - ...
%                     2*(LinkData(6,2).^2 + LinkData(7,2).^2));%Yaw computation
%   xs(4,i+1) = LinkData(8,2);
% end

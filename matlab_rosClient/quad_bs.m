function f = quad_bs()
% Demonstration of nonlinear trajectory tracking control of underactuated
% systems such as a quadrotor. 
%
% See paper M. Kobilarov, "Trajectory tracking of a class of underactuated
% systems with external disturbances", 2013
%
% Author: Marin Kobilarov, marin(at)jhu.edu, 2013


clear 

m = 1.316;
J=[0.0128;0.0128;0.0128];
fg = [0; 0; -9.81*m];
e = [0; 0; 1];
dd = [.3; .3; .005]

% system parameters
S.m = m;
S.ag = [0;0;-9.81];
S.J = J;
S.e = [0;0;1];

% desired state
sd = traj([], 0);
sd.R = [];

% initial state
s.p = [0;0;0.182466];
rpy = [0;0;0];%Initial roll pitch yaw
initialquat = rpy2quat(rpy);
s.R = quat2mat(initialquat);
s.w = zeros(3,1);
s.v = zeros(3,1);
s.u = -S.ag'*S.e*S.m;
s.du = 0;

% timing
h = .01;
tf = 1000*h;

pds = [];
ps = [];
Rs = [];
vds = [];
vs = [];
ws = [];
us = [];
uns = [];
LFs = [];
Ts = [];
ts = [];

%Gazebo Matlab Interface:
sim = GazeboMatlabSimulator;
sim.ActuatedLinks = [1];
initialmodelstate = MatlabRigidBodyState;
initialmodelstate.position = s.p;
initialmodelstate.orientation = initialquat;
%Initial velocity etc are 0.
sim.SetModelState('quadrotor',initialmodelstate);
sim.Configure(0.001,1);
%Helper variable: 
bodywrench{1} = MatlabLinkInput;


% graphics
gr = 1;
G=[];
if gr
    markerinfo1 = MarkerInfo;%Current Trajectory
    markerinfo1.color = [0;0;1;1];%Blue
    markerinfo1.id = 1;%Unique id to mark the trajectory
    markerinfo2 = MarkerInfo;%Desired Trajectory
    
    markerinfo1.action = markerinfo1.MODIFY;
    sim.PublishTrajectory(initialmodelstate.position,markerinfo1);
    markerinfo1.action = markerinfo1.ADD;
    markerinfo2.action = markerinfo2.MODIFY;
    sim.PublishTrajectory(sd.p,markerinfo2);
     markerinfo2.action = markerinfo2.ADD;
end

% position dynamics matrices
S.A = [zeros(3), eye(3);
       zeros(3, 6)];

S.B = [zeros(3);
       eye(3)/m];


% gains
S.kp = 1; S.kv = 1;
S.k1 = 1; S.k2 = 1;

S.K = [diag([S.kp S.kp S.kp]), diag([S.kv S.kv S.kv])];
S.Q = eye(6);
S.P = lyap((S.A - S.B*S.K)', S.Q);


for t=h:h:tf,

  % get desired state sd at time t
  sd = traj(sd, t);
  sd.Rref=eye(3);
    
  % computing tracking controls
  [s, sd, T, ddu, LF] = track(s, sd, h, S);

  % update current state s.u based on s.ddu double integrating ddu to get
  % du
  s = update(s, h, ddu);
  %Set the link input to gazebo:
  bodywrench{1}.force = [0;0;s.u];
  bodywrench{1}.torque = T;
  [LinkState,~] = sim.Step(h,[],bodywrench);
  s.p = LinkState{1}.position;
  s.R = quat2mat(LinkState{1}.orientation);
  s.v = LinkState{1}.linearvelocity;
  s.w = LinkState{1}.angularvelocity;
  % record states, etc...
  if (t > h)
    Ts = [Ts, T];
    us = [us, s.u];
    
    ws = [ws, s.w];

    vs = [vs, s.v];
    vds = [vds, sd.dp];

    pds = [pds, sd.p];
    ps = [ps, s.p];

    ts = [ts, t];
    LFs = [LFs, LF];    
  end
  
  % graphics
  if gr 
      if rem(t,5*h) == 0
          sim.PublishTrajectory(LinkState{1}.position,markerinfo1);
          sim.PublishTrajectory(sd.p,markerinfo2);
      end
  end
  
end


lw = 3;
fs = 15;

plot3(ps(1,:), ps(2,:), ps(3,:), 'b-','LineWidth',3)
hold on
plot3(pds(1,1:5:end), pds(2,1:5:end), pds(3,1:5:end), 'r--','LineWidth',3)
set(gca, 'FontSize',fs)
xlabel('m')
ylabel('m')
zlabel('m')
axis equal

h = legend('$\mathbf x$','$\mathbf x_d$')

set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


figure 

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,1)
plot(ts, vs(1,:), 'b','LineWidth',lw)
hold on
plot(ts, vds(1,:), 'r--','LineWidth',lw);
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('m/s')
h = legend('$\dot x$','$\dot x_d$')
set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,2)
plot(ts, vs(2,:), 'b','LineWidth',lw)
hold on
plot(ts, vds(2,:), 'r--','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('$\dot y$','$\dot y_d$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,3)
plot(ts, vs(3,:), 'b','LineWidth',lw)
hold on
plot(ts, vds(3,:), 'r--','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('m/s')
h = legend('$\dot z$','$\dot z_d$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,4)
plot(ts, ws(1,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('rad/s')
h = legend('${\omega}_x$')
set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,5)
plot(ts, ws(2,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('rad/s')
h = legend('${\omega}_y$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,6)
plot(ts, ws(3,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('rad/s')
h = legend('${\omega}_z$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,7)
plot(ts, Ts(1,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('Nm')
h = legend('${\mathbf\tau}_x$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,8)
plot(ts, Ts(2,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('${\mathbf\tau}_y$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,9)
plot(ts, Ts(3,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('${\mathbf\tau}_z$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,10)
plot(ts, us, '-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('N')
h = legend('$u_{commanded}$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,11)
if ~isempty(uns)
  plot(ts, uns, '-','LineWidth',lw);
  set(gca, 'FontSize',fs)
  xlabel('sec.')
  ylabel('N')
  h = legend('$u_{actual}$')
  set(h,'Interpreter','latex')
  
  set(gca, 'Position', get(gca, 'OuterPosition') - ...
           get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
end

subplot(5,3,12)
plot(ts, LFs, '-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('N')
h = legend('$V$')
set(h,'Interpreter','latex')



set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);



function [s, sd, T, ddu, LF] = track(s, sd, h, S)
% compute T and ddu to stabilize to a desired state sd from current state s

f = S.m*S.ag;
g = s.R*(S.e*s.u);

x = [s.p; s.v];
dx = S.A*x + S.B*(f + g);

xd = [sd.p; sd.dp];
dxd = [sd.dp; sd.d2p];
d2xd = [sd.d2p; sd.d3p];

z0 = x - xd;
dz0 = dx - dxd;

gd = S.m*sd.d2p - S.K*z0 - f;
z1 = g - gd;

dgd = S.m*sd.d3p - S.K*dz0;

ad = dgd - S.B'*S.P*z0 - S.k1*z1;

sd.u = norm(gd);
sd.du = ad'*gd/sd.u;
if sd.u < eps
  disp('thrust close to zero!')
  pause 0
end

wh = so3_ad(s.w);

dg = s.R*(wh*S.e*s.u + S.e*s.du);

dz1 = dg - dgd;

d2x = S.A*dx + S.B*dg;

d2gd = S.m*sd.d4p - S.K*(d2x - d2xd);

dad = d2gd - S.B'*S.P*dz0 - S.k1*dz1;

z2 = dg - ad;

bd = dad - z1 - S.k2*z2;

bz = gd/sd.u;
by = unit(cross(bz, [1;0;0]));
Rd = [unit(cross(by,bz)), by, bz];

if isempty(sd.R)
  sd.R = Rd;
end

c = s.R'*bd - wh*(wh*S.e*s.u + 2*S.e*s.du);

ddu = S.e'*c;
T = S.J.*cross(S.e, c/s.u) - cross(S.J.*s.w, s.w);

LF = (z0'*S.P*z0 + z1'*z1 + z2'*z2)/2;



function sd = traj(sd, t)
% desired trajectory

w = 1;
r = 1;
vz = .5;
sd.p = [r*cos(w*t); r*sin(w*t); t*vz];
sd.dp = [-r*w*sin(w*t); r*w*cos(w*t); ones(size(t))*vz];
sd.d2p = [-r*w^2*cos(w*t); -r*w^2*sin(w*t); zeros(size(t))];
sd.d3p = [r*w^3*sin(w*t); -r*w^3*cos(w*t); zeros(size(t))];
sd.d4p = [r*w^4*cos(w*t); r*w^4*sin(w*t); zeros(size(t))];


function s = update(s, h, ddu)
% update s.u using s.du using an Euler-step

s.du = s.du + h*ddu;
s.u = s.u + h*s.du;

% dw = (cross(S.J.*s.w, s.w) + T)./S.J;
% dv = S.ag + s.R*S.e*u/S.m;

% s.w = s.w + h*dw;
% s.v = s.v + h*dv;

% s.R = s.R*so3_exp(h*s.w);
% s.p = s.p + h*s.v;




function f = so3_exp(w, varargin)

theta = sqrt(w'*w);

if ~isnumeric(w(1)) && theta == sym('0')
  f = sym(eye(3));
  return
end

if (isnumeric(w(1)) && theta < eps)
  f = eye(3);
  return
end


%if (theta < eps)
%  f = eye(3);
%  return
%end



w_ = w/theta;
wh = so3_ad(w_);

f = eye(3) + wh*sin(theta) + wh*wh*(1-cos(theta));

if (~isnumeric(w(1)))
  for i=1:length(w),
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
  end
end


function f = so3_ad(w)
f=[0, -w(3), w(2);
   w(3), 0, -w(1);
   -w(2), w(1), 0];

function out = unit(in)
if norm(in) > 0.01
    out = in / norm(in);
else
    out = in;
end

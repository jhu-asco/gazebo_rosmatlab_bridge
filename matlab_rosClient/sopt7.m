function [pxs, pus] = sopt7(pxs, pus, S)

N = length(pus);
n = length(pxs(1).m);
c = length(pus(1).m);

M = 50;

S.M=M;
S.n = n;

xss = zeros(n, N+1, M); % aligned
uss = zeros(c, N, M);  
ucss = zeros(c, N, M);


Js = zeros(1,M);

for j=1:M,
  x0 = mvnrnd(pxs(1).m, pxs(1).S);
  [xs, us, ucs] = sample_traj(pxs, pus, S);
  xss(:,:,j) = xs;
  uss(:,:,j) = us;
  ucss(:,:,j) = ucs;
  
  for i=1:N,
    Js(j) = Js(j) +  S.L(xss(:,i,j), ucss(:,i,j), S);      
  end
  Js(j) = Js(j) +  S.Lf(xss(:,N+1,j), S);
  
  plot(xs(1,:),xs(2,:),'-r');
  hold on
end

for i=2:N,
  xis = reshape(xss(:,i,:), n, M);
  pxs(i).m = mean(xis')';
  pxs(i).S = cov(xis');
  plotcov2(pxs(i).m(1:2), pxs(i).S(1:2,1:2), 'plot-opts',{'g'}); 
end

Js'
  
v=1;

for i=N:-1:1
  
  cs = Js;
  
  [cmin, imin] = min(cs);
  b = max(1/min(cs), 1e-3);
  %  b = 1/mean(cs);
  ws = exp(-b*cs);
  ws = ws/sum(ws);
  %disp('ws'), ws'
  
  xs = reshape(xss(:,i,:), n, M);
  us = reshape(uss(:,i,:), c, M);
  ucs = reshape(ucss(:,i,:), c, M);
    
  Sc = wcov([ucs; xs]', ws);
    
  Su = wcov(us', ws);
  
  disp(['det(Sc)=' num2str(det(Sc))]);
  pus(i).m = (1 - v).*pus(i).m + v.*(us*ws');
  
  %  pus(i).S = (1 - v).*pus(i).S + v.*wcov(dus', ws);% + 1e-6*eye(c);    
  pus(i).S = (1 - v).*pus(i).S + v.*Su;
  
  pus(i).K = Sc(1:c, c+1:end)*inv(Sc(c+1:end, c+1:end));
%  pus(i).K = zeros(c,n);
  %  pus(i).K = Sc(1:c, c+1:end)*inv(px.S);
  
  %  pus(i).K = -.3*eye(2);
  
  pus(i).Sux = Sc(1:c, c+1:end);
  
  K = pus(i).K;
  K
end

hold off


function [xs, us, ucs] = sample_traj(pxs, pus, S)

n = length(pxs(1).m);
m = length(pus(1).m);
N = length(pus);

xs = zeros(n, N+1);
us = zeros(m, N);
ucs = zeros(m, N);
xs(:,1) = mvnrnd(pxs(1).m, pxs(1).S);
%xs(:,1) = pxs(1).m;
%Try setting xs(:,1) here thru gazebo
mex_mmap('stringreq',S.sim.Mex_data,'worldreset');
pause(0.01);
%Set car to initial posn:
modelposeandtwist = [xs(1:2,1)' 0.05 rpy2quat([xs(3,1), 0, 0])' xs(4,1) zeros(1,5)];%13x1
mex_mmap('setmodelstate',S.sim.Mex_data,'Unicycle',modelposeandtwist);
pause(0.01);
nofsteps = S.dt/S.sim.physxtimestep;
for i=1:N
  us(:,i) = mvnrnd(pus(i).m, pus(i).S);
  if ~isempty(pus(i).K)
    ucs(:,i) = us(:,i) + pus(i).K*(xs(:,i) - pxs(i).m);
  else
    ucs(:,i) = us(:,i);
  end
  us1 = repmat(ucs(:,i),2,1);%For both left and right wheels

  if i == 1
      [LinkData,JointData] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(S.jointids)-1, us1, ...
          [], [], uint32([0,nofsteps]));
      x0(1:2,1) = LinkData(1:2,1);
      x0(3,1) = atan2(2*(LinkData(4,1).*LinkData(7,1) + LinkData(5,1).*LinkData(6,1)), 1 - ...
          2*(LinkData(6,1).^2 + LinkData(7,1).^2));%Yaw computation
      x0(4,1) = LinkData(8,1);
      %Verify Initial Posn:
      if norm(xs(:,1) - x0)>1e-3
          disp('xs is not starting from x0');
      end
      if norm(JointData([1,4],[2,4])) > 1e-3
          disp('Steering Joints not straight');
      end
  else
      [LinkData,~] = mex_mmap('runsimulation',S.sim.Mex_data, uint32(S.jointids)-1, us1, ...
          [], [], uint32([0,nofsteps]));
  end
  xs(1:2,i+1) = LinkData(1:2,2);
  xs(3,i+1) = atan2(2*(LinkData(4,2).*LinkData(7,2) + LinkData(5,2).*LinkData(6,2)), 1 - ...
      2*(LinkData(6,2).^2 + LinkData(7,2).^2));%Yaw computation
  xs(4,i+1) = LinkData(8,2);
end

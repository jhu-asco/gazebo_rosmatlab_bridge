function [V, Vn, dV, xs, us, Ds] = sddp(x0, uds, xds, Ds, S)
% Second-order numerical optimal control. (Sampling version of DDP)
% The code computes the optimal control adjustment for a given dynamical system
%
% params:
% x0 - initial state
% uds - m-N matrix with discrete controls
% xds - Trajectory produced by us (If not provided will be generated)
% cs - Gains for controller
% S - problem data:
%     S.L  : handle to the cost function
%     S.f  : handle to the discrete dynamics functions
%     S.mu : regularizing constant (default is 0)
%     S.a  : initial step size (default is 1)
%     S.diff : difference function (default is minus, i.e. vector space)
%
% return:
%   V: current value function
%   Vn: new value function
%   dV: predicted change in value function
%   xs: Optimal Trajectory after one iteration
%   us: Optimal Controls after one iteration
%   cs: Control Intercepts computed from backward pass (Useful to create a controller)
%   Ds: Control gains computed from backward pass (Useful to create a controller)
%
%
% Note: this implementation is most closely related to second-order 
% methods known as stage-wise Newton (SN) - Bertsekas, 2003 
% and differential-dynamic-programming (DDP), Mayne, 1966. 
% In this implementation second-order terms in the dynamics 
% are ignored which corresponds to the linear-quadratic-subproblem
% (LQS) approach (see also iterative-LQR (Todorov et al)).
%
% Disclaimer: the code is for education purposes only
%
% Author: Marin Kobilarov marin(at)jhu.edu

%if ~isfield(S, 'diff')
%  S.diff = @diff_def;
%end

if ~isfield(S, 'mu')
  S.mu = 0;
end

if ~isfield(S, 'mu0')
  S.mu0 = 1e-3;
end

if ~isfield(S, 'dmu0')
  S.dmu0 = 2;
end

if ~isfield(S, 'mumax')
  S.mumax = 1e6;
end

if ~isfield(S, 'a')
  S.a = 1;
end

if ~isfield(S, 'amin')
  S.amin = 1e-32;
end

if ~isfield(S, 'n')
  S.n = length(x0);
end

if ~isfield(S, 'info')
  S.info = 0;
end


n = S.n;
m = size(uds, 1);

N = size(uds, 2);

cs = zeros(m, N);
Ps = zeros(n,n,N+1);
vs = zeros(n,N+1);

% integrate trajectory and get terminal cost if not provided already
if isempty(xds)
  xds = S.f(x0, uds, [],[],[],S);%Openloop integration of controls
end
%%%%%%%%%%%Linearize(to get A,B)%%%%%%%%%%%%%%%%%
[As, Bs] = EvaluateJacobians(@(x0,dus)S.f(x0,dus,xds,cs,Ds,S), x0, xds, uds);
% [As, Bs] = EvaluateJacobians(@(x0,dus)S.f(x0,dus,xds,cs,zeros(m,n,N),S), x0, xds, uds);


%%%%%%%%%%%Backward Pass%%%%%%%%%%%%%%%%%
[L, Lx, Lxx, ~, ~] = S.L(N+1, xds(:,end), [], S);

% initialize
V = L;
v = Lx;
P = Lxx;

dV = [0; 0];

Ps(:,:,N+1) = P;
vs(:,N+1) = v;

for k=N:-1:1,
  
  x = xds(:,k);
  u = uds(:,k);
  A = As((k-1)*n + (1:n),:);%n x n
  B = Bs((k-1)*n + (1:n),:);%n x m
  
  [L, Lx, Lxx, Lu, Luu] = S.L(k, x, u, S);
  
  V = V + L;
  
  Qx = Lx + A'*v;
  Qu = Lu + B'*v;
  Qxx = Lxx + A'*P*A;
  Quu = Luu + B'*P*B;
  Qux = B'*P*A;
  
  mu = S.mu;
  dmu = 1;
  
  while 1
    Quum = Quu + mu*eye(m);
    
    [F, d] = chol(Quum);
    if d == 0
      % this is the standard quadratic rule specified by Tassa and Todorov
      dmu = min(1/S.dmu0, dmu/S.dmu0);
      if (mu*dmu > S.mu0)
        mu = mu*dmu;
      else
        mu = S.mu0;
      end
      
      if S.info
        disp(['[I] Ddp::Backward: reduced mu=' num2str(mu) ' at k=' num2str(k)]);          
      end
      break;      
    end

    dmu = max(S.dmu0, dmu*S.dmu0);
    mu = max(S.mu0, mu*dmu);
        
    if S.info
      disp(['[I] Ddp::Backward: increased mu=' num2str(mu) ' at k=' num2str(k)]);
    end
    
    if (mu > S.mumax)
      disp(['[W] Ddp::Backward: mu= ' num2str(mu) 'exceeded maximum ']);
      break;
    end
    
  end
  
  if (mu > S.mumax)
    break;
  end
  
  % control law is du = c + D*dx
  cD = -F\(F'\[Qu, Qux]);
  c = cD(:, 1);
  D = cD(:, 2:end);
  
  v = Qx + D'*Qu;
  P = Qxx + D'*Qux;
  
  dV = dV + [c'*Qu; c'*Quu*c/2];
  
  vs(:, k) = v;
  Ps(:, :, k) = P;

  cs(:, k) = c; 
  Ds(:, :, k) = D; 

end


%%%%%%%%%%%Forward Pass%%%%%%%%%%%%%%%%%

% s1 = .1;
% s2 = .5;
b1 = .25;
% b2 = 2;

a = S.a;

% measured change in V
dVm = eps;

while dVm > 0

  % new measured cost
  Vn = 0;

  % Evaluate Trajectory with gains computed from Backward Pass and current
  % step size
  [xs,us] = S.f(x0, uds, xds,a*cs,Ds,S);
  
  %Evaluate the Cost of the Trajectory
  for k=1:N,
    
    Ln = S.L(k, xs(:,k), us(:,k), S);
    
    Vn = Vn + Ln;
  end
  
  L = S.L(N+1, xs(:,end), [], S);
  Vn = Vn + L;
  
  %Evaluate the difference in cost
  dVm = Vn - V;
  
  if a == 0
      break;
  end
  
  if dVm > 0
    a = b1*a;
    if S.info
      disp(['[I] Ddp: decreasing a=' num2str(a)])
    end
    
    if a < S.amin
      a = 0;
    end

    continue    
  end
      
  %dVp = [a; a*a]'*dV;
  
  %r = dVm/dVp;
  
  %if r < s1
  %  a = b1*a;
  %else
  %  if r >= s2 
  %    a = b2*a;
  %  end
  %end
  %if S.info
  %  disp(['[I] ddp: decreasing a=' num2str(a)])
  %end
  
end


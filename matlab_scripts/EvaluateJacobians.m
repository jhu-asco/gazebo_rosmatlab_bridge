function [ As, Bs ] = EvaluateJacobians(sys_traj, x0, xs_n, us, Ns, du_scale, dx_scale )
%EVALUATECOSTANDJACOBIANS: Evaluates the cost and jacobians by perturbing the controls and fitting a linear model to the dynamics
%   Finds the cost and Jacobians for a given trajectory
%   x0: nx1 state vector; us: 2xN control instances; S: system parameters
%   Ns: number of samples optional sys_traj: Function to generate xs from
%   x0 and us for a given system
%   xs_n is the nominal trajectory generated using us. If not provided will
%   be computed
%   Output: As (nx*N, nx) stack of state jacobians for the trajectory
%   Bs (nx*N, nu) stack of control jacobians for the trajectory
rng(1020102);%Set seed for generating same random behaviour everytime
%Convenience Variables:
N = size(us,2);
nx = size(x0,1);
nu = size(us,1);

%Check arguments:
if nargin <= 4
    Ns = 2*(nx+nu);
end
if nargin <= 5
%     du_scale = 0.001;
    du_scale = 0.000001;
end
if nargin <= 6
%     dx_scale = 0.001;
    dx_scale = 0.000001;
end

%Steps:
%Set Matrices for accumulating the perturbations
dusmatrix = du_scale*randn(nu*N,Ns);
dxsmatrix = zeros(nx*(N+1),Ns);
dxsmatrix(1:nx, :) = dx_scale*randn(nx, Ns);%Set perturbations to starting state

%Generate Nominal Trajectory:
if isempty(xs_n)
    xs_n = sys_traj(x0, us);
end
%For Ns samples
for i = 1:Ns
    %Perturb initial state
    x0_s = x0 + dxsmatrix(1:nx, i);
    %Perturbed controls:
    if ~isempty(us)
      us_s = us + reshape(dusmatrix(:,i),nu,N);
    else
      us_s = reshape(dusmatrix(:,i),nu,N);
    end
    % 2. Generate Perturbed States
    [xs_s,us_f] = sys_traj(x0_s, us_s);
    dusmatrix(:,i) = reshape(us_f - us, nu*N,1);
    % 3. Store deviations
    dxsmatrix(:,i) = reshape(xs_s - xs_n,nx*(N+1),1);
end
%Set Outputs:
ABs = zeros(nx+nu, nx*N);
for i = 1:N
    XU = [dxsmatrix((i-1)*nx + (1:nx),:);dusmatrix((i-1)*nu + (1:nu),:)]';
    ABs(:,(i-1)*nx + (1:nx)) = XU \ dxsmatrix(i*nx + (1:nx), :)';
end
As = ABs(1:nx, :)';
Bs = ABs(nx+1:nx+nu, :)';
%For Gazebo we cannot perturb velocities for initial state (for joints).
%Hence We copy the jacobian of the first state from the state next to it.
As(1:nx,:) = As(nx+(1:nx),:);
%end
end

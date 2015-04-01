function G = draw_rb(G, x, R, c, d)

x
R
c
d
G
% draws a 3-d box with given dimension
% @param G graphics handle useful for animation purposes (initially pass [])
% @param x position (3x1 vector)
% @param R orientation (3x3 matrix)
% @param c color (character, e.g. 'r', 'b', 'g', etc...)
% @param d box dimensions (3x1 vector)
% Note: pass [] to any of these parameters to use the default
%
% @return G the populated graphics object. 
% Note: if you want to do a simulation you can call this function
% continuously and just pass G every time so that it updates the
% current position and orientation of the box


l = .2;
b=.1;

if (isempty(x))
  x = zeros(3,1);
end

if (isempty(R))
  R = eye(3);
end

if (isempty(c))
  c = 'b';
end

if (isempty(d))
  d = [1;1;1];
end


if (isempty(G))
  %if (1)
  
  G.hs = [];
  
  
  %  [Xs,Ys,Zs]=cylinder2P([S.r S.r], 5, [0; 0; -S.r]',[0; 0; S.r]' );
  
  %  [Xs,Ys,Zs]=ellipsoid(0,0,0,b,b,b);
  
  %  G.Xs{1}=Xs;
  %  G.Ys{1}=Ys;
  %  G.Zs{1}=Zs;
  
  %  x=[-.5 .5 .5 -.5 -.5 -.5;.5 .5 -.5 -.5 .5 .5;.5 .5 -.5 -.5 .5 .5;-.5 .5 .5 -.5 -.5 -.5]*s;
  %  y=[-.5 -.5 .5 .5 -.5 -.5;-.5 .5 .5 -.5 -.5 -.5;-.5 .5 .5 -.5 .5 .5;-.5 -.5 .5 .5 .5 .5]*s;
  %  z=[-.5 -.5 -.5 -.5 -.5 .5;-.5 -.5 -.5 -.5 -.5 .5;.5 .5 .5 .5 -.5 .5;.5 .5 .5 .5 -.5 .5]*s;
  
  x=[-.5 .5 .5 -.5 -.5 ;.5 .5 -.5 -.5 .5 ;.5 .5 -.5 -.5 .5 ;-.5 .5 .5 -.5 -.5 ]*d(1);
  y=[-.5 -.5 .5 .5 -.5 ;-.5 .5 .5 -.5 -.5 ;-.5 .5 .5 -.5 .5 ;-.5 -.5 .5 .5 .5 ]*d(2);
  z=[-.5 -.5 -.5 -.5 -.5 ;-.5 -.5 -.5 -.5 -.5 ;.5 .5 .5 .5 -.5 ;.5 .5 .5 .5 -.5 ]*d(3);

  G.hs(1) = patch(x,y,z,'k');
  set(G.hs(1),'edgecolor','w')

  G.Xs{1} = get(G.hs(1),'XData');
  G.Ys{1} = get(G.hs(1),'YData');
  G.Zs{1} = get(G.hs(1),'ZData');
    
  set(G.hs(1),'EdgeColor','none', ...
              'FaceColor',c, ...
              'FaceLighting','phong', ...
              'AmbientStrength',0.3, ...
              'DiffuseStrength',0.8, ...
              'SpecularStrength',0.9, ...
              'SpecularExponent',25, ...
              'BackFaceLighting','lit');
  
  camlight left;
  hidden off  
end


e = so3_log(R);
for i=1:length(G.hs),
  set(G.hs(i),'XData', G.Xs{i} + x(1));
  set(G.hs(i),'YData', G.Ys{i} + x(2));
  set(G.hs(i),'ZData', G.Zs{i} + x(3));
    
  if (norm(e) > .0001)
    rotate(G.hs(i), e/norm(e), norm(e)*180/pi, x);
  end
end

%set(h,'Visible','off')
%axis square
%axis equal

function f = so3_log(R, varargin)

if (nargin>1)
  if (norm(R-eye(3),'fro') < 2*eps)
    f = zeros(3);
    return
  end
end

phi = acos(1/2*(trace(R)-1));

if (nargin>1)
  if (abs(phi) < 1e-10)
    f = zeros(3);
    return
  end
end

f = so3_hatinv(phi/(2*sin(phi))*(R-R'));

function v = so3_hatinv(xi)
v = [xi(3,2); xi(1,3); xi(2,1)];
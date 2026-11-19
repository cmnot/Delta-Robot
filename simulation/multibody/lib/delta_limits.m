function [beta, gam] = delta_limits(X, q, G)
%DELTA_LIMITS  Quantities for the workspace limits (same definition as Claude_helper V1)
%   beta : arm angle below the horizontal plane [deg] = asin(-(up . arm direction))
%   gam  : ball-joint tilt [deg] = |90 - angle(rod, ball stem axis)|
N = size(q,2); beta = nan(3,N); gam = nan(3,N);
for i = 1:3
    a = G.a(:,i); u = G.u(:,i); w = cross(a,u);
    dirArm = u*cos(q(i,:)) + w*sin(q(i,:));
    beta(i,:) = asind(-(G.up.'*dirArm));
    C = delta_elbow(q(i,:),G,i);
    rod = X + G.E(:,i) - C;
    c = (a.'*rod) ./ sqrt(sum(rod.^2,1));
    gam(i,:) = abs(90 - acosd(max(min(c,1),-1)));
end
end

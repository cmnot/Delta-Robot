function [X, ok] = delta_fk(q, G)
%DELTA_FK  Forward kinematics by intersecting three spheres (trilateration).
%   [X, ok] = delta_fk(q, G)    q : 3xN [rad]    X : 3xN [m] world
%   Sphere i: centre S_i = C_i(q_i) - E_i, radius L2_i.
%   Of the two intersections, the LOWER one (along -G.up) is returned.
N = size(q,2); X = nan(3,N); ok = false(1,N);
C = zeros(3,3,N);
for i = 1:3, C(:,i,:) = reshape(delta_elbow(q(i,:),G,i),3,1,N); end
r = G.L2(:);
for n = 1:N
    S = squeeze(C(:,:,n)) - G.E;            % 3x3, column i = centre i
    p1 = S(:,1); p2 = S(:,2); p3 = S(:,3);
    d  = norm(p2-p1);
    ex = (p2-p1)/d;
    ii = ex.'*(p3-p1);
    t  = p3 - p1 - ii*ex;
    if norm(t) < 1e-12, continue; end
    ey = t/norm(t);
    ez = cross(ex,ey);
    jj = ey.'*(p3-p1);
    x  = (r(1)^2 - r(2)^2 + d^2)/(2*d);
    y  = (r(1)^2 - r(3)^2 + ii^2 + jj^2)/(2*jj) - (ii/jj)*x;
    z2 = r(1)^2 - x^2 - y^2;
    if z2 < 0, continue; end
    z = sqrt(z2);
    Xa = p1 + x*ex + y*ey + z*ez;
    Xb = p1 + x*ex + y*ey - z*ez;
    if G.up.'*Xa < G.up.'*Xb, X(:,n) = Xa; else, X(:,n) = Xb; end
    ok(n) = true;
end
end

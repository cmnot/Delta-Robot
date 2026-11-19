function C = delta_elbow(q, G, i)
%DELTA_ELBOW  Position of arm i's ball-pair midpoint (the "elbow") in world frame.
%   C = delta_elbow(q, G, i)   q : 1xN joint angle [rad] (Simscape q of joint i)
%   C : 3xN [m]
%   Elbow C = P + d*a + L1*(cos(q)*u + sin(q)*(a x u))
P = G.P(:,i); a = G.a(:,i); u = G.u(:,i); w = cross(a,u);
q = q(:).';
C = P + G.d(i)*a + G.L1(i)*(u*cos(q) + w*sin(q));
end

function th_deg = circle_IK(t, circ, geom)
% ====================== ACTUATOR OF THE DELTA ROBOT ======================
% Input : t     simulation time [s]
% Param : circ  your variables from params_delta.m
%         geom  robot geometry measured from this model (results/geometry.mat)
% Output: th_deg  3 servo angles [deg] -> Revolute Joint, Joint1, Joint2
%
% 1) WHERE should the effector be at time t?  (robot frame, mm)
R3 = [geom.ex geom.ey geom.ez];
pHome  = (R3.'*(geom.Xhome - geom.O))*1000;      % home position
pStart = [circ.R; 0; circ.zc];                    % first point on the circle
t1 = circ.Tmove;  t2 = t1 + circ.Tcircle;  t3 = t2 + circ.Tmove;
if t < t1                                         % phase 1: home -> circle
    s = mj(t/circ.Tmove);
    p = pHome + (pStart - pHome)*s;
elseif t < t2                                     % phase 2: circle
    phi = 2*pi*circ.laps*mj((t - t1)/circ.Tcircle);
    p = [circ.R*cos(phi); circ.R*sin(phi); circ.zc];
elseif t < t3                                     % phase 3: circle -> home
    s = mj((t - t2)/circ.Tmove);
    p = pStart + (pHome - pStart)*s;
else                                              % stay at home
    p = pHome;
end
% 2) the same point in the Simscape world frame [m]
X = geom.O + R3*p/1000;
% 3) INVERSE KINEMATICS: which servo angle puts each arm there?
%    rod length must fit:  |X + E_i - C_i(q)| = L2   ->   A cos q + B sin q = K
th_deg = zeros(3,1);
for i = 1:3
    a = geom.a(:,i);  u = geom.u(:,i);  v = cross(a,u);
    D = X + geom.E(:,i) - geom.P(:,i) - geom.d(i)*a;
    A = 2*geom.L1(i)*(u.'*D);
    B = 2*geom.L1(i)*(v.'*D);
    K = D.'*D + geom.L1(i)^2 - geom.L2(i)^2;
    c = min(max(K/sqrt(A^2+B^2), -1), 1);
    q = atan2(B,A) + geom.branch(i)*acos(c);       % elbow-out solution
    qh = geom.qhome(i);
    q = qh + mod(q - qh + pi, 2*pi) - pi;          % keep it near the home angle
    th_deg(i) = q*180/pi;
end
end

function s = mj(x)
% minimum-jerk blend 0 -> 1
x = min(max(x,0),1);
s = 10*x^3 - 15*x^4 + 6*x^5;
end

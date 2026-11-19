function K = circle_kinematics(t, circ, G)
%CIRCLE_KINEMATICS  Analytical position / velocity / acceleration analysis of the circle
%   (independent of Simscape, used to check the simulation)
%
%   Effector path: lib/circle_path.m (home -> circle -> home, minimum-jerk), analytical derivatives
%   For every arm i the rod length is constant:
%       f_i = |s_i|^2 - L2^2 = 0,      s_i = X + E_i - C_i(q_i)      (rod vector)
%   Differentiate once  (VELOCITY):      s_i'(Xd - C_i' qd_i) = 0
%       ->  qd_i  = s_i'Xd / (s_i'C_i')
%   Differentiate twice (ACCELERATION):  s_i'(Xdd - C_i'' qd_i^2 - C_i' qdd_i) + |sd_i|^2 = 0
%       ->  qdd_i = (s_i'(Xdd - C_i'' qd_i^2) + |sd_i|^2) / (s_i'C_i')
%   with C_i' = L1(-sin q u + cos q v),  C_i'' = -L1(cos q u + sin q v),  sd_i = Xd - C_i' qd_i
%
%   Outputs (world frame, SI units unless noted): K.X, K.Xd, K.Xdd (3xN), K.q, K.qd, K.qdd (3xN, Simscape
%   joint convention), K.th (servo angle below horizontal, deg), K.s (rod vectors 3x3xN),
%   K.wRodArm, K.wRodEff (3x3xN relative angular velocity of the rod at the arm / effector ball joint)
t = t(:).';  N = numel(t);
[xl, vl, al] = circle_path(t, circ, G);           % mm, mm/s, mm/s^2 (robot frame)
xl = xl/1000; vl = vl/1000; al = al/1000;
Rm = [G.ex G.ey G.ez];
K.t = t;  K.Xl = xl;  K.X = G.O + Rm*xl;  K.Xd = Rm*vl;  K.Xdd = Rm*al;
K.q = nan(3,N); qr = G.qhome;
for n = 1:N
    [K.q(:,n), ok] = delta_ik(K.X(:,n), G, qr);
    if ~ok, error('Circle point at t = %.3f s is outside the workspace', t(n)); end
    qr = K.q(:,n);
end
K.qd = zeros(3,N); K.qdd = zeros(3,N); K.s = zeros(3,3,N);
K.wRodArm = zeros(3,3,N); K.wRodEff = zeros(3,3,N);
for i = 1:3
    a = G.a(:,i); u = G.u(:,i); v = cross(a,u); L1 = G.L1(i);
    q = K.q(i,:);
    C   = G.P(:,i) + G.d(i)*a + L1*(u*cos(q) + v*sin(q));
    C1  = L1*(-u*sin(q) + v*cos(q));          % dC/dq
    C2  = -L1*(u*cos(q) + v*sin(q));          % d2C/dq2
    s   = K.X + G.E(:,i) - C;
    den = sum(s.*C1,1);
    qd  = sum(s.*K.Xd,1)./den;
    sd  = K.Xd - C1.*qd;
    qdd = (sum(s.*(K.Xdd - C2.*qd.^2),1) + sum(sd.^2,1))./den;
    K.qd(i,:) = qd;  K.qdd(i,:) = qdd;  K.s(:,i,:) = reshape(s,3,1,N);
    % rod angular velocity (perpendicular to the rod): w = n x nd
    L = sqrt(sum(s.^2,1)); nrod = s./L;
    nd = (sd - nrod.*sum(nrod.*sd,1))./L;
    wr = cross(nrod, nd, 1);
    K.wRodEff(:,i,:) = reshape(wr, 3,1,N);              % effector does not rotate
    K.wRodArm(:,i,:) = reshape(wr - a*qd, 3,1,N);       % arm rotates with qd about a
end
K.th = servo_angle(K.q, G);
end

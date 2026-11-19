function [q, ok, okArm] = delta_ik(X, G, qRef)
%DELTA_IK  Inverse kinematics of the delta robot (general vector form).
%   [q, ok] = delta_ik(X, G, qRef)
%   X    : 3xN effector point (centroid of the 6 effector balls) in WORLD frame [m]
%   G    : geometry struct from run_02_calibrate (delta_geometry.mat)
%   qRef : 3x1 or 3xN reference angles [rad] used to pick the elbow branch
%          (default G.qhome -> keeps the same "elbow-out" branch as the model)
%   q    : 3xN joint angles [rad], same sign convention as Simscape joint q
%   ok   : 1xN true if all three arms can reach
%
%   For arm i:  |X + E_i - C_i(q_i)| = L2_i
%   ->  A*cos(q) + B*sin(q) = K
%       A = 2 L1 (D.u),  B = 2 L1 (D.w),  K = |D|^2 + L1^2 - L2^2,
%       D = X + E_i - P_i - d_i a_i
%   ->  q = atan2(B,A) +/- acos(K / sqrt(A^2+B^2))
if nargin < 3 || isempty(qRef), qRef = G.qhome; end
N = size(X,2);
if size(qRef,2) == 1, qRef = repmat(qRef,1,N); end
q = nan(3,N); okArm = false(3,N);
for i = 1:3
    P = G.P(:,i); a = G.a(:,i); u = G.u(:,i); w = cross(a,u);
    L1 = G.L1(i); L2 = G.L2(i);
    D = X + G.E(:,i) - (P + G.d(i)*a);
    A = 2*L1*(u.'*D);
    B = 2*L1*(w.'*D);
    K = sum(D.^2,1) + L1^2 - L2^2;
    R = hypot(A,B);
    c = K./R;
    okArm(i,:) = abs(c) <= 1;
    c = max(min(c,1),-1);
    phi = atan2(B,A);
    s = acos(c);
    q1 = wrapAround(phi + s, qRef(i,:));
    q2 = wrapAround(phi - s, qRef(i,:));
    use1 = abs(q1 - qRef(i,:)) <= abs(q2 - qRef(i,:));
    qi = q2; qi(use1) = q1(use1);
    q(i,:) = qi;
end
ok = all(okArm,1);
q(:,~ok) = NaN;
end

function y = wrapAround(x, ref)
% put angle x within (ref-pi, ref+pi]
y = ref + mod(x - ref + pi, 2*pi) - pi;
end

function th = servo_angle(q, G)
%SERVO_ANGLE  Servo arm angle BELOW the horizontal plane [deg]  (Symbolic convention:
%   0 = arm horizontal pointing outward, +90 = arm pointing straight down)
%   q : 3xN Simscape joint angles [rad]
[O,~,~,ez] = robot_frame(G);
th = zeros(size(q));
for i = 1:3
    a = G.a(:,i); u = G.u(:,i); w = cross(a,u);
    r = G.P(:,i) - O; r = r - (r.'*ez)*ez; r = r/norm(r);     % outward horizontal
    d = u*cos(q(i,:)) + w*sin(q(i,:));                         % arm direction
    th(i,:) = atan2d(-(ez.'*d), r.'*d);
end
end

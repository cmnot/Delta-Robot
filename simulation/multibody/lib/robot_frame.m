function [O, ex, ey, ez] = robot_frame(G)
%ROBOT_FRAME  Robot frame in world coordinates (same convention as the Symbolic part)
%   O  : on the robot axis at the height of the servo shafts [m]
%   ez : up (= -gravity),  ey : horizontal toward servo 1,  ex = ey x ez
ez = G.up(:);
O  = mean(G.P,2);
v  = G.P(:,1) - O;  v = v - (v.'*ez)*ez;
ey = v/norm(v);
ex = cross(ey, ez);
end

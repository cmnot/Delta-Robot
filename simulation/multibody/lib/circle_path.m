function [p, pd, pdd] = circle_path(t, circ, G)
%CIRCLE_PATH  Analytical effector path, velocity and acceleration (robot frame, mm, mm/s, mm/s^2)
%   Same formulas as the circle_IK block (home -> circle -> home, minimum-jerk blends).
t = t(:).'; N = numel(t);
R3 = [G.ex G.ey G.ez];
pH = (R3.'*(G.Xhome - G.O))*1000;  pS = [circ.R; 0; circ.zc];
t1 = circ.Tmove; t2 = t1 + circ.Tcircle; t3 = t2 + circ.Tmove;
p = zeros(3,N); pd = zeros(3,N); pdd = zeros(3,N);
for n = 1:N
    tn = t(n);
    if tn < t1
        [s,sd,sdd] = mj(tn/circ.Tmove, circ.Tmove);
        p(:,n) = pH + (pS-pH)*s; pd(:,n) = (pS-pH)*sd; pdd(:,n) = (pS-pH)*sdd;
    elseif tn < t2
        [s,sd,sdd] = mj((tn-t1)/circ.Tcircle, circ.Tcircle);
        k = 2*pi*circ.laps; ph = k*s; phd = k*sd; phdd = k*sdd; R = circ.R;
        p(:,n)   = [R*cos(ph); R*sin(ph); circ.zc];
        pd(:,n)  = R*phd*[-sin(ph); cos(ph); 0];
        pdd(:,n) = R*phdd*[-sin(ph); cos(ph); 0] - R*phd^2*[cos(ph); sin(ph); 0];
    elseif tn < t3
        [s,sd,sdd] = mj((tn-t2)/circ.Tmove, circ.Tmove);
        p(:,n) = pS + (pH-pS)*s; pd(:,n) = (pH-pS)*sd; pdd(:,n) = (pH-pS)*sdd;
    else
        p(:,n) = pH;
    end
end
end

function [s, sd, sdd] = mj(x, T)
x = min(max(x,0),1);
s   = 10*x^3 - 15*x^4 + 6*x^5;
sd  = (30*x^2 - 60*x^3 + 30*x^4)/T;
sdd = (60*x - 180*x^2 + 120*x^3)/T^2;
end

function circ = params_delta()
%PARAMS_DELTA  >>> YOUR VARIABLES <<<  (edit here, then run step3_circle or press Run in the model)
%   Motion (same as the circle of Claude_helper V1):
%     phase 1  home  -> circle start (straight line)        time Tmove
%     phase 2  circle, laps turns                           time Tcircle
%     phase 3  circle start -> home (straight line)         time Tmove
%   Every phase uses the minimum-jerk blend  s(x) = 10x^3 - 15x^4 + 6x^5  (x = 0..1),
%   so velocity and acceleration start and end at zero (no jerk at the joints).
%   On the circle:  x = R cos(phi), y = R sin(phi), z = zc,   phi = 2*pi*laps*s
%   Robot frame: origin on the robot axis at the servo shafts, z up, y toward servo 1.
circ.R      = 30;    % [mm] circle radius
circ.zc     = -81;   % [mm] circle height (below the servo shafts) - middle of the best workspace band
circ.laps   = 2;     % [-]  number of turns
circ.Tcircle = 4;    % [s]  time for all turns
circ.Tmove  = 1.5;   % [s]  home -> circle and circle -> home
circ.Thold  = 0.3;   % [s]  stay at home at the end
circ.filterTau = 0.002;  % [s] smoothing of the angle command inside Simscape
circ.Tend = 2*circ.Tmove + circ.Tcircle + circ.Thold;   % [s] stop time (do not edit)
end

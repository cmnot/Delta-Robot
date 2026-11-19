%% STEP3_CIRCLE  Simulate the circle and analyse the SERVO JOINTS:
%   angle, angular velocity, angular acceleration and torque the servo must give,
%   plus the passive ball joints and the effector. Simscape results are checked against
%   the analytical velocity / acceleration analysis (lib/circle_kinematics.m).
%   Edit the circle in params_delta.m.
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'lib'));
cfg = v2_config(); addpath(cfg.root, cfg.modelDir);
circ = params_delta();
S = load(fullfile(cfg.resultsDir,'geometry.mat')); G = S.geom;
[O, ex, ey, ez] = robot_frame(G);  Rm = [ex ey ez];
outDir = fullfile(cfg.resultsDir,'circle'); if ~exist(outDir,'dir'), mkdir(outDir); end

%% 1) simulate (the MATLAB Function block inside the model does circle + IK)
load_system(cfg.mdl);
fprintf('Simulating %g s ...\n', circ.Tend);
out = sim(cfg.mdl, 'ReturnWorkspaceOutputs','on');
[t, qS]  = getsig(out,'th_q');   qS = deg2rad(qS.');      % Simscape joint angle  [rad]
[~, wS]  = getsig(out,'th_w');   wS = deg2rad(wS.');      % [rad/s]
[~, aS]  = getsig(out,'th_a');   aS = deg2rad(aS.');      % [rad/s^2]
[~, tau] = getsig(out,'th_tau'); tau = tau.';             % [N*m]
[~, pE]  = getsig(out,'pE1');    XS = pE.'/1000 - G.effOff;   % effector centre [m]
t = t(:).';

%% 2) whole motion: home -> circle -> home (starts and ends at rest)
tA = t;
K = circle_kinematics(tA, circ, G);

% Simscape joint angle q  ->  servo angle theta (below horizontal), same sign convention as Symbolic
sgn = zeros(3,1);
for i = 1:3
    d1 = servo_angle(G.qhome + [0.01*(1:3==i)].', G) - servo_angle(G.qhome, G);
    sgn(i) = sign(d1(i));
end
thS = servo_angle(qS, G);          thA = K.th;                 % [deg]
wSd = rad2deg(sgn.*wS);            wAd = rad2deg(sgn.*K.qd);   % [deg/s]
aSd = rad2deg(sgn.*aS);            aAd = rad2deg(sgn.*K.qdd);  % [deg/s^2]
tauS = sgn.*tau;                                               % [N*m] positive = pushes arm down
powr = tauS.*deg2rad(wSd);                                     % [W]

% passive ball joints (analytical): |relative angular velocity|, |relative angular acceleration|
dt = t(2)-t(1);
wArm = squeeze(sqrt(sum(K.wRodArm.^2,1)));  wEff = squeeze(sqrt(sum(K.wRodEff.^2,1)));   % 3xN rad/s
aArmV = gradient3(K.wRodArm, dt);  aEffV = gradient3(K.wRodEff, dt);
aArm = squeeze(sqrt(sum(aArmV.^2,1)));  aEff = squeeze(sqrt(sum(aEffV.^2,1)));
if size(wArm,1) ~= 3, wArm = wArm.'; wEff = wEff.'; aArm = aArm.'; aEff = aEff.'; end

% effector
vA = sqrt(sum(K.Xd.^2,1))*1e3;  accA = sqrt(sum(K.Xdd.^2,1))*1e3;           % mm/s, mm/s^2
vS = sqrt(sum(gradient(XS,dt).^2,1))*1e3;
ePath = sqrt(sum((XS - K.X).^2,1))*1e3;                                    % mm
XlS = Rm.'*(XS - O)*1e3;  XlA = K.Xl*1e3;

%% 3) figures (whole motion)
lw = 1.4; L3 = {'servo 1','servo 2','servo 3'};
f = newfig('Path');
plot3(XlA(1,:),XlA(2,:),XlA(3,:),'k--','LineWidth',1); hold on;
plot3(XlS(1,:),XlS(2,:),XlS(3,:),'r','LineWidth',lw);
Pb = Rm.'*(G.P-O)*1e3; plot3(Pb(1,:),Pb(2,:),Pb(3,:),'bo','MarkerFaceColor','b');
text(Pb(1,:),Pb(2,:),Pb(3,:)+6, {' servo 1',' servo 2',' servo 3'});
axis equal; grid on; xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]'); view(35,25);
legend('commanded circle','Simscape effector','servo shafts','Location','best');
title(sprintf('Effector path  (max deviation %.3f mm)', max(ePath)));
saveas(f, fullfile(outDir,'1_path.png'));

f = newfig('Servo angle');
plotPair(t, thS, thA, lw); ylabel('\theta [deg]  (below horizontal)');
title('Servo angle \theta (solid = Simscape, dashed = analytical IK)'); saveas(f, fullfile(outDir,'2_servo_angle.png'));

f = newfig('Servo angular velocity');
plotPair(t, wSd, wAd, lw); ylabel('\omega [deg/s]');
title('Servo angular velocity (solid = Simscape, dashed = analytical)'); saveas(f, fullfile(outDir,'3_servo_velocity.png'));

f = newfig('Servo angular acceleration');
plotPair(t, aSd, aAd, lw); ylabel('\alpha [deg/s^2]');
title('Servo angular acceleration (solid = Simscape, dashed = analytical)'); saveas(f, fullfile(outDir,'4_servo_acceleration.png'));

f = newfig('Servo torque');
plot(t, tauS*1e3, 'LineWidth', lw); grid on; xlabel('t [s]'); ylabel('\tau [mN\cdotm]'); legend(L3,'Location','best');
title('Torque each servo must provide (Simscape, no payload)'); saveas(f, fullfile(outDir,'5_servo_torque.png'));

f = newfig('Ball joints');
subplot(2,1,1); plot(t, rad2deg(wArm),'LineWidth',lw); hold on; set(gca,'ColorOrderIndex',1);
plot(t, rad2deg(wEff),'--','LineWidth',lw); grid on; ylabel('|\omega_{rel}| [deg/s]');
legend('arm side 1','arm side 2','arm side 3','effector side 1','effector side 2','effector side 3','Location','eastoutside');
title('Passive ball joints: relative angular velocity');
subplot(2,1,2); plot(t, rad2deg(aArm),'LineWidth',lw); hold on; set(gca,'ColorOrderIndex',1);
plot(t, rad2deg(aEff),'--','LineWidth',lw); grid on; ylabel('|\alpha_{rel}| [deg/s^2]'); xlabel('t [s]');
title('Passive ball joints: relative angular acceleration'); saveas(f, fullfile(outDir,'6_ball_joints.png'));

f = newfig('Effector');
subplot(2,1,1); plot(t, vS,'r','LineWidth',lw); hold on; plot(t, vA,'k--'); grid on; ylabel('|v| [mm/s]');
legend('Simscape','analytical'); title('Effector speed');
subplot(2,1,2); plot(t, accA,'k','LineWidth',lw); grid on; ylabel('|a| [mm/s^2]'); xlabel('t [s]');
title('Effector acceleration (analytical)'); saveas(f, fullfile(outDir,'7_effector.png'));

%% 4) summary table
mx = @(v) max(abs(v),[],2);
fid = fopen(fullfile(outDir,'summary.txt'),'w');
for fk = [1 fid]
    fprintf(fk,'\n===== Circle: R = %g mm at z = %g mm, %g laps in %g s (home -> circle -> home, %.1f s total) =====\n', ...
        circ.R, circ.zc, circ.laps, circ.Tcircle, circ.Tend);
    fprintf(fk,'Effector: max speed %.1f mm/s, max acceleration %.1f mm/s^2, max path deviation %.4f mm\n', max(vA), max(accA), max(ePath));
    fprintf(fk,'\nSERVO JOINTS (what each servo must deliver)\n');
    fprintf(fk,'%-28s %12s %12s %12s\n','', 'servo 1','servo 2','servo 3');
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','theta min [deg]', min(thS,[],2));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','theta max [deg]', max(thS,[],2));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','theta range [deg]', max(thS,[],2)-min(thS,[],2));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','|omega| max [deg/s]', mx(wSd));
    fprintf(fk,'%-28s %12.3f %12.3f %12.3f\n','|omega| max [rad/s]', deg2rad(mx(wSd)));
    fprintf(fk,'%-28s %12.3f %12.3f %12.3f\n','|omega| max [rpm]', mx(wSd)/6);
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','|alpha| max [deg/s^2]', mx(aSd));
    fprintf(fk,'%-28s %12.3f %12.3f %12.3f\n','|alpha| max [rad/s^2]', deg2rad(mx(aSd)));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','|tau| max [mN*m]', mx(tauS)*1e3);
    fprintf(fk,'%-28s %12.3f %12.3f %12.3f\n','|tau| max [kgf*cm]', mx(tauS)/0.0980665);
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','tau rms [mN*m]', sqrt(mean(tauS.^2,2))*1e3);
    fprintf(fk,'%-28s %12.4f %12.4f %12.4f\n','|power| max [W]', mx(powr));
    fprintf(fk,'\nCHECK Simscape vs analytical (max difference)\n');
    fprintf(fk,'%-28s %12.4f %12.4f %12.4f\n','theta [deg]', mx(thS-thA));
    fprintf(fk,'%-28s %12.4f %12.4f %12.4f\n','omega [deg/s]', mx(wSd-wAd));
    fprintf(fk,'%-28s %12.4f %12.4f %12.4f\n','alpha [deg/s^2]', mx(aSd-aAd));
    fprintf(fk,'\nPASSIVE BALL JOINTS (relative to the part they sit on)\n');
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','arm side |omega| max [deg/s]', rad2deg(mx(wArm)));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','arm side |alpha| max [deg/s^2]', rad2deg(mx(aArm)));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','eff. side |omega| max [deg/s]', rad2deg(mx(wEff)));
    fprintf(fk,'%-28s %12.2f %12.2f %12.2f\n','eff. side |alpha| max [deg/s^2]', rad2deg(mx(aEff)));
end
fclose(fid);
T = array2table([t.', thS.', wSd.', aSd.', tauS.'*1e3], 'VariableNames', ...
    {'t_s','th1_deg','th2_deg','th3_deg','w1_degps','w2_degps','w3_degps', ...
     'a1_degps2','a2_degps2','a3_degps2','tau1_mNm','tau2_mNm','tau3_mNm'});
writetable(T, fullfile(outDir,'servo_timeseries.csv'));
save(fullfile(outDir,'circle_data.mat'),'t','thS','thA','wSd','wAd','aSd','aAd','tauS','XS','K','circ');
fprintf('\nSaved figures, summary.txt and servo_timeseries.csv in %s\n', outDir);
fprintf('Video: set the view in Mechanics Explorer, then run step4_video\n');

function plotPair(t, A, B, lw)
plot(t, A, 'LineWidth', lw); hold on; set(gca,'ColorOrderIndex',1);
plot(t, B, '--', 'LineWidth', 1); grid on; xlabel('t [s]');
legend('servo 1','servo 2','servo 3','Location','best');
end

function D = gradient3(V, dt)
% time derivative of a 3x3xN array along the 3rd dimension
D = zeros(size(V));
for i = 1:size(V,2)
    D(:,i,:) = reshape(gradient(squeeze(V(:,i,:)), dt), 3, 1, []);
end
end

%% STEP2_WORKSPACE  Reachable workspace (same method and figures as Claude_helper V1)
% A point is usable when: IK has a solution for all 3 arms, every arm angle is inside
% [beta_min, beta_max] below horizontal, and every ball-joint tilt <= ball_alpha_max.
clear; clc;
addpath(fullfile(fileparts(mfilename('fullpath')),'lib'));
cfg = v2_config(); addpath(cfg.root);
S = load(fullfile(cfg.resultsDir,'geometry.mat')); G = S.geom;
[O, ex, ey, ez] = robot_frame(G);  Rm = [ex ey ez];
outDir = fullfile(cfg.resultsDir,'workspace'); if ~exist(outDir,'dir'), mkdir(outDir); end
reach = max(G.L1) + max(G.L2);
fR = max(arrayfun(@(k) norm((G.P(:,k)-O) - ((G.P(:,k)-O).'*ez)*ez), 1:3));
ok_at = @(X) checkPts(X, G, cfg);

%% usable full-circle radius at each height (rays from the robot axis)
hs = -(0.005:0.002:reach+0.01); phis = deg2rad(0:5:355); rs = 0:0.001:(fR+reach);
rUse = zeros(size(hs)); rMax = zeros(size(hs));
for ih = 1:numel(hs)
    [Rg,Pg] = ndgrid(rs,phis);
    X = O + hs(ih)*ez + ex*(Rg(:).'.*cos(Pg(:).')) + ey*(Rg(:).'.*sin(Pg(:).'));
    okv = reshape(ok_at(X), size(Rg));
    if ~okv(1,1), continue; end
    rr = zeros(1,numel(phis));
    for ip = 1:numel(phis)
        b = find(~okv(:,ip),1); if isempty(b), rr(ip) = rs(end); else, rr(ip) = rs(max(b-1,1)); end
    end
    rUse(ih) = min(rr); rMax(ih) = max(Rg(okv));
end
reachable = rUse > 0; zTop = max(hs(reachable)); zBot = min(hs(reachable));
bestR = max(rUse); band = find(rUse >= bestR - 0.001);
[~, ib] = min(abs(hs - mean(hs(band)))); bestH = hs(ib);     % middle of the best band

%% 3-D cloud
g = -(fR+reach):0.004:(fR+reach);
[A,B,H] = ndgrid(g,g,hs(1:2:end));
X = O + ex*A(:).' + ey*B(:).' + ez*H(:).';
P = Rm.'*(X(:,ok_at(X)) - O)*1e3;
Pb = Rm.'*(G.P - O)*1e3;

f1 = newfig('Workspace 3D');
scatter3(P(1,:),P(2,:),P(3,:),4,P(3,:),'filled'); hold on;
plot3(Pb(1,:),Pb(2,:),Pb(3,:),'ko','MarkerFaceColor','k');
% red circle 1: largest usable full circle (same as the top view)
th = linspace(0,2*pi,200); rU = rUse(ib)*1e3;
plot3(rU*cos(th), rU*sin(th), bestH*1e3 + 0*th, 'r-', 'LineWidth', 2.5);
% red circle 2: the circle the robot actually moves on (params_delta.m)
circ = params_delta();
plot3(circ.R*cos(th), circ.R*sin(th), circ.zc + 0*th, 'r--', 'LineWidth', 2.5);
axis equal; grid on; colorbar; xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z (up) [mm]');
legend('reachable points','servo shafts', ...
    sprintf('usable circle r = %.0f mm (z = %.0f mm)', rU, bestH*1e3), ...
    sprintf('motion circle R = %g mm (z = %g mm)', circ.R, circ.zc), 'Location','northeastoutside');
title('Reachable workspace of the effector point'); view(35,20);
saveas(f1, fullfile(outDir,'workspace_3d.png'));

f2 = newfig('Usable radius');
plot(rUse*1e3, hs*1e3,'b','LineWidth',1.5); hold on; plot(rMax*1e3, hs*1e3,'r--','LineWidth',1);
plot(rUse(ib)*1e3, bestH*1e3,'ko','MarkerFaceColor','y');
grid on; xlabel('radius [mm]'); ylabel('z relative to servo shafts [mm]');
legend('usable full circle radius','max reach in any direction','best height','Location','best');
title('Workspace size vs height'); saveas(f2, fullfile(outDir,'radius_vs_height.png'));

f3 = newfig('Top view at best height');
[Gx,Gy] = ndgrid(g,g);
oks = reshape(ok_at(O + bestH*ez + ex*Gx(:).' + ey*Gy(:).'), size(Gx));
imagesc(g*1e3,g*1e3,oks.'); axis xy equal tight; colormap([1 1 1; 0.6 0.8 1]); hold on;
th = linspace(0,2*pi,200); plot(rUse(ib)*1e3*cos(th), rUse(ib)*1e3*sin(th),'r','LineWidth',1.5);
plot(Pb(1,:),Pb(2,:),'ko','MarkerFaceColor','k');
xlabel('x [mm]'); ylabel('y [mm]');
title(sprintf('Top view at z = %.0f mm (usable circle r = %.1f mm)', bestH*1e3, rUse(ib)*1e3));
saveas(f3, fullfile(outDir,'top_view_best_height.png'));

f4 = newfig('Side section');
[Gx,Gz] = ndgrid(g,hs);
oks = reshape(ok_at(O + ex*Gx(:).' + ez*Gz(:).'), size(Gx));
imagesc(g*1e3,hs*1e3,oks.'); axis xy equal tight; colormap([1 1 1; 0.6 0.8 1]);
xlabel('x [mm]'); ylabel('z (up) [mm]'); title('Side section through the robot axis');
saveas(f4, fullfile(outDir,'side_section.png'));

fid = fopen(fullfile(outDir,'summary.txt'),'w');
for fk = [1 fid]
    fprintf(fk,'\n===== Workspace (robot frame: z = 0 at servo shafts, up positive) =====\n');
    fprintf(fk,'Limits: arm %g..%g deg below horizontal, ball tilt <= %g deg\n', cfg.beta_min_deg, cfg.beta_max_deg, cfg.ball_alpha_max_deg);
    fprintf(fk,'On the robot axis the effector reaches z = %.1f .. %.1f mm (stroke %.1f mm)\n', zTop*1e3, zBot*1e3, (zTop-zBot)*1e3);
    fprintf(fk,'Largest usable full circle : radius %.1f mm, for z = %.1f .. %.1f mm\n', bestR*1e3, hs(band(1))*1e3, hs(band(end))*1e3);
    fprintf(fk,'Best work height           : z = %.1f mm\n', bestH*1e3);
    fprintf(fk,'Max horizontal reach       : %.1f mm from the axis\n', max(rMax)*1e3);
    fprintf(fk,'Home pose                  : z = %.1f mm\n', -((O-G.Xhome).'*ez)*1e3);
end
fclose(fid);
save(fullfile(outDir,'workspace_data.mat'),'hs','rUse','rMax','bestR','bestH','zTop','zBot');

function ok = checkPts(X, G, cfg)
[q, ok] = delta_ik(X, G, G.qhome);
if any(ok)
    [beta, gam] = delta_limits(X(:,ok), q(:,ok), G);
    good = all(beta >= cfg.beta_min_deg & beta <= cfg.beta_max_deg & gam <= cfg.ball_alpha_max_deg, 1);
    idx = find(ok); ok(idx(~good)) = false;
end
end

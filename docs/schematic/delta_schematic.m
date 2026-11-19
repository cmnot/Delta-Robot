function delta_schematic(mode)
%DELTA_SCHEMATIC  3-D kinematic (skeleton) diagram of the Delta Robot
%   delta_schematic              -> joints as in the Simscape Multibody model
%                                   (arm side = Spherical, platform side = Universal)
%   delta_schematic('prototype') -> joints as in the real robot (12 Spherical)
%
%   Mechanics of Machinery convention
%     link 1  = ground (fixed base, hatched)
%     link 2-4  = servo arms (upper arms)
%     link 5-10 = parallel rods (two per arm)
%     link 11   = moving platform (end effector)
%     joints    = capital letters, A-C revolute (servo shafts)
%   Colours follow the Simscape Multibody model.
%   The figure is saved next to this file as PNG.

if nargin < 1, mode = 'multibody'; end
mode = lower(mode);
isMB = strcmp(mode, 'multibody');

%% ---------------- geometry (measured from the Simscape model) [mm] ----------------
L1 = 40.0;            % servo arm length
L2 = 82.0;            % rod length
f  = 40.72;           % radius: robot axis -> servo shaft
e  = 29.64;           % radius: platform centre -> ball pair
h  = 15.0;            % half spacing of the two parallel rods
phi = [90 -30 -150];  % servo azimuth [deg]  (servo 1 on +y)
X  = [0; 0; -95];     % platform centre for the drawing (inside the workspace)

%% ---------------- colours (Simscape model) ----------------
cGround = [0.30 0.30 0.30];   % upper base + servo body (dark grey)
cServo  = [0.45 0.45 0.45];
cArm    = [0.35 0.72 0.50];   % link 1 part "link1 with servo" (green)
cRod    = [0.76 0.74 0.33];   % rod1_v5B (olive yellow)
cPlat   = [0.72 0.72 0.72];   % base1 = moving platform (light grey)
cJoint  = [0.85 0.15 0.15];   % joint symbols (red) so they stand out

%% ---------------- inverse kinematics (same equation as the report) ----------------
z = [0;0;1];
th = zeros(1,3); P = zeros(3,3); C = zeros(3,3); r = zeros(3,3); t = zeros(3,3);
for i = 1:3
    r(:,i) = [cosd(phi(i)); sind(phi(i)); 0];   % radial (outward)
    t(:,i) = cross(z, r(:,i));                  % tangential = servo shaft axis
    P(:,i) = f*r(:,i);                          % servo shaft (revolute joint)
    D = X + e*r(:,i) - P(:,i);
    A = 2*L1*(D.'*r(:,i));  B = -2*L1*(D.'*z);  K = D.'*D + L1^2 - L2^2;
    if abs(K) > hypot(A,B), error('Point X is outside the workspace'); end
    th(i) = atan2(B,A) - acos(K/hypot(A,B));    % angle below horizontal (elbow out)
    C(:,i) = P(:,i) + L1*(cos(th(i))*r(:,i) - sin(th(i))*z);   % elbow
end

%% ---------------- figure ----------------
fig = figure('Color','w','Position',[80 60 1100 850]);
try, theme(fig,'light'); catch, end   % MATLAB R2025a+ dark theme -> force light (ignored in older versions)
ax = axes('Parent',fig, 'Position',[0.02 0.02 0.96 0.88]); hold(ax,'on'); axis(ax,'equal'); axis(ax,'off');

% ===== link 1 : ground (one rigid frame that holds the three servo shafts) =====
zb = 14;
G3 = P + [0;0;zb]*[1 1 1];
patch(ax, G3(1,:), G3(2,:), G3(3,:), cGround, 'FaceAlpha',0.18, 'EdgeColor',cGround, 'LineWidth',2.5);
for i = 1:3   % bracket + fixed-pivot (ground) symbol above each servo shaft
    plot3(ax, P(1,i)*[1 1], P(2,i)*[1 1], [P(3,i)+4.2 zb], '-', 'Color',cGround, 'LineWidth',3);
    drawGroundHatch(ax, G3(:,i), t(:,i), cGround);
end

% ===== revolute joints A, B, C (servo shafts) =====
for i = 1:3
    drawCyl(ax, P(:,i) - 9*t(:,i), P(:,i) + 9*t(:,i), 4.2, cServo);
    q1 = P(:,i) - 16*t(:,i); q2 = P(:,i) + 16*t(:,i);
    plot3(ax, [q1(1) q2(1)], [q1(2) q2(2)], [q1(3) q2(3)], '-.', 'Color',[0 0 0], 'LineWidth',0.8);
end

% ===== links 2-4 : servo arms (with the cross bar that carries the two balls) =====
U = zeros(3,6); Lw = zeros(3,6);  % upper / lower ball centres
for i = 1:3
    drawCyl(ax, P(:,i), C(:,i), 2.6, cArm);
    drawCyl(ax, C(:,i) - h*t(:,i), C(:,i) + h*t(:,i), 2.2, cArm);
    U(:,2*i-1) = C(:,i) - h*t(:,i);  U(:,2*i) = C(:,i) + h*t(:,i);
    Lw(:,2*i-1) = X + e*r(:,i) - h*t(:,i);  Lw(:,2*i) = X + e*r(:,i) + h*t(:,i);
end

% ===== links 5-10 : parallel rods =====
for k = 1:6, drawCyl(ax, U(:,k), Lw(:,k), 1.6, cRod); end

% ===== link 11 : moving platform =====
pp = []; for i = 1:3, pp = [pp, Lw(:,2*i-1), Lw(:,2*i)]; end %#ok<AGROW>
ctr = mean(pp(1:2,:),2); [~,ord] = sort(atan2(pp(2,:)-ctr(2), pp(1,:)-ctr(1)));
pp = pp(:,ord);
patch(ax, pp(1,:), pp(2,:), pp(3,:), cPlat, 'FaceAlpha',0.6, 'EdgeColor',[0.4 0.4 0.4], 'LineWidth',1.2);
plot3(ax, X(1), X(2), X(3), 'k+', 'MarkerSize',10, 'LineWidth',1.5);

% ===== joints D..I (arm side, spherical) and J..O (platform side) =====
for k = 1:6
    drawSphere(ax, U(:,k), 3.4, cJoint);
    if isMB
        i = ceil(k/2); a1 = t(:,i);
        a2 = cross(Lw(:,k) - U(:,k), a1); a2 = a2/norm(a2);
        drawCyl(ax, Lw(:,k) - 4.5*a1, Lw(:,k) + 4.5*a1, 1.5, cJoint);   % axis fixed in platform
        drawCyl(ax, Lw(:,k) - 4.5*a2, Lw(:,k) + 4.5*a2, 1.5, cJoint);   % axis fixed in rod
        drawSphere(ax, Lw(:,k), 1.9, cJoint);
    else
        drawSphere(ax, Lw(:,k), 3.4, cJoint);
    end
end

% ===== labels =====
L_ = 'ABCDEFGHIJKLMNO';
for i = 1:3   % revolute
    p = P(:,i) + 9*r(:,i) + [0;0;8];
    text(ax, p(1),p(2),p(3), L_(i), 'FontSize',15,'FontWeight','bold','Color',[0.6 0 0], 'HorizontalAlignment','center');
end
for k = 1:6
    i = ceil(k/2); sg = 2*mod(k,2)-1;
    p = U(:,k) + 8*r(:,i) - sg*5*t(:,i) + [0;0;4];
    text(ax, p(1),p(2),p(3), L_(3+k), 'FontSize',14,'FontWeight','bold','Color',[0.6 0 0], 'HorizontalAlignment','center');
    p = Lw(:,k) + 6*r(:,i) - sg*6*t(:,i) - [0;0;9];
    text(ax, p(1),p(2),p(3), L_(9+k), 'FontSize',14,'FontWeight','bold','Color',[0.6 0 0], 'HorizontalAlignment','center');
end
linkLabel(ax, 0.85*(G3(:,1)+G3(:,3))/2 + [0;0;6], '1');
for i = 1:3
    linkLabel(ax, (P(:,i)+C(:,i))/2 + [0;0;11], num2str(i+1));
end
for k = 1:6
    i = ceil(k/2); sg = 2*mod(k,2)-1;
    linkLabel(ax, 0.55*U(:,k) + 0.45*Lw(:,k) - sg*7*t(:,i) + 4*r(:,i), num2str(4+k));
end
linkLabel(ax, X + [0;0;4], '11');

% ===== robot frame =====
O = [0;0;0]; s = 18;
quiver3(ax, O(1),O(2),O(3), s,0,0, 0, 'Color',[0.85 0 0], 'LineWidth',1.6, 'MaxHeadSize',0.6);
quiver3(ax, O(1),O(2),O(3), 0,s,0, 0, 'Color',[0 0.6 0], 'LineWidth',1.6, 'MaxHeadSize',0.6);
quiver3(ax, O(1),O(2),O(3), 0,0,s, 0, 'Color',[0 0 0.85], 'LineWidth',1.6, 'MaxHeadSize',0.6);
text(ax, s+3,0,0,'x','Color',[0.85 0 0],'FontSize',12,'FontWeight','bold');
text(ax, 0,s+3,0,'y','Color',[0 0.6 0],'FontSize',12,'FontWeight','bold');
text(ax, 0,0,s+4,'z','Color',[0 0 0.85],'FontSize',12,'FontWeight','bold');

% ===== legend (dummy handles) =====
hL(1) = plot3(ax, nan,nan,nan,'s','MarkerSize',11,'MarkerFaceColor',cGround,'MarkerEdgeColor','k');
hL(2) = plot3(ax, nan,nan,nan,'s','MarkerSize',11,'MarkerFaceColor',cServo,'MarkerEdgeColor','k');
hL(3) = plot3(ax, nan,nan,nan,'-','LineWidth',5,'Color',cArm);
hL(4) = plot3(ax, nan,nan,nan,'-','LineWidth',4,'Color',cRod);
hL(5) = plot3(ax, nan,nan,nan,'s','MarkerSize',11,'MarkerFaceColor',cPlat,'MarkerEdgeColor','k');
hL(6) = plot3(ax, nan,nan,nan,'o','MarkerSize',10,'MarkerFaceColor',cJoint,'MarkerEdgeColor','k');
names = {'Link 1  Ground (fixed base)', 'A, B, C  Revolute joint (servo shaft)', ...
         'Link 2-4  Servo arm', 'Link 5-10  Parallel rod', 'Link 11  Moving platform', ...
         'D-I  Spherical joint (arm side)'};
if isMB
    hL(7) = plot3(ax, nan,nan,nan,'+','MarkerSize',12,'LineWidth',3,'Color',cJoint);
    names{7} = 'J-O  Universal joint (platform side)';
    ttl = 'Delta Robot kinematic diagram - Simscape Multibody model';
    dof = 'M = 6(11-1) - 5(3) - 4(6) - 3(6) = 3 DoF';
else
    names{6} = 'D-O  Spherical joint';
    ttl = 'Delta Robot kinematic diagram - physical prototype';
    dof = 'M = 6(11-1) - 5(3) - 3(12) = 9 DoF  (3 useful + 6 rod spin)';
end
lg = legend(ax, hL, names, 'Location','northeast', 'FontSize',11);
set(lg, 'Color','w', 'TextColor','k', 'EdgeColor','k');
title(ax, {ttl, ['Gruebler-Kutzbach:  ' dof]}, 'FontSize',14, 'Color','k');

view(ax, 65, 24);
lighting(ax,'gouraud'); camlight(ax,'headlight'); material(ax,'dull');
set(ax,'Clipping','off');

%% ---------------- command window summary ----------------
fprintf('\nDelta Robot kinematic diagram (%s)\n', mode);
fprintf('  Platform centre X = [%.1f %.1f %.1f] mm\n', X);
fprintf('  Servo angles below horizontal: %.1f  %.1f  %.1f deg\n', rad2deg(th));
fprintf('  Links  : N = 11  (1 ground, 2-4 arms, 5-10 rods, 11 platform)\n');
if isMB
    fprintf('  Joints : 3 R (A-C), 6 S (D-I), 6 U (J-O)\n');
else
    fprintf('  Joints : 3 R (A-C), 12 S (D-O)\n');
end
fprintf('  %s\n\n', dof);

%% ---------------- save ----------------
out = fullfile(fileparts(mfilename('fullpath')), ['delta_schematic_' mode '.png']);
set(fig,'PaperPositionMode','auto');
print(fig, out, '-dpng', '-r150');
fprintf('  Saved %s\n', out);
end

% =====================================================================
function drawCyl(ax, p1, p2, rad, col)
% solid cylinder from p1 to p2
[xc,yc,zc] = cylinder(rad, 24);
v = p2 - p1; Lc = norm(v); v = v/Lc;
zc = zc*Lc;
k = cross([0;0;1], v);
if norm(k) < 1e-9
    R = eye(3); if v(3) < 0, R = diag([1 -1 -1]); end
else
    k = k/norm(k); a = acos(max(-1,min(1,v(3))));
    Kx = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
    R = eye(3) + sin(a)*Kx + (1-cos(a))*Kx*Kx;
end
pts = R*[xc(:) yc(:) zc(:)].';
xc = reshape(pts(1,:),size(xc)) + p1(1);
yc = reshape(pts(2,:),size(yc)) + p1(2);
zc = reshape(pts(3,:),size(zc)) + p1(3);
surf(ax, xc, yc, zc, 'FaceColor',col, 'EdgeColor','none');
fill3(ax, xc(1,:), yc(1,:), zc(1,:), col, 'EdgeColor','none');
fill3(ax, xc(2,:), yc(2,:), zc(2,:), col, 'EdgeColor','none');
end

function drawSphere(ax, c, rad, col)
[xs,ys,zs] = sphere(20);
surf(ax, rad*xs+c(1), rad*ys+c(2), rad*zs+c(3), 'FaceColor',col, 'EdgeColor','none');
end

function drawGroundHatch(ax, p, t, col)
% short ground symbol (line + hatches) on top of the base above each servo
q1 = p - 12*t; q2 = p + 12*t;
plot3(ax, [q1(1) q2(1)], [q1(2) q2(2)], [q1(3) q2(3)], '-', 'Color',col, 'LineWidth',2.5);
for s = -12:4:12
    a = p + s*t; b = a + 3.5*t + [0;0;4];
    plot3(ax, [a(1) b(1)], [a(2) b(2)], [a(3) b(3)], '-', 'Color',col, 'LineWidth',1);
end
end

function linkLabel(ax, p, s)
% link number in a box (white background so it stays readable)
text(ax, p(1),p(2),p(3), s, 'FontSize',13, 'FontWeight','bold', 'Color',[0 0 0], ...
     'HorizontalAlignment','center', 'VerticalAlignment','middle', ...
     'BackgroundColor',[1 1 0.85], 'EdgeColor',[0 0 0], 'LineWidth',1.2, 'Margin',2);
end

function seg = clipLine(px, py, a, b)
% part of segment a-b inside the convex polygon (px,py); [] if outside
n = numel(px); t0 = 0; t1 = 1; d = b - a;
cx = mean(px); cy = mean(py);
for k = 1:n
    p = [px(k) py(k)]; q = [px(mod(k,n)+1) py(mod(k,n)+1)];
    nrm = [q(2)-p(2), -(q(1)-p(1))];
    if dot(nrm, [cx cy]-p) < 0, nrm = -nrm; end      % inward normal
    num = dot(nrm, a - p); den = dot(nrm, d);
    if abs(den) < 1e-12
        if num < 0, seg = []; return; end
    else
        tt = -num/den;
        if den > 0, t0 = max(t0, tt); else, t1 = min(t1, tt); end
    end
end
if t0 >= t1, seg = []; return; end
seg = [a + t0*d; a + t1*d];
end

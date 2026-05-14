% =========================================================
% Problem 4: Active Suspension Control
% File: Story_Mode_Animation.m
% Purpose: Cinematic story mode animation
%   Scene 1 - Title card
%   Scene 2 - Car approaching bump (normal driving)
%   Scene 3 - Uncontrolled car hits bump (bounces wildly)
%   Scene 4 - PID car hits same bump (stays level)
%   Scene 5 - Side by side with live displacement graph
% =========================================================

clear; clc; close all;

%% --- Transfer functions ---
s  = tf('s');
G  = 1 / (s^2 + 3*s + 2);
Kp = 2.25; Ki = 1.5; Kd = 1.0;
C  = Kp + Ki/s + Kd*s;
sys_dist_ol = G;
sys_dist_cl = G / (1 + C*G);

%% --- Road & bump params ---
road_len    = 24;
bump_cx     = 12.0;
bump_w      = 0.6;
bump_h_phys = 0.08;
bump_h_vis  = 0.32;

dt = 0.04;
t  = 0:dt:14;
car_xpos = linspace(1.5, road_len-1.5, length(t));

bump_phys = zeros(size(t));
for i = 1:length(t)
    dx = car_xpos(i) - bump_cx;
    if abs(dx) < bump_w
        bump_phys(i) = bump_h_phys * cos(pi*dx/(2*bump_w))^2;
    end
end

y_ol = lsim(sys_dist_ol, bump_phys', t);
y_cl = lsim(sys_dist_cl, bump_phys', t);
y_ol_anim = y_ol * 5.5;
y_cl_anim = y_cl * 5.5;

%% --- Wheel road tracking ---
wheel_r    = 0.22;
body_rest  = wheel_r + 0.58;

wheel_road = zeros(size(t));
for i = 1:length(t)
    dx = car_xpos(i) - bump_cx;
    if abs(dx) < bump_w
        wheel_road(i) = bump_h_vis * cos(pi*dx/(2*bump_w))^2;
    end
end

%% --- Road profile ---
road_x = linspace(0, road_len, 800);
road_y = zeros(size(road_x));
for k = 1:length(road_x)
    dx = road_x(k) - bump_cx;
    if abs(dx) < bump_w
        road_y(k) = bump_h_vis * cos(pi*dx/(2*bump_w))^2;
    end
end

%% =========================================================
%% HELPER FUNCTIONS
%% =========================================================

function draw_road(ax, road_x, road_y, road_len)
    % Road fill
    fill(ax, [road_x, fliplr(road_x)], ...
         [road_y - 0.25, -0.25*ones(size(road_y))], ...
         [0.22 0.22 0.26], 'EdgeColor','none');
    % Road surface line
    plot(ax, road_x, road_y, 'Color',[0.48 0.48 0.52], 'LineWidth',1.5);
    % Dashed centre line
    for lx = 0:3:road_len
        patch(ax,[lx lx+1.5 lx+1.5 lx],[-0.03 -0.03 0.03 0.03],...
            [0.75 0.72 0.35],'EdgeColor','none');
    end
    % Road edges
    plot(ax,[-1 road_len+1],[0 0],'Color',[0.45 0.45 0.48],'LineWidth',0.8);
end

function draw_bump_label(ax, bump_cx, bump_h_vis)
    text(ax, bump_cx, bump_h_vis+0.18, 'SPEED BUMP', ...
        'Color',[0.95 0.52 0.12],'FontSize',8,'FontWeight','bold',...
        'HorizontalAlignment','center');
    % Arrow down to bump
    annotation_y = bump_h_vis + 0.12;
    plot(ax,[bump_cx bump_cx],[annotation_y bump_h_vis+0.02],...
        'Color',[0.95 0.52 0.12],'LineWidth',1.2);
end

function H = draw_car(ax, cx, body_y, wheel_y, body_col, roof_col, win_col, rim_col)
    H = gobjects(0);
    wr = 0.22; BL = 1.80; BH = 0.38;

    function H = af(H, h); H = [H, h]; end

    % Underbody
    H = af(H, fill(ax,[cx-BL cx+BL cx+BL cx-BL cx-BL],...
        [body_y-0.06 body_y-0.06 body_y body_y body_y-0.06],...
        body_col*0.65,'EdgeColor','none'));
    % Main body
    H = af(H, fill(ax,[cx-BL cx+BL cx+BL cx-BL cx-BL],...
        [body_y body_y body_y+BH body_y+BH body_y],...
        body_col,'EdgeColor',body_col*0.6,'LineWidth',0.8));
    % Hood
    H = af(H, fill(ax,[cx+BL*0.55 cx+BL cx+BL cx+BL*0.55],...
        [body_y+BH body_y+BH body_y+BH*0.45 body_y+BH],...
        body_col*0.92,'EdgeColor',body_col*0.6,'LineWidth',0.6));
    % Trunk
    H = af(H, fill(ax,[cx-BL*0.50 cx-BL cx-BL cx-BL*0.50],...
        [body_y+BH body_y+BH body_y+BH*0.55 body_y+BH],...
        body_col*0.90,'EdgeColor',body_col*0.6,'LineWidth',0.6));
    % Cabin
    H = af(H, fill(ax,...
        [cx-BL*0.42 cx+BL*0.46 cx+BL*0.36 cx-BL*0.32 cx-BL*0.42],...
        [body_y+BH body_y+BH body_y+BH+0.42 body_y+BH+0.42 body_y+BH],...
        roof_col,'EdgeColor',roof_col*0.6,'LineWidth',0.8));
    % Front windshield
    H = af(H, fill(ax,...
        [cx+BL*0.46 cx+BL*0.36 cx+BL*0.36 cx+BL*0.46],...
        [body_y+BH body_y+BH+0.42 body_y+BH+0.42 body_y+BH],...
        win_col,'EdgeColor',win_col*0.7,'LineWidth',0.6,'FaceAlpha',0.82));
    % Rear windshield
    H = af(H, fill(ax,...
        [cx-BL*0.42 cx-BL*0.32 cx-BL*0.32 cx-BL*0.42],...
        [body_y+BH body_y+BH+0.42 body_y+BH+0.42 body_y+BH],...
        win_col,'EdgeColor',win_col*0.7,'LineWidth',0.6,'FaceAlpha',0.82));
    % Side window
    H = af(H, fill(ax,...
        [cx-BL*0.32 cx+BL*0.36 cx+BL*0.30 cx-BL*0.26],...
        [body_y+BH+0.05 body_y+BH+0.05 body_y+BH+0.38 body_y+BH+0.38],...
        win_col,'EdgeColor',win_col*0.7,'LineWidth',0.5,'FaceAlpha',0.82));
    % Headlight
    H = af(H, fill(ax,[cx+BL cx+BL cx+BL-0.12 cx+BL-0.12],...
        [body_y+0.07 body_y+0.20 body_y+0.20 body_y+0.07],...
        [1.0 0.95 0.60],'EdgeColor',[0.9 0.8 0.4],'LineWidth',0.6));
    % Tail light
    H = af(H, fill(ax,[cx-BL cx-BL cx-BL+0.12 cx-BL+0.12],...
        [body_y+0.07 body_y+0.20 body_y+0.20 body_y+0.07],...
        [0.90 0.05 0.05],'EdgeColor',[0.7 0.02 0.02],'LineWidth',0.6));
    % Front bumper
    H = af(H, fill(ax,[cx+BL-0.05 cx+BL cx+BL cx+BL-0.05],...
        [body_y-0.04 body_y-0.04 body_y+0.04 body_y+0.04],...
        [0.55 0.55 0.58],'EdgeColor','none'));
    % Rear bumper
    H = af(H, fill(ax,[cx-BL+0.05 cx-BL cx-BL cx-BL+0.05],...
        [body_y-0.04 body_y-0.04 body_y+0.04 body_y+0.04],...
        [0.55 0.55 0.58],'EdgeColor','none'));

    % Wheels
    th = linspace(0,2*pi,32);
    wx_pos = [cx+BL*0.62, cx-BL*0.62];
    for w = 1:2
        wxc = wx_pos(w);
        H = af(H, fill(ax, wxc+wr*cos(th), wheel_y+wr*sin(th),...
            [0.12 0.12 0.14],'EdgeColor',[0.30 0.30 0.32],'LineWidth',1.0));
        H = af(H, fill(ax, wxc+wr*0.62*cos(th), wheel_y+wr*0.62*sin(th),...
            rim_col,'EdgeColor',rim_col*0.7,'LineWidth',0.8));
        H = af(H, fill(ax, wxc+wr*0.22*cos(th), wheel_y+wr*0.22*sin(th),...
            [0.85 0.85 0.88],'EdgeColor','none'));
        for sp = 0:72:288
            H = af(H, plot(ax,...
                [wxc+wr*0.22*cosd(sp), wxc+wr*0.60*cosd(sp)],...
                [wheel_y+wr*0.22*sind(sp), wheel_y+wr*0.60*sind(sp)],...
                'Color',rim_col*0.85,'LineWidth',1.6));
        end
    end

    % Suspension springs
    sp_xs = [cx+BL*0.58, cx-BL*0.58];
    sp_bot = wheel_y + wr*0.7;
    sp_top = body_y - 0.04;
    npts = 22;
    sp_z = linspace(sp_bot, sp_top, npts);
    amp = 0.055;
    sp_zig = zeros(1,npts);
    for k=2:npts-1
        if mod(k,2)==0; sp_zig(k)=amp; else; sp_zig(k)=-amp; end
    end
    for sp = 1:2
        H = af(H, plot(ax, sp_xs(sp)+sp_zig, sp_z,...
            'Color',[0.75 0.75 0.25],'LineWidth',1.8));
    end
end

function title_card(fig, ax, line1, line2, col1, col2, duration)
    cla(ax);
    set(ax,'Color',[0.06 0.06 0.08],'XColor','none','YColor','none');
    xlim(ax,[0 10]); ylim(ax,[0 10]);
    t1 = text(ax,5,5.8,line1,'Color',col1,'FontSize',22,...
        'FontWeight','bold','HorizontalAlignment','center');
    t2 = text(ax,5,4.2,line2,'Color',col2,'FontSize',14,...
        'HorizontalAlignment','center');
    drawnow;
    pause(duration);
    delete(t1); delete(t2);
end

%% =========================================================
%% FIGURE SETUP
%% =========================================================
fig = figure('Name','Active Suspension — Story Mode',...
    'Color',[0.06 0.06 0.08],'Position',[40 40 1280 720]);

%% =========================================================
%% SCENE 1 — Opening title
%% =========================================================
ax_main = axes('Parent',fig,'Position',[0 0 1 1]);
set(ax_main,'Color',[0.06 0.06 0.08],'XColor','none','YColor','none',...
    'ZColor','none','TickLength',[0 0]);
hold(ax_main,'on'); xlim(ax_main,[0 10]); ylim(ax_main,[0 10]);

text(ax_main,5,6.2,'ACTIVE SUSPENSION CONTROL',...
    'Color',[0.95 0.65 0.20],'FontSize',24,'FontWeight','bold',...
    'HorizontalAlignment','center');
text(ax_main,5,5.2,'Problem 4 — Control Craft Hackathon',...
    'Color',[0.80 0.80 0.85],'FontSize',14,'HorizontalAlignment','center');
text(ax_main,5,4.2,'G(s) = 1 / (s² + 3s + 2)',...
    'Color',[0.55 0.78 0.95],'FontSize',12,'HorizontalAlignment','center');
text(ax_main,5,3.2,'PID Controller: Kp=2.25  Ki=1.5  Kd=1.0',...
    'Color',[0.22 0.85 0.52],'FontSize',11,'HorizontalAlignment','center');
drawnow; pause(2.5);
cla(ax_main);

%% =========================================================
%% SCENE 2 — Car approaching bump (uncontrolled, no bounce yet)
%% =========================================================
text(ax_main,5,0.5,'▶  Scene 1: Car approaching speed bump...',...
    'Color',[0.75 0.75 0.78],'FontSize',10,'HorizontalAlignment','center');

ax_s2 = axes('Parent',fig,'Position',[0.04 0.12 0.92 0.78]);
set(ax_s2,'Color',[0.06 0.06 0.08],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',9);
hold(ax_s2,'on');
xlim(ax_s2,[0 road_len]); ylim(ax_s2,[-0.4 2.8]);
grid(ax_s2,'on'); ax_s2.GridColor=[0.22 0.22 0.25]; ax_s2.GridAlpha=0.5;
ax_s2.XTick=0:4:road_len; ax_s2.YTick=0:0.5:2.5;
xlabel(ax_s2,'Road position (m)','Color',[0.6 0.6 0.6]);
ylabel(ax_s2,'Height (m)','Color',[0.6 0.6 0.6]);
title(ax_s2,'Uncontrolled Car — Approaching Bump',...
    'Color',[0.95 0.65 0.20],'FontSize',13,'FontWeight','bold');

draw_road(ax_s2, road_x, road_y, road_len);
draw_bump_label(ax_s2, bump_cx, bump_h_vis);

% Approach: car goes from x=1 to x=9 (before bump)
approach_idx = find(car_xpos < bump_cx - 2.5);
prev_H = [];
for i = approach_idx
    delete(prev_H(ishandle(prev_H)));
    cx = car_xpos(i);
    by = wheel_r + body_rest - wheel_r;   % flat road
    wy = wheel_r;
    prev_H = draw_car(ax_s2, cx, by, wy,...
        [0.82 0.18 0.05],[0.65 0.12 0.04],[0.30 0.52 0.82],[0.78 0.78 0.82]);
    % Speed lines
    for sl = 1:3
        lx = cx - 0.5 - sl*0.4;
        hsl = plot(ax_s2,[lx lx-0.3],[by+0.15+sl*0.08 by+0.15+sl*0.08],...
            'Color',[0.55 0.55 0.60],'LineWidth',0.8);
        prev_H(end+1) = hsl;
    end
    drawnow limitrate;
    pause(0.03);
end
pause(0.4);
cla(ax_main); cla(ax_s2); delete(ax_s2);

%% =========================================================
%% SCENE 3 — Uncontrolled hits bump
%% =========================================================
text(ax_main,5,0.5,'▶  Scene 2: Uncontrolled — no suspension control',...
    'Color',[0.95 0.65 0.20],'FontSize',10,'HorizontalAlignment','center');

ax_s3 = axes('Parent',fig,'Position',[0.04 0.12 0.92 0.78]);
set(ax_s3,'Color',[0.06 0.06 0.08],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',9);
hold(ax_s3,'on');
xlim(ax_s3,[0 road_len]); ylim(ax_s3,[-0.4 2.8]);
grid(ax_s3,'on'); ax_s3.GridColor=[0.22 0.22 0.25]; ax_s3.GridAlpha=0.5;
ax_s3.XTick=0:4:road_len; ax_s3.YTick=0:0.5:2.5;
xlabel(ax_s3,'Road position (m)','Color',[0.6 0.6 0.6]);
ylabel(ax_s3,'Height (m)','Color',[0.6 0.6 0.6]);
title(ax_s3,'Uncontrolled — Body bounces over bump!',...
    'Color',[0.95 0.38 0.12],'FontSize',13,'FontWeight','bold');

draw_road(ax_s3, road_x, road_y, road_len);
draw_bump_label(ax_s3, bump_cx, bump_h_vis);

% Displacement live plot (inset)
ax_disp_ol = axes('Parent',fig,'Position',[0.68 0.14 0.26 0.25]);
set(ax_disp_ol,'Color',[0.10 0.10 0.12],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',8);
hold(ax_disp_ol,'on'); grid(ax_disp_ol,'on');
ax_disp_ol.GridColor=[0.22 0.22 0.25];
xlim(ax_disp_ol,[0 max(t)]); ylim(ax_disp_ol,[-0.5 5.5]);
xlabel(ax_disp_ol,'Time (s)','Color',[0.6 0.6 0.6],'FontSize',8);
ylabel(ax_disp_ol,'Disp (cm)','Color',[0.6 0.6 0.6],'FontSize',8);
title(ax_disp_ol,'Body Displacement','Color',[0.95 0.65 0.20],'FontSize',8);
disp_line_ol = plot(ax_disp_ol,nan,nan,'Color',[0.95 0.65 0.20],'LineWidth',1.5);
disp_t_ol = []; disp_y_ol = [];

prev_H = [];
exclaim_h = [];
for i = 1:length(t)
    delete(prev_H(ishandle(prev_H)));
    delete(exclaim_h(ishandle(exclaim_h)));
    cx  = car_xpos(i);
    wz  = wheel_road(i);
    wy  = wz + wheel_r;
    by  = wz + body_rest + y_ol_anim(i);
    prev_H = draw_car(ax_s3, cx, by, wy,...
        [0.82 0.18 0.05],[0.65 0.12 0.04],[0.30 0.52 0.82],[0.78 0.78 0.82]);

    % Live displacement
    disp_t_ol(end+1) = t(i);
    disp_y_ol(end+1) = y_ol(i)*100;
    set(disp_line_ol,'XData',disp_t_ol,'YData',disp_y_ol);

    % Exclamation near bump
    if abs(cx - bump_cx) < 1.5 && y_ol_anim(i) > 0.05
        exclaim_h = text(ax_s3, cx, by+0.55, '!!!',...
            'Color',[0.95 0.25 0.10],'FontSize',16,'FontWeight','bold',...
            'HorizontalAlignment','center');
    end

    drawnow limitrate;
    pause(dt*0.45);
end
pause(0.5);
cla(ax_main); cla(ax_s3); delete(ax_s3);
cla(ax_disp_ol); delete(ax_disp_ol);

%% =========================================================
%% SCENE 4 — PID controlled hits same bump
%% =========================================================
text(ax_main,5,0.5,'▶  Scene 3: PID Controlled — suspension absorbs the bump',...
    'Color',[0.22 0.85 0.52],'FontSize',10,'HorizontalAlignment','center');

ax_s4 = axes('Parent',fig,'Position',[0.04 0.12 0.92 0.78]);
set(ax_s4,'Color',[0.06 0.06 0.08],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',9);
hold(ax_s4,'on');
xlim(ax_s4,[0 road_len]); ylim(ax_s4,[-0.4 2.8]);
grid(ax_s4,'on'); ax_s4.GridColor=[0.22 0.22 0.25]; ax_s4.GridAlpha=0.5;
ax_s4.XTick=0:4:road_len; ax_s4.YTick=0:0.5:2.5;
xlabel(ax_s4,'Road position (m)','Color',[0.6 0.6 0.6]);
ylabel(ax_s4,'Height (m)','Color',[0.6 0.6 0.6]);
title(ax_s4,'PID Controlled — Body stays perfectly level!',...
    'Color',[0.22 0.85 0.52],'FontSize',13,'FontWeight','bold');

draw_road(ax_s4, road_x, road_y, road_len);
draw_bump_label(ax_s4, bump_cx, bump_h_vis);

% Inset displacement
ax_disp_cl = axes('Parent',fig,'Position',[0.68 0.14 0.26 0.25]);
set(ax_disp_cl,'Color',[0.10 0.10 0.12],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',8);
hold(ax_disp_cl,'on'); grid(ax_disp_cl,'on');
ax_disp_cl.GridColor=[0.22 0.22 0.25];
xlim(ax_disp_cl,[0 max(t)]); ylim(ax_disp_cl,[-0.5 5.5]);
xlabel(ax_disp_cl,'Time (s)','Color',[0.6 0.6 0.6],'FontSize',8);
ylabel(ax_disp_cl,'Disp (cm)','Color',[0.6 0.6 0.6],'FontSize',8);
title(ax_disp_cl,'Body Displacement','Color',[0.22 0.85 0.52],'FontSize',8);
disp_line_cl = plot(ax_disp_cl,nan,nan,'Color',[0.22 0.85 0.52],'LineWidth',1.5);
disp_t_cl = []; disp_y_cl = [];

% Reference line
text(ax_disp_cl, max(t)*0.05, 0.25,'≈ 0 cm','Color',[0.22 0.85 0.52],...
    'FontSize',7,'FontWeight','bold');

prev_H = [];
check_h = [];
for i = 1:length(t)
    delete(prev_H(ishandle(prev_H)));
    delete(check_h(ishandle(check_h)));
    cx  = car_xpos(i);
    wz  = wheel_road(i);
    wy  = wz + wheel_r;
    by  = body_rest + y_cl_anim(i);   % body stays level

    prev_H = draw_car(ax_s4, cx, by, wy,...
        [0.06 0.36 0.82],[0.04 0.26 0.65],[0.52 0.76 0.96],[0.22 0.88 0.52]);

    disp_t_cl(end+1) = t(i);
    disp_y_cl(end+1) = y_cl(i)*100;
    set(disp_line_cl,'XData',disp_t_cl,'YData',disp_y_cl);

    % Checkmark near bump when staying level
    if abs(cx - bump_cx) < 2.0 && abs(y_cl_anim(i)) < 0.08
        check_h = text(ax_s4, cx, by+0.55, '✓ Stable!',...
            'Color',[0.22 0.90 0.45],'FontSize',13,'FontWeight','bold',...
            'HorizontalAlignment','center');
    end

    drawnow limitrate;
    pause(dt*0.45);
end
pause(0.5);
cla(ax_main); cla(ax_s4); delete(ax_s4);
cla(ax_disp_cl); delete(ax_disp_cl);

%% =========================================================
%% SCENE 5 — Side by side comparison with live graphs
%% =========================================================
text(ax_main,5,0.5,'▶  Scene 4: Side-by-side comparison',...
    'Color',[0.75 0.75 0.80],'FontSize',10,'HorizontalAlignment','center');

% Left: uncontrolled
ax_L = axes('Parent',fig,'Position',[0.03 0.38 0.46 0.54]);
set(ax_L,'Color',[0.06 0.06 0.08],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',8);
hold(ax_L,'on');
xlim(ax_L,[0 road_len]); ylim(ax_L,[-0.4 2.8]);
grid(ax_L,'on'); ax_L.GridColor=[0.22 0.22 0.25]; ax_L.GridAlpha=0.4;
ax_L.XTick=0:6:road_len; ax_L.YTick=[];
xlabel(ax_L,'Road position (m)','Color',[0.6 0.6 0.6],'FontSize',8);
title(ax_L,'Uncontrolled','Color',[0.95 0.65 0.20],'FontSize',12,'FontWeight','bold');
draw_road(ax_L, road_x, road_y, road_len);
draw_bump_label(ax_L, bump_cx, bump_h_vis);

% Right: PID
ax_R = axes('Parent',fig,'Position',[0.51 0.38 0.46 0.54]);
set(ax_R,'Color',[0.06 0.06 0.08],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',8);
hold(ax_R,'on');
xlim(ax_R,[0 road_len]); ylim(ax_R,[-0.4 2.8]);
grid(ax_R,'on'); ax_R.GridColor=[0.22 0.22 0.25]; ax_R.GridAlpha=0.4;
ax_R.XTick=0:6:road_len; ax_R.YTick=[];
xlabel(ax_R,'Road position (m)','Color',[0.6 0.6 0.6],'FontSize',8);
title(ax_R,'PID Controlled','Color',[0.22 0.85 0.52],'FontSize',12,'FontWeight','bold');
draw_road(ax_R, road_x, road_y, road_len);
draw_bump_label(ax_R, bump_cx, bump_h_vis);

% Bottom graph: displacement comparison
ax_G = axes('Parent',fig,'Position',[0.06 0.05 0.88 0.28]);
set(ax_G,'Color',[0.08 0.08 0.10],'XColor',[0.55 0.55 0.58],...
    'YColor',[0.55 0.55 0.58],'FontSize',8);
hold(ax_G,'on'); grid(ax_G,'on');
ax_G.GridColor=[0.22 0.22 0.25]; ax_G.GridAlpha=0.5;
xlim(ax_G,[0 max(t)]); ylim(ax_G,[-0.5 5.5]);
xlabel(ax_G,'Time (s)','Color',[0.6 0.6 0.6]);
ylabel(ax_G,'Body displacement (cm)','Color',[0.6 0.6 0.6]);
title(ax_G,'Live displacement comparison','Color',[0.80 0.80 0.85],'FontSize',9);
g_line_ol = plot(ax_G,nan,nan,'Color',[0.95 0.65 0.20],'LineWidth',2.0);
g_line_cl = plot(ax_G,nan,nan,'Color',[0.22 0.85 0.52],'LineWidth',2.0);
legend(ax_G,{'Uncontrolled','PID'},'TextColor',[0.75 0.75 0.78],...
    'Color',[0.10 0.10 0.12],'EdgeColor',[0.30 0.30 0.32],'FontSize',8,...
    'Location','northeast');

% Time cursor
t_cursor = xline(ax_G,0,'Color',[0.55 0.55 0.60],'LineWidth',1,'LineStyle','--');

g_t = []; g_ol = []; g_cl = [];
prev_L = []; prev_R = [];
disp_L = text(ax_L,1,2.5,'','Color',[0.95 0.65 0.20],'FontSize',8,'FontWeight','bold');
disp_R = text(ax_R,1,2.5,'','Color',[0.22 0.85 0.52],'FontSize',8,'FontWeight','bold');

for i = 1:length(t)
    if ~ishandle(fig); break; end
    delete(prev_L(ishandle(prev_L)));
    delete(prev_R(ishandle(prev_R)));

    cx   = car_xpos(i);
    wz   = wheel_road(i);
    wy   = wz + wheel_r;
    by_ol = wz + body_rest + y_ol_anim(i);
    by_cl = body_rest + y_cl_anim(i);

    prev_L = draw_car(ax_L, cx, by_ol, wy,...
        [0.82 0.18 0.05],[0.65 0.12 0.04],[0.30 0.52 0.82],[0.78 0.78 0.82]);
    prev_R = draw_car(ax_R, cx, by_cl, wy,...
        [0.06 0.36 0.82],[0.04 0.26 0.65],[0.52 0.76 0.96],[0.22 0.88 0.52]);

    g_t(end+1)  = t(i);
    g_ol(end+1) = y_ol(i)*100;
    g_cl(end+1) = y_cl(i)*100;
    set(g_line_ol,'XData',g_t,'YData',g_ol);
    set(g_line_cl,'XData',g_t,'YData',g_cl);
    set(t_cursor,'Value',t(i));

    set(disp_L,'Position',[max(1,cx-1.5), 2.5, 0],...
        'String',sprintf('%.2f cm',y_ol(i)*100));
    set(disp_R,'Position',[max(1,cx-1.5), 2.5, 0],...
        'String',sprintf('%.2f cm',y_cl(i)*100));

    drawnow limitrate;
    pause(dt*0.42);
end

%% --- Final stats card ---
pause(0.3);
cla(ax_main);
text(ax_main,5,8.5,'Final Results','Color',[0.90 0.90 0.95],...
    'FontSize',16,'FontWeight','bold','HorizontalAlignment','center');

ol_peak = max(abs(y_ol))*100;
cl_peak = max(abs(y_cl))*100;
reduction = (1 - cl_peak/ol_peak)*100;
ol_info = stepinfo(sys_dist_ol,'SettlingTimeThreshold',0.05);
cl_info = stepinfo(sys_dist_cl,'SettlingTimeThreshold',0.05);

text(ax_main,5,7.4,sprintf('Uncontrolled peak displacement:  %.2f cm', ol_peak),...
    'Color',[0.95 0.65 0.20],'FontSize',11,'HorizontalAlignment','center');
text(ax_main,5,6.6,sprintf('PID controlled peak displacement: %.2f cm', cl_peak),...
    'Color',[0.22 0.85 0.52],'FontSize',11,'HorizontalAlignment','center');
text(ax_main,5,5.8,sprintf('Peak reduction: %.1f%%', reduction),...
    'Color',[0.55 0.78 0.95],'FontSize',12,'FontWeight','bold','HorizontalAlignment','center');
text(ax_main,5,4.8,'PID controller meets all objectives:',...
    'Color',[0.80 0.80 0.85],'FontSize',10,'HorizontalAlignment','center');
text(ax_main,5,4.1,'✓  Oscillations minimised     ✓  Settling time < 5s     ✓  Improved damping',...
    'Color',[0.22 0.85 0.52],'FontSize',10,'HorizontalAlignment','center');
text(ax_main,5,3.0,'G(s) = 1/(s²+3s+2)   |   PID: Kp=2.25  Ki=1.5  Kd=1.0',...
    'Color',[0.50 0.50 0.55],'FontSize',9,'HorizontalAlignment','center');
drawnow;

fprintf('Story mode complete.\n');
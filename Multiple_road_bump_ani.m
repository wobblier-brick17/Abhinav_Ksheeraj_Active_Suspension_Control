% =========================================================
% Problem 4: Active Suspension Control
% File: Multi_Bump_Animation.m
% Purpose: 2D animation of car driving over 3 road bumps
%          of increasing size (5cm, 10cm, 20cm).
%          Left = uncontrolled, Right = PID controlled.
%          Body stays fixed (x) on PID side — only suspension moves.
% =========================================================

clear; clc; close all;

%% --- Transfer functions ---
s  = tf('s');
G  = 1 / (s^2 + 3*s + 2);

Kp = 2.25; Ki = 1.5; Kd = 1.0;
C  = Kp + Ki/s + Kd*s;

sys_dist_ol = G / (1 + 0);
sys_dist_cl = G / (1 + C*G);

%% --- Bump positions and sizes ---
bump_centres = [3.0,  7.0,  12.0];
bump_heights = [0.05, 0.10, 0.20];
bump_labels  = {'5 cm','10 cm','20 cm'};
bump_w       = 0.35;

%% --- Road profile ---
road_len = 16;
road_x   = linspace(0, road_len, 1000);
road_y   = zeros(size(road_x));
for b = 1:3
    for k = 1:length(road_x)
        dx = road_x(k) - bump_centres(b);
        if abs(dx) < bump_w
            road_y(k) = road_y(k) + bump_heights(b) * cos(pi*dx/(2*bump_w))^2;
        end
    end
end

%% --- Simulate ---
dt = 0.04;
t  = 0:dt:20;

car_x_start = 0.5;
car_x_end   = road_len - 0.5;
car_xpos    = linspace(car_x_start, car_x_end, length(t));

road_interp = @(x) interp1(road_x, road_y, min(max(x,road_x(1)),road_x(end)));
u_dist = arrayfun(road_interp, car_xpos)';

y_ol = lsim(sys_dist_ol, u_dist, t);
y_cl = lsim(sys_dist_cl, u_dist, t);

vis_scale = 1.5;
y_ol_anim = y_ol * vis_scale;
y_cl_anim = y_cl * vis_scale;

%% --- Wheel ground contact ---
wheel_r    = 0.20;
chassis_h  = 0.22;
spring_nat = 0.32;

wheel_y_road = zeros(size(t));
for i = 1:length(t)
    wheel_y_road(i) = wheel_r + road_interp(car_xpos(i));
end

%% --- Figure ---
fig = figure('Name','Multi-Bump Active Suspension Animation',...
    'Color',[0.10 0.10 0.12],'Position',[60 60 1280 680]);

ax1 = subplot(1,2,1);
set(ax1,'Color',[0.10 0.10 0.12],'XColor',[0.7 0.7 0.7],...
    'YColor',[0.7 0.7 0.7],'FontSize',10);
hold(ax1,'on'); axis(ax1,'equal');
xlim(ax1,[0 road_len]); ylim(ax1,[-0.4 2.8]);
title(ax1,'Uncontrolled — 3 bump road',...
    'Color',[0.95 0.65 0.2],'FontSize',12,'FontWeight','bold');
xlabel(ax1,'Road position (m)','Color',[0.7 0.7 0.7]);
ylabel(ax1,'Height (m)','Color',[0.7 0.7 0.7]);
grid(ax1,'on'); ax1.GridColor=[0.28 0.28 0.28]; ax1.GridAlpha=0.4;

ax2 = subplot(1,2,2);
set(ax2,'Color',[0.10 0.10 0.12],'XColor',[0.7 0.7 0.7],...
    'YColor',[0.7 0.7 0.7],'FontSize',10);
hold(ax2,'on'); axis(ax2,'equal');
xlim(ax2,[0 road_len]); ylim(ax2,[-0.4 2.8]);
title(ax2,'PID Controlled — body stays level',...
    'Color',[0.25 0.85 0.55],'FontSize',12,'FontWeight','bold');
xlabel(ax2,'Road position (m)','Color',[0.7 0.7 0.7]);
ylabel(ax2,'Height (m)','Color',[0.7 0.7 0.7]);
grid(ax2,'on'); ax2.GridColor=[0.28 0.28 0.28]; ax2.GridAlpha=0.4;

%% Draw road
road_fill_y = [road_y - 0.25, -0.25*ones(size(road_y))];
road_fill_x = [road_x, fliplr(road_x)];

fill(ax1, road_fill_x, road_fill_y, [0.22 0.22 0.26], 'EdgeColor','none');
plot(ax1, road_x, road_y, 'Color',[0.52 0.52 0.56], 'LineWidth',1.8);
plot(ax1, [0 road_len],[0 0],'Color',[0.4 0.4 0.44],'LineWidth',0.8);

fill(ax2, road_fill_x, road_fill_y, [0.22 0.22 0.26], 'EdgeColor','none');
plot(ax2, road_x, road_y, 'Color',[0.52 0.52 0.56], 'LineWidth',1.8);
plot(ax2, [0 road_len],[0 0],'Color',[0.4 0.4 0.44],'LineWidth',0.8);

%% Bump labels
for b = 1:3
    bh = bump_heights(b);
    text(ax1, bump_centres(b), bh+0.18, bump_labels{b}, ...
        'Color',[0.95 0.55 0.15],'FontSize',8,'FontWeight','bold',...
        'HorizontalAlignment','center');
    text(ax2, bump_centres(b), bh+0.18, bump_labels{b}, ...
        'Color',[0.95 0.55 0.15],'FontSize',8,'FontWeight','bold',...
        'HorizontalAlignment','center');
end

%% Car dimensions
car_w     = 1.8;
car_h     = 0.44;
car_x_fix = road_len / 2;   % PID body fixed x = centre of road

%% --- Init graphics ---

% UNCONTROLLED
wh1_ol = rectangle(ax1,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.20 0.20 0.23],'EdgeColor',[0.72 0.72 0.72],'LineWidth',1.2);
wh2_ol = rectangle(ax1,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.20 0.20 0.23],'EdgeColor',[0.72 0.72 0.72],'LineWidth',1.2);
chas_ol  = fill(ax1,[0 1 1 0],[0 0 1 1],[0.32 0.32 0.36],'EdgeColor','none');
sp1_ol   = plot(ax1,[0 0],[0 1],'Color',[0.92 0.62 0.18],'LineWidth',1.8);
sp2_ol   = plot(ax1,[0 0],[0 1],'Color',[0.92 0.62 0.18],'LineWidth',1.8);
body_ol  = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.82 0.40 0.10],'EdgeColor',[0.95 0.55 0.15],'LineWidth',1.5);
roof_ol  = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.70 0.32 0.08],'EdgeColor',[0.95 0.55 0.15],'LineWidth',1.0);
win_ol   = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.32 0.52 0.74],'EdgeColor',[0.5 0.7 0.9],'LineWidth',0.8);
dtxt_ol  = text(ax1, 1, 2.55,'','Color',[0.95 0.65 0.2],'FontSize',9,'FontWeight','bold');
bump_ind_ol = text(ax1, road_len/2, 2.35,'','Color',[0.95 0.45 0.15],...
    'FontSize',10,'FontWeight','bold','HorizontalAlignment','center');

% PID CONTROLLED
wh1_cl = rectangle(ax2,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.20 0.20 0.23],'EdgeColor',[0.72 0.72 0.72],'LineWidth',1.2);
wh2_cl = rectangle(ax2,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.20 0.20 0.23],'EdgeColor',[0.72 0.72 0.72],'LineWidth',1.2);
chas_cl  = fill(ax2,[0 1 1 0],[0 0 1 1],[0.32 0.32 0.36],'EdgeColor','none');
sp1_cl   = plot(ax2,[0 0],[0 1],'Color',[0.22 0.88 0.52],'LineWidth',1.8);
sp2_cl   = plot(ax2,[0 0],[0 1],'Color',[0.22 0.88 0.52],'LineWidth',1.8);
body_cl  = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.10 0.42 0.76],'EdgeColor',[0.18 0.62 0.96],'LineWidth',1.5);
roof_cl  = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.06 0.32 0.64],'EdgeColor',[0.18 0.62 0.96],'LineWidth',1.0);
win_cl   = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.52 0.76 0.95],'EdgeColor',[0.7 0.9 1.0],'LineWidth',0.8);
dtxt_cl  = text(ax2, car_x_fix-0.6, 2.55,'','Color',[0.22 0.88 0.52],'FontSize',9,'FontWeight','bold');
bump_ind_cl = text(ax2, road_len/2, 2.35,'','Color',[0.22 0.88 0.52],...
    'FontSize',10,'FontWeight','bold','HorizontalAlignment','center');

ttxt = annotation(fig,'textbox',[0.43 0.93 0.14 0.055],'String','t = 0.00 s',...
    'Color',[0.88 0.88 0.88],'FontSize',11,'FontWeight','bold',...
    'EdgeColor','none','BackgroundColor','none','HorizontalAlignment','center');

%% Spring generator
function [sx, sy] = make_spring(xc, y_bot, y_top, n_coils)
    n_pts  = n_coils * 4 + 2;
    sy_raw = linspace(y_bot, y_top, n_pts);
    sx_raw = zeros(1, n_pts);
    amp    = 0.07;
    for k = 2:n_pts-1
        if mod(k,2)==0; sx_raw(k) =  amp; else; sx_raw(k) = -amp; end
    end
    sx = xc + sx_raw;
    sy = sy_raw;
end

%% --- Animation loop ---
fprintf('Running multi-bump animation... Close window to stop.\n');

for i = 1:length(t)
    if ~ishandle(fig); break; end

    cx   = car_xpos(i);
    wy   = wheel_y_road(i);
    d_ol = y_ol_anim(i);
    d_cl = y_cl_anim(i);

    by_ol = wy + chassis_h + spring_nat + d_ol;
    by_cl = wy + chassis_h + spring_nat + d_cl;

    wf = cx + car_w*0.30;
    wr = cx - car_w*0.30;

    % Bump indicator
    bump_str = '';
    for b = 1:3
        if abs(cx - bump_centres(b)) < bump_w*3
            bump_str = sprintf('Bump %d: %s', b, bump_labels{b});
        end
    end

    %% UNCONTROLLED — everything travels with car
    set(wh1_ol,'Position',[wf-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(wh2_ol,'Position',[wr-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(chas_ol,'XData',[wr wf wf wr],...
        'YData',[wy+wheel_r*0.5, wy+wheel_r*0.5,...
                 wy+wheel_r*0.5+chassis_h, wy+wheel_r*0.5+chassis_h]);
    [sx1,sy1] = make_spring(wf, wy+wheel_r*0.5+chassis_h, by_ol, 5);
    [sx2,sy2] = make_spring(wr, wy+wheel_r*0.5+chassis_h, by_ol, 5);
    set(sp1_ol,'XData',sx1,'YData',sy1);
    set(sp2_ol,'XData',sx2,'YData',sy2);
    set(body_ol,'XData',[cx-car_w/2, cx+car_w/2, cx+car_w/2, cx-car_w/2, cx-car_w/2],...
        'YData',[by_ol, by_ol, by_ol+car_h, by_ol+car_h, by_ol]);
    set(roof_ol,'XData',[cx-car_w*0.34,cx+car_w*0.34,cx+car_w*0.30,cx-car_w*0.30,cx-car_w*0.34],...
        'YData',[by_ol+car_h,by_ol+car_h,by_ol+car_h+0.30,by_ol+car_h+0.30,by_ol+car_h]);
    set(win_ol,'XData',[cx-car_w*0.30,cx+car_w*0.30,cx+car_w*0.26,cx-car_w*0.26,cx-car_w*0.30],...
        'YData',[by_ol+car_h+0.03,by_ol+car_h+0.03,by_ol+car_h+0.26,by_ol+car_h+0.26,by_ol+car_h+0.03]);
    set(dtxt_ol,'Position',[cx-0.6, 2.55, 0],...
        'String',sprintf('disp: %.1f cm', y_ol(i)*100));
    set(bump_ind_ol,'String',bump_str);

    %% PID — wheels+chassis travel, BODY FIXED AT car_x_fix
    set(wh1_cl,'Position',[wf-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(wh2_cl,'Position',[wr-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(chas_cl,'XData',[wr wf wf wr],...
        'YData',[wy+wheel_r*0.5, wy+wheel_r*0.5,...
                 wy+wheel_r*0.5+chassis_h, wy+wheel_r*0.5+chassis_h]);
    % Springs connect chassis to body — all use cx, body travels with car
    [sx1,sy1] = make_spring(wf, wy+wheel_r*0.5+chassis_h, by_cl, 5);
    [sx2,sy2] = make_spring(wr, wy+wheel_r*0.5+chassis_h, by_cl, 5);
    set(sp1_cl,'XData',sx1,'YData',sy1);
    set(sp2_cl,'XData',sx2,'YData',sy2);
    % Body moves with car horizontally (cx), but y uses constant baseline + tiny d_cl only
    % This means body doesn't rise with road bump — suspension absorbs it
    by_cl_body = wheel_r + chassis_h + spring_nat + d_cl;
    set(body_cl,'XData',[cx-car_w/2, cx+car_w/2, cx+car_w/2, cx-car_w/2, cx-car_w/2],...
        'YData',[by_cl_body, by_cl_body, by_cl_body+car_h, by_cl_body+car_h, by_cl_body]);
    set(roof_cl,'XData',[cx-car_w*0.34,cx+car_w*0.34,cx+car_w*0.30,cx-car_w*0.30,cx-car_w*0.34],...
        'YData',[by_cl_body+car_h,by_cl_body+car_h,by_cl_body+car_h+0.30,by_cl_body+car_h+0.30,by_cl_body+car_h]);
    set(win_cl,'XData',[cx-car_w*0.30,cx+car_w*0.30,cx+car_w*0.26,cx-car_w*0.26,cx-car_w*0.30],...
        'YData',[by_cl_body+car_h+0.03,by_cl_body+car_h+0.03,by_cl_body+car_h+0.26,by_cl_body+car_h+0.26,by_cl_body+car_h+0.03]);
    set(dtxt_cl,'Position',[cx-0.6, 2.55, 0],...
        'String',sprintf('disp: %.1f cm', y_cl(i)*100));
    set(bump_ind_cl,'String',bump_str);

    set(ttxt,'String',sprintf('t = %.2f s', t(i)));
    drawnow limitrate;
    pause(dt * 0.55);
end

fprintf('Animation complete.\n');
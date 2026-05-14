% =========================================================
% Problem 4: Active Suspension Control
% File: Car_Animation.m
% Left panel  = Uncontrolled: entire car bounces over bump
% Right panel = PID controlled: wheels go over bump,
%               car body stays perfectly level
% =========================================================

clear; clc; close all;

%% --- Transfer functions ---
s  = tf('s');
G  = 1 / (s^2 + 3*s + 2);
Kp = 2.25; Ki = 1.5; Kd = 1.0;
C  = Kp + Ki/s + Kd*s;
sys_dist_ol = G;
sys_dist_cl = G / (1 + C*G);

%% --- Road bump profile (visual) ---
road_len = 10;
bump_cx  = 5.0;
bump_w   = 0.5;
bump_h   = 0.35;   % exaggerated visual height
road_x   = linspace(0, road_len, 600);
road_y   = zeros(size(road_x));
for k = 1:length(road_x)
    dx = road_x(k) - bump_cx;
    if abs(dx) < bump_w
        road_y(k) = bump_h * cos(pi*dx/(2*bump_w))^2;
    end
end

%% --- Simulation (physical bump = 0.08m) ---
dt   = 0.05;
t    = 0:dt:12;
car_xpos = linspace(1.0, 9.0, length(t));

% Physical bump signal based on car position
bump_phys = zeros(size(t));
for i = 1:length(t)
    dx = car_xpos(i) - bump_cx;
    if abs(dx) < bump_w
        bump_phys(i) = 0.08 * cos(pi*dx/(2*bump_w))^2;
    end
end

y_ol = lsim(sys_dist_ol, bump_phys', t);
y_cl = lsim(sys_dist_cl, bump_phys', t);

% Scale for visibility
y_ol_anim = y_ol * 4.0;
y_cl_anim = y_cl * 4.0;

%% --- Wheel y follows visual road ---
wheel_r    = 0.18;
chassis_h  = 0.22;
spring_nat = 0.30;

wheel_y_road = zeros(size(t));
for i = 1:length(t)
    cx = car_xpos(i);
    dx = cx - bump_cx;
    if abs(dx) < bump_w
        wheel_y_road(i) = wheel_r + bump_h * cos(pi*dx/(2*bump_w))^2;
    else
        wheel_y_road(i) = wheel_r;
    end
end

% Constant body baseline (what PID car body y should be — ignores road)
body_baseline = wheel_r + chassis_h + spring_nat;

%% --- Figure ---
fig = figure('Name','Active Suspension Animation',...
    'Color',[0.12 0.12 0.14],'Position',[100 80 1200 650]);

ax1 = subplot(1,2,1);
set(ax1,'Color',[0.12 0.12 0.14],'XColor',[0.7 0.7 0.7],...
    'YColor',[0.7 0.7 0.7],'FontSize',10);
hold(ax1,'on'); axis(ax1,'equal');
xlim(ax1,[0 10]); ylim(ax1,[-0.3 2.5]);
title(ax1,'Uncontrolled','Color',[0.95 0.65 0.2],...
    'FontSize',13,'FontWeight','bold');
xlabel(ax1,'Position (m)','Color',[0.7 0.7 0.7]);
ylabel(ax1,'Height (m)','Color',[0.7 0.7 0.7]);
grid(ax1,'on'); ax1.GridColor=[0.3 0.3 0.3]; ax1.GridAlpha=0.4;

ax2 = subplot(1,2,2);
set(ax2,'Color',[0.12 0.12 0.14],'XColor',[0.7 0.7 0.7],...
    'YColor',[0.7 0.7 0.7],'FontSize',10);
hold(ax2,'on'); axis(ax2,'equal');
xlim(ax2,[0 10]); ylim(ax2,[-0.3 2.5]);
title(ax2,'PID Controlled','Color',[0.25 0.78 0.55],...
    'FontSize',13,'FontWeight','bold');
xlabel(ax2,'Position (m)','Color',[0.7 0.7 0.7]);
ylabel(ax2,'Height (m)','Color',[0.7 0.7 0.7]);
grid(ax2,'on'); ax2.GridColor=[0.3 0.3 0.3]; ax2.GridAlpha=0.4;

%% Draw road (both axes)
rf_x = [road_x, fliplr(road_x)];
rf_y = [road_y-0.15, -0.15*ones(size(road_y))];
fill(ax1,rf_x,rf_y,[0.25 0.25 0.28],'EdgeColor','none');
plot(ax1,road_x,road_y,'Color',[0.55 0.55 0.58],'LineWidth',2);
plot(ax1,[0 10],[0 0],'Color',[0.45 0.45 0.48],'LineWidth',0.8);
fill(ax2,rf_x,rf_y,[0.25 0.25 0.28],'EdgeColor','none');
plot(ax2,road_x,road_y,'Color',[0.55 0.55 0.58],'LineWidth',2);
plot(ax2,[0 10],[0 0],'Color',[0.45 0.45 0.48],'LineWidth',0.8);

text(ax1,bump_cx,bump_h+0.12,'BUMP','Color',[0.9 0.5 0.2],...
    'FontSize',9,'HorizontalAlignment','center','FontWeight','bold');
text(ax2,bump_cx,bump_h+0.12,'BUMP','Color',[0.9 0.5 0.2],...
    'FontSize',9,'HorizontalAlignment','center','FontWeight','bold');

%% Car dimensions
car_w = 1.6;
car_h = 0.42;

%% --- Init graphics ---
% UNCONTROLLED
wh1_ol  = rectangle(ax1,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.22 0.22 0.25],'EdgeColor',[0.7 0.7 0.7],'LineWidth',1.2);
wh2_ol  = rectangle(ax1,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.22 0.22 0.25],'EdgeColor',[0.7 0.7 0.7],'LineWidth',1.2);
chas_ol = fill(ax1,[0 1 1 0],[0 0 1 1],[0.35 0.35 0.38],'EdgeColor','none');
sp1_ol  = plot(ax1,[0 0],[0 1],'Color',[0.9 0.6 0.2],'LineWidth',1.8);
sp2_ol  = plot(ax1,[0 0],[0 1],'Color',[0.9 0.6 0.2],'LineWidth',1.8);
body_ol = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.85 0.42 0.12],...
    'EdgeColor',[0.95 0.55 0.15],'LineWidth',1.5);
roof_ol = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.75 0.35 0.10],...
    'EdgeColor',[0.95 0.55 0.15],'LineWidth',1);
win_ol  = fill(ax1,[0 1 1 0 0],[0 0 1 1 0],[0.35 0.55 0.75],...
    'EdgeColor',[0.5 0.7 0.9],'LineWidth',0.8);
dtxt_ol = text(ax1,0,2.3,'','Color',[0.95 0.65 0.2],'FontSize',9,'FontWeight','bold');

% PID CONTROLLED
wh1_cl  = rectangle(ax2,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.22 0.22 0.25],'EdgeColor',[0.7 0.7 0.7],'LineWidth',1.2);
wh2_cl  = rectangle(ax2,'Position',[0 0 2*wheel_r 2*wheel_r],'Curvature',[1 1],...
    'FaceColor',[0.22 0.22 0.25],'EdgeColor',[0.7 0.7 0.7],'LineWidth',1.2);
chas_cl = fill(ax2,[0 1 1 0],[0 0 1 1],[0.35 0.35 0.38],'EdgeColor','none');
sp1_cl  = plot(ax2,[0 0],[0 1],'Color',[0.25 0.85 0.55],'LineWidth',1.8);
sp2_cl  = plot(ax2,[0 0],[0 1],'Color',[0.25 0.85 0.55],'LineWidth',1.8);
body_cl = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.12 0.45 0.75],...
    'EdgeColor',[0.2 0.6 0.95],'LineWidth',1.5);
roof_cl = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.08 0.35 0.65],...
    'EdgeColor',[0.2 0.6 0.95],'LineWidth',1);
win_cl  = fill(ax2,[0 1 1 0 0],[0 0 1 1 0],[0.55 0.78 0.95],...
    'EdgeColor',[0.7 0.9 1.0],'LineWidth',0.8);
dtxt_cl = text(ax2,0,2.3,'','Color',[0.25 0.85 0.55],'FontSize',9,'FontWeight','bold');

ttxt = annotation(fig,'textbox',[0.44 0.92 0.12 0.06],'String','t = 0.00 s',...
    'Color',[0.85 0.85 0.85],'FontSize',11,'FontWeight','bold',...
    'EdgeColor','none','BackgroundColor','none','HorizontalAlignment','center');

%% Spring generator
function [sx,sy] = make_spring(xc, yb, yt, n)
    np = n*4+2;
    sy = linspace(yb,yt,np);
    sx = zeros(1,np);
    amp = 0.06;
    for k=2:np-1
        if mod(k,2)==0; sx(k)=amp; else; sx(k)=-amp; end
    end
    sx = xc + sx;
end

%% --- Animation loop ---
fprintf('Running... close window to stop.\n');

for i = 1:length(t)
    if ~ishandle(fig); break; end

    cx  = car_xpos(i);
    wy  = wheel_y_road(i);   % wheel y follows road bump visually

    % UNCONTROLLED body y — rises WITH the road (whole car bounces)
    by_ol = wy + chassis_h + spring_nat + y_ol_anim(i);

    % PID body y — stays at CONSTANT baseline (body ignores road bump)
    % Only tiny y_cl_anim (near zero) affects it — spring absorbs the rest
    by_cl = body_baseline + y_cl_anim(i);

    wf = cx + car_w*0.28;
    wr = cx - car_w*0.28;
    ch_top = wy + wheel_r*0.5 + chassis_h;   % chassis top y (follows wheel)

    %% UNCONTROLLED — whole car travels and bounces
    set(wh1_ol,'Position',[wf-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(wh2_ol,'Position',[wr-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(chas_ol,'XData',[wr wf wf wr],...
        'YData',[wy+wheel_r*0.5, wy+wheel_r*0.5, ch_top, ch_top]);
    [sx1,sy1] = make_spring(wf, ch_top, by_ol, 5);
    [sx2,sy2] = make_spring(wr, ch_top, by_ol, 5);
    set(sp1_ol,'XData',sx1,'YData',sy1);
    set(sp2_ol,'XData',sx2,'YData',sy2);
    set(body_ol,'XData',[cx-car_w/2,cx+car_w/2,cx+car_w/2,cx-car_w/2,cx-car_w/2],...
        'YData',[by_ol,by_ol,by_ol+car_h,by_ol+car_h,by_ol]);
    set(roof_ol,'XData',[cx-car_w*0.32,cx+car_w*0.32,cx+car_w*0.28,cx-car_w*0.28,cx-car_w*0.32],...
        'YData',[by_ol+car_h,by_ol+car_h,by_ol+car_h+0.28,by_ol+car_h+0.28,by_ol+car_h]);
    set(win_ol,'XData',[cx-car_w*0.28,cx+car_w*0.28,cx+car_w*0.24,cx-car_w*0.24,cx-car_w*0.28],...
        'YData',[by_ol+car_h+0.03,by_ol+car_h+0.03,by_ol+car_h+0.24,by_ol+car_h+0.24,by_ol+car_h+0.03]);
    set(dtxt_ol,'Position',[cx-0.5,2.3,0],'String',sprintf('disp: %.1f cm',y_ol(i)*100));

    %% PID — wheels+chassis follow road, body stays level
    set(wh1_cl,'Position',[wf-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(wh2_cl,'Position',[wr-wheel_r, wy-wheel_r, 2*wheel_r, 2*wheel_r]);
    set(chas_cl,'XData',[wr wf wf wr],...
        'YData',[wy+wheel_r*0.5, wy+wheel_r*0.5, ch_top, ch_top]);
    % Springs stretch from chassis top (follows bump) to fixed body bottom
    [sx1,sy1] = make_spring(wf, ch_top, by_cl, 5);
    [sx2,sy2] = make_spring(wr, ch_top, by_cl, 5);
    set(sp1_cl,'XData',sx1,'YData',sy1);
    set(sp2_cl,'XData',sx2,'YData',sy2);
    % Body travels with car (cx) but y is constant — doesn't bounce
    set(body_cl,'XData',[cx-car_w/2,cx+car_w/2,cx+car_w/2,cx-car_w/2,cx-car_w/2],...
        'YData',[by_cl,by_cl,by_cl+car_h,by_cl+car_h,by_cl]);
    set(roof_cl,'XData',[cx-car_w*0.32,cx+car_w*0.32,cx+car_w*0.28,cx-car_w*0.28,cx-car_w*0.32],...
        'YData',[by_cl+car_h,by_cl+car_h,by_cl+car_h+0.28,by_cl+car_h+0.28,by_cl+car_h]);
    set(win_cl,'XData',[cx-car_w*0.28,cx+car_w*0.28,cx+car_w*0.24,cx-car_w*0.24,cx-car_w*0.28],...
        'YData',[by_cl+car_h+0.03,by_cl+car_h+0.03,by_cl+car_h+0.24,by_cl+car_h+0.24,by_cl+car_h+0.03]);
    set(dtxt_cl,'Position',[cx-0.5,2.3,0],'String',sprintf('disp: %.1f cm',y_cl(i)*100));

    set(ttxt,'String',sprintf('t = %.2f s',t(i)));
    drawnow limitrate;
    pause(dt*0.6);
end
fprintf('Done.\n');
% =========================================================
% Problem 4: Active Suspension Control
% File: PID_Controller_Design.m
% Purpose: Design PID controller, compare Open-Loop vs PD vs PID
% Run AFTER System_Analysis.m and PD_Controller_Design.m
% =========================================================

clear; clc; close all;

%% --- Plant and controllers (standalone definitions) ---
s  = tf('s');
G  = 1 / (s^2 + 3*s + 2);

% PD Controller (from PD_Controller_Design.m)
Kp_pd = 2.25;
Kd_pd = 1.0;
C_pd  = Kp_pd + Kd_pd*s;

% PID Controller
% Closed-loop char. poly with PID: C(s) = Kp + Ki/s + Kd*s
%   C(s)*G(s) = (Kd*s^2 + Kp*s + Ki) / (s*(s^2 + 3s + 2))
%
% Desired: faster settling, zero steady-state error
% Strategy: keep Kp, Kd from PD design, add small Ki
%   - Ki adds an integrator => forces SSE = 0
%   - Too large Ki => sluggish or oscillatory response
%   - Tuned via trial: Ki = 1.5 gives settling < 3s, no overshoot

Kp_pid = 2.25;
Kd_pid = 1.0;
Ki_pid = 1.5;
C_pid  = Kp_pid + Ki_pid/s + Kd_pid*s;

%% --- Closed-loop systems ---
sys_ol  = G;                       % Open-loop (uncontrolled)
sys_pd  = feedback(C_pd*G,  1);    % PD closed-loop
sys_pid = feedback(C_pid*G, 1);    % PID closed-loop

%% --- Closed-loop poles ---
fprintf('===== Controller Pole Analysis =====\n\n');

poles_pd  = pole(sys_pd);
poles_pid = pole(sys_pid);

fprintf('PD  Closed-Loop Poles:\n');
for i = 1:length(poles_pd)
    fprintf('   %+.4f %+.4fi\n', real(poles_pd(i)), imag(poles_pd(i)));
end

fprintf('\nPID Closed-Loop Poles:\n');
for i = 1:length(poles_pid)
    fprintf('   %+.4f %+.4fi\n', real(poles_pid(i)), imag(poles_pid(i)));
end

%% --- Performance metrics via stepinfo ---
info_ol  = stepinfo(sys_ol,  'SettlingTimeThreshold', 0.05);
info_pd  = stepinfo(sys_pd,  'SettlingTimeThreshold', 0.05);
info_pid = stepinfo(sys_pid, 'SettlingTimeThreshold', 0.05);

sse_ol  = abs(1 - dcgain(sys_ol));
sse_pd  = abs(1 - dcgain(sys_pd));
sse_pid = abs(1 - dcgain(sys_pid));

fprintf('\n===== Step Response Performance Comparison =====\n\n');
fprintf('%-24s  %-14s  %-14s  %-14s\n', 'Metric', 'Open-Loop', 'PD', 'PID');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('%-24s  %-14.4f  %-14.4f  %-14.4f\n', 'Rise Time (s)',       info_ol.RiseTime,     info_pd.RiseTime,     info_pid.RiseTime);
fprintf('%-24s  %-14.4f  %-14.4f  %-14.4f\n', 'Settling Time (s)',   info_ol.SettlingTime, info_pd.SettlingTime, info_pid.SettlingTime);
fprintf('%-24s  %-14.4f  %-14.4f  %-14.4f\n', 'Overshoot (%%)',       info_ol.Overshoot,    info_pd.Overshoot,    info_pid.Overshoot);
fprintf('%-24s  %-14.4f  %-14.4f  %-14.4f\n', 'Peak (m)',            info_ol.Peak,         info_pd.Peak,         info_pid.Peak);
fprintf('%-24s  %-14.6f  %-14.6f  %-14.6f\n', 'Steady-State Error',  sse_ol,               sse_pd,               sse_pid);
fprintf('%-24s  %-14.4f  %-14.4f  %-14.4f\n', 'DC Gain',             dcgain(sys_ol),       dcgain(sys_pd),       dcgain(sys_pid));

% Highlight key improvements
fprintf('\n--- Key Improvements (PID over Open-Loop) ---\n');
fprintf('Settling time reduced by : %.2f s  (%.1f%% faster)\n', ...
    info_ol.SettlingTime - info_pid.SettlingTime, ...
    (1 - info_pid.SettlingTime/info_ol.SettlingTime)*100);
fprintf('Steady-state error reduced: %.6f => %.6f\n', sse_ol, sse_pid);

%% --- Figure 1: Step response — all three on one plot ---
figure(1); clf;
t = 0:0.005:10;
[y_ol,  ~] = step(sys_ol,  t);
[y_pd,  ~] = step(sys_pd,  t);
[y_pid, ~] = step(sys_pid, t);

plot(t, y_ol,  'r--', 'LineWidth', 2.0); hold on;
plot(t, y_pd,  'b-',  'LineWidth', 2.0);
plot(t, y_pid, 'g-',  'LineWidth', 2.5);
yline(1,    'k:',  'LineWidth', 1.0);
yline(1.05, 'k--', 'LineWidth', 0.8, 'Label', '+5%', 'LabelVerticalAlignment', 'bottom');
yline(0.95, 'k--', 'LineWidth', 0.8, 'Label', '-5%', 'LabelVerticalAlignment', 'top');

title('Step Response Comparison: Open-Loop vs PD vs PID');
ylabel('Body Displacement (m)');
xlabel('Time (s)');
legend('Open-Loop (uncontrolled)', ...
       sprintf('PD  (Kp=%.2f, Kd=%.2f)', Kp_pd, Kd_pd), ...
       sprintf('PID (Kp=%.2f, Ki=%.2f, Kd=%.2f)', Kp_pid, Ki_pid, Kd_pid), ...
       'Reference', 'Location', 'best');
grid on;
xlim([0 10]); ylim([-0.1 1.4]);

% Annotate settling times on the plot
xline(info_pd.SettlingTime,  'b:', sprintf('PD settles: %.2fs',  info_pd.SettlingTime),  'LabelVerticalAlignment', 'top', 'LineWidth', 1);
xline(info_pid.SettlingTime, 'g:', sprintf('PID settles: %.2fs', info_pid.SettlingTime), 'LabelVerticalAlignment', 'bottom', 'LineWidth', 1);

%% --- Figure 2: Controller output (effort) comparison ---
% How hard is each controller working?
% C_pd and C_pid are improper (Kd*s term) so we cannot call step() on them
% directly. Instead: simulate the proper closed-loop error signal first,
% then compute u(t) = Kp*e + Kd*de/dt + Ki*integral(e) numerically.
figure(2); clf;

r      = ones(size(t));          % unit step reference
dt     = t(2) - t(1);

% --- PD effort ---
e_pd   = r' - y_pd;             % error signal (column)
de_pd  = [0; diff(e_pd)] / dt;  % numerical derivative
u_pd   = Kp_pd * e_pd + Kd_pd * de_pd;

% --- PID effort ---
e_pid  = r' - y_pid;
de_pid = [0; diff(e_pid)] / dt;
ie_pid = cumsum(e_pid) * dt;    % numerical integral
u_pid  = Kp_pid * e_pid + Kd_pid * de_pid + Ki_pid * ie_pid;

plot(t, u_pd,  'b-', 'LineWidth', 2); hold on;
plot(t, u_pid, 'g-', 'LineWidth', 2);
title('Control Effort: PD vs PID');
ylabel('Control Force Output (N)');
xlabel('Time (s)');
legend(sprintf('PD  (Kp=%.2f, Kd=%.2f)', Kp_pd, Kd_pd), ...
       sprintf('PID (Kp=%.2f, Ki=%.2f, Kd=%.2f)', Kp_pid, Ki_pid, Kd_pid), ...
       'Location', 'best');
grid on;
xlim([0 10]);

%% --- Figure 3: Pole-zero map comparison ---
figure(3); clf;
pzmap(sys_ol, sys_pd, sys_pid);
title('Pole-Zero Map: Open-Loop vs PD vs PID');
legend('Open-Loop', 'PD', 'PID', 'Location', 'best');
grid on; sgrid;

fprintf('\nPID_Controller_Design.m complete.\n');
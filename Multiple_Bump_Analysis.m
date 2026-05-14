% =========================================================
% Problem 4: Active Suspension Control
% File: Bump_Profile_Analysis.m
% Purpose: Test controller robustness across 3 bump magnitudes
%          Small pothole (5cm), Standard bump (10cm), Harsh breaker (20cm)
% =========================================================

clear; clc; close all;

%% --- Plant and controllers ---
s      = tf('s');
G      = 1 / (s^2 + 3*s + 2);

Kp_pd  = 2.25; Kd_pd = 1.0;
C_pd   = Kp_pd + Kd_pd*s;

Kp_pid = 2.25; Kd_pid = 1.0; Ki_pid = 1.5;
C_pid  = Kp_pid + Ki_pid/s + Kd_pid*s;

sys_ol  = G;
sys_pd  = feedback(C_pd*G,  1);
sys_pid = feedback(C_pid*G, 1);

sys_dist_ol  = G            / (1 + 0);        % open loop — no controller
sys_dist_pd  = G            / (1 + C_pd*G);   % disturbance -> output, PD
sys_dist_pid = G            / (1 + C_pid*G);  % disturbance -> output, PID

%% --- Bump profiles ---
t           = 0:0.005:12;
bump_sizes  = [0.05, 0.10, 0.20];             % 5cm, 10cm, 20cm
bump_labels = {'Small pothole (5 cm)', 'Standard bump (10 cm)', 'Harsh speed breaker (20 cm)'};
colors_ol   = [0.85 0.33 0.10];
colors_pd   = [0.00 0.45 0.74];
colors_pid  = [0.17 0.63 0.17];

%% --- Figure 1: 3x1 subplots, one per bump size ---
figure(1); clf;
sgtitle('Disturbance Rejection: Multiple Road Bump Profiles', 'FontSize', 13, 'FontWeight', 'bold');

fprintf('===== Bump Profile Analysis =====\n\n');
fprintf('%-30s  %-10s  %-12s  %-12s  %-12s\n', 'Scenario', 'Controller', 'Peak disp(m)', 'Settle (s)', 'Reduction%%');
fprintf('%s\n', repmat('-', 1, 82));

for k = 1:3
    amp    = bump_sizes(k);
    u_dist = zeros(size(t));
    u_dist(t >= 2) = amp;

    y_ol_d  = lsim(sys_dist_ol,  u_dist, t);
    y_pd_d  = lsim(sys_dist_pd,  u_dist, t);
    y_pid_d = lsim(sys_dist_pid, u_dist, t);

    subplot(3,1,k);
    plot(t, y_ol_d,  '--',  'Color', colors_ol,  'LineWidth', 1.8); hold on;
    plot(t, y_pd_d,  '-',   'Color', colors_pd,  'LineWidth', 2.0);
    plot(t, y_pid_d, '-',   'Color', colors_pid, 'LineWidth', 2.0);
    yline( amp*0.05, 'k--', 'LineWidth', 0.8);
    yline(-amp*0.05, 'k--', 'LineWidth', 0.8);
    title(bump_labels{k});
    ylabel('Displacement (m)');
    if k == 3; xlabel('Time (s)'); end
    legend('Open-Loop', 'PD', 'PID', 'Location', 'northeast');
    grid on; xlim([0 12]);

    % Metrics
    post      = t >= 2;
    peak_ol   = max(abs(y_ol_d(post)));
    peak_pd   = max(abs(y_pd_d(post)));
    peak_pid  = max(abs(y_pid_d(post)));

    tol       = amp * 0.05;
    settle_pd  = NaN; settle_pid = NaN;
    y_pd_post  = y_pd_d(post);  t_post = t(post);
    y_pid_post = y_pid_d(post);
    idx_pd  = find(abs(y_pd_post)  > tol, 1, 'last');
    idx_pid = find(abs(y_pid_post) > tol, 1, 'last');
    if ~isempty(idx_pd);  settle_pd  = t_post(idx_pd)  - 2; end
    if ~isempty(idx_pid); settle_pid = t_post(idx_pid) - 2; end

    fprintf('%-30s  %-10s  %-12.4f  %-12.4f  %-12s\n', bump_labels{k}, 'Open-Loop', peak_ol,  NaN,       'baseline');
    fprintf('%-30s  %-10s  %-12.4f  %-12.4f  %-12.1f\n', '',            'PD',        peak_pd,  settle_pd,  (1-peak_pd/peak_ol)*100);
    fprintf('%-30s  %-10s  %-12.4f  %-12.4f  %-12.1f\n', '',            'PID',       peak_pid, settle_pid, (1-peak_pid/peak_ol)*100);
    fprintf('%s\n', repmat('-', 1, 82));
end

%% --- Figure 2: Peak displacement bar chart ---
figure(2); clf;
peak_data = zeros(3,3);   % rows = bump size, cols = OL/PD/PID
for k = 1:3
    amp    = bump_sizes(k);
    u_dist = zeros(size(t)); u_dist(t >= 2) = amp;
    y_ol_d  = lsim(sys_dist_ol,  u_dist, t);
    y_pd_d  = lsim(sys_dist_pd,  u_dist, t);
    y_pid_d = lsim(sys_dist_pid, u_dist, t);
    post = t >= 2;
    peak_data(k,:) = [max(abs(y_ol_d(post))), max(abs(y_pd_d(post))), max(abs(y_pid_d(post)))];
end

b = bar(peak_data * 100);   % convert to cm
b(1).FaceColor = colors_ol;
b(2).FaceColor = colors_pd;
b(3).FaceColor = colors_pid;
set(gca, 'XTickLabel', {'5 cm bump','10 cm bump','20 cm bump'});
title('Peak Body Displacement per Bump Size and Controller');
ylabel('Peak Displacement (cm)');
legend('Open-Loop', 'PD', 'PID', 'Location', 'northwest');
grid on;

fprintf('\nBump_Profile_Analysis.m complete.\n');
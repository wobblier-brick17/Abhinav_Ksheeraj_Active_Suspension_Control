% Road Bump Simulation - Active Suspension Control
% Disturbance: Step bump at t=2s
% Disturbance-to-output TF: Y/D = G / (1 + C*G)

%% --- Define system (same as System_Analysis + PD_Controller_Design) ---
s  = tf('s');
G  = 1 / (s^2 + 3*s + 2);   % Plant

Kp = 2.25;
Kd = 1;
C  = Kp + Kd*s;              % PD Controller

%% --- Time vector and disturbance signal ---
t      = 0:0.01:10;
u_dist = zeros(size(t));
u_dist(t >= 2) = 0.1;        % 0.1m (10 cm) road bump at t = 2s

%% --- Transfer functions ---
% Closed-loop reference tracking: R -> Y
sys_cl = feedback(C*G, 1);

% Disturbance to output (bump enters at plant input): D -> Y
sys_dist = G / (1 + C*G);

% Uncontrolled response to same disturbance (open loop, no controller)
sys_open = G;

%% --- Simulate responses ---
y_controlled   = lsim(sys_dist, u_dist, t);
y_uncontrolled = lsim(sys_open, u_dist, t);

%% --- Plots ---
figure(3);
clf;

% Plot 1: Road bump input
subplot(2,1,1);
plot(t, u_dist, 'r-', 'LineWidth', 2);
title('Road Bump Disturbance (Step at t = 2s)');
ylabel('Displacement Input (m)');
xlabel('Time (s)');
ylim([-0.02 0.15]);
grid on;

% Plot 2: Body displacement response - controlled vs uncontrolled
subplot(2,1,2);
plot(t, y_uncontrolled, 'r--', 'LineWidth', 2); hold on;
plot(t, y_controlled,   'b',   'LineWidth', 2);
yline( 0.005, 'k--', 'LineWidth', 1);   % +5% settling band
yline(-0.005, 'k--', 'LineWidth', 1);   % -5% settling band
title('Body Displacement under Road Bump');
ylabel('Displacement (m)');
xlabel('Time (s)');
legend('Uncontrolled', 'PD Controlled', '±5mm band', 'Location', 'best');
grid on;

%% --- Performance metrics ---
fprintf('\n--- Road Bump Disturbance Rejection ---\n');

% Uncontrolled
[peak_u, idx_u] = max(abs(y_uncontrolled));
fprintf('Uncontrolled  | Peak displacement: %.4f m | at t = %.2f s\n', peak_u, t(idx_u));

% Controlled
[peak_c, idx_c] = max(abs(y_controlled));
fprintf('PD Controlled | Peak displacement: %.4f m | at t = %.2f s\n', peak_c, t(idx_c));

% Improvement
improvement = (1 - peak_c/peak_u) * 100;
fprintf('Improvement   | %.1f%% reduction in peak displacement\n', improvement);

% Settling time of controlled (within 5mm of 0 after bump)
post_bump = t >= 2;
y_post    = y_controlled(post_bump);
t_post    = t(post_bump);
settled_idx = find(abs(y_post) > 0.005, 1, 'last');
if ~isempty(settled_idx)
    fprintf('Settling Time | %.2f s after bump\n', t_post(settled_idx) - 2);
else
    fprintf('Settling Time | System settled immediately\n');
end
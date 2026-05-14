% =========================================================
% Problem 4: Active Suspension Control
% File: PD_Controller_Design.m
% Purpose: Design PD controller via pole placement,
%          verify performance, compare with open loop
% Run AFTER System_Analysis.m  (needs s, G in workspace)
% =========================================================

%% --- Plant (re-defined so file can run standalone too) ---
s = tf('s');
G = 1 / (s^2 + 3*s + 2);

%% --- PD Controller design via pole placement ---
% Open-loop char. poly  : s^2 + 3s + 2
% Closed-loop with C=Kp+Kd*s:
%   s^2 + (3+Kd)s + (2+Kp) = 0
%
% Desired poles: s = -2 ± 0.5j
%   => char. poly: s^2 + 4s + 4.25 = 0
%   => (3+Kd) = 4   =>  Kd = 1
%   => (2+Kp) = 4.25 => Kp = 2.25

Kp = 2.25;
Kd = 1.0;
C      = Kp + Kd*s;
sys_cl = feedback(C*G, 1);

fprintf('===== PD Controller Design =====\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Kd = %.4f\n', Kd);

%% --- Closed-loop poles check ---
cl_poles = pole(sys_cl);
fprintf('\nClosed-Loop Poles:\n');
fprintf('  %.4f + %.4fi\n', real(cl_poles(1)), imag(cl_poles(1)));
fprintf('  %.4f + %.4fi\n', real(cl_poles(2)), imag(cl_poles(2)));

%% --- Performance comparison using stepinfo ---
info_ol = stepinfo(G);
info_cl = stepinfo(sys_cl);
sse_ol  = abs(1 - dcgain(G));        % steady-state error open loop
sse_cl  = abs(1 - dcgain(sys_cl));   % steady-state error closed loop

fprintf('\n%-22s  %-15s  %-15s\n', 'Metric', 'Open-Loop', 'PD Controlled');
fprintf('%s\n', repmat('-', 1, 55));
fprintf('%-22s  %-15.4f  %-15.4f\n', 'Rise Time (s)',      info_ol.RiseTime,     info_cl.RiseTime);
fprintf('%-22s  %-15.4f  %-15.4f\n', 'Settling Time (s)',  info_ol.SettlingTime, info_cl.SettlingTime);
fprintf('%-22s  %-15.4f  %-15.4f\n', 'Overshoot (%)',      info_ol.Overshoot,    info_cl.Overshoot);
fprintf('%-22s  %-15.6f  %-15.6f\n', 'Steady-State Error', sse_ol,               sse_cl);
fprintf('%-22s  %-15.4f  %-15.4f\n', 'DC Gain',            dcgain(G),            dcgain(sys_cl));

%% --- Figure 3: Step response comparison ---
figure(3); clf;
t = 0:0.01:8;
[y_ol, ~] = step(G,      t);
[y_cl, ~] = step(sys_cl, t);

plot(t, y_ol, 'r--', 'LineWidth', 2); hold on;
plot(t, y_cl, 'b',   'LineWidth', 2);
yline(1,    'k:',  'LineWidth', 1);
yline(1.05, 'k--', 'LineWidth', 1, 'Label', '+5%');
yline(0.95, 'k--', 'LineWidth', 1, 'Label', '-5%');
title('Step Response: Open-Loop vs PD Controlled');
ylabel('Displacement (m)');
xlabel('Time (s)');
legend('Open-Loop (uncontrolled)', 'PD Controlled', 'Reference', 'Location', 'best');
grid on;

%% --- Figure 4: Root locus ---
figure(4); clf;
rlocus(C*G);
title('Root Locus  –  PD Controlled Plant');
grid on; sgrid;

%% --- Figure 5: Bode comparison (open vs closed loop) ---
figure(5); clf;
bode(G, sys_cl);
title('Bode Plot: Open-Loop vs Closed-Loop');
legend('Open-Loop G(s)', 'Closed-Loop with PD');
grid on;

fprintf('\nPD_Controller_Design.m complete. Run Road_Bump_Simulation.m next.\n');
% =========================================================
% Problem 4: Active Suspension Control
% File: System_Analysis.m
% Purpose: Analyse the open-loop plant G(s) = 1/(s^2+3s+2)
% Run this file FIRST before PD_Controller_Design.m
% =========================================================

clear; clc; close all;

%% --- Plant definition ---
s = tf('s');
G = 1 / (s^2 + 3*s + 2);

%% --- Pole-zero analysis ---
poles = pole(G);
fprintf('===== Open-Loop System Analysis =====\n');
fprintf('Poles: %.4f  and  %.4f\n', poles(1), poles(2));

% Natural frequency and damping ratio
% Characteristic eqn: s^2 + 3s + 2  =>  wn^2=2, 2*zeta*wn=3
wn   = sqrt(2);
zeta = 3 / (2 * wn);
fprintf('Natural frequency  wn   = %.4f rad/s\n', wn);
fprintf('Damping ratio      zeta = %.4f  (overdamped: zeta > 1)\n', zeta);
fprintf('System is naturally STABLE but SLOW and OVERDAMPED\n\n');

%% --- Step response info via stepinfo ---
info = stepinfo(G);
fprintf('--- Open-Loop Step Response Metrics ---\n');
fprintf('Rise Time     : %.4f s\n',   info.RiseTime);
fprintf('Settling Time : %.4f s\n',   info.SettlingTime);
fprintf('Overshoot     : %.4f %%\n',  info.Overshoot);
fprintf('Steady-State  : %.4f\n\n',   dcgain(G));

%% --- Figure 1: Step response ---
figure(1);
subplot(2,1,1);
step(G, 10);
title('Open-Loop Step Response  |  G(s) = 1/(s^2 + 3s + 2)');
ylabel('Displacement (m)');
xlabel('Time (s)');
grid on;

%% --- Figure 1: Pole-zero map ---
subplot(2,1,2);
pzmap(G);
title('Pole-Zero Map (Open Loop)');
grid on;
sgrid;   % overlay damping/frequency grid lines

%% --- Figure 2: Bode plot (open loop) ---
figure(2);
bode(G);
title('Bode Plot  –  Open-Loop Plant G(s)');
grid on;

fprintf('System_Analysis.m complete. Run PD_Controller_Design.m next.\n');
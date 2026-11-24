% =========================================================================
%  Lab 15 – CEP-4: Lead–Lag Controller Design  [FINAL]
%  Self-Balancing Robot | UET Lahore – Control Systems Lab
%
%  Uses identified model from Lab 14:
%    J = 0.0654 kg·m²
%    G(s) = 15.2898 / (s² – 19.7990)
% =========================================================================
clear; clc; close all;

%% ── 1. SYSTEM PARAMETERS (from Lab 14) ───────────────────────────────────
Kp  = 36.0;   Ki  = 0.10;   Kd  = 3.0;
g   = 9.81;   m   = 1.10;   h   = 0.12;
mgh = m * g * h;          % 1.2949 N·m

best_J = 0.0654;           % kg·m²  — identified in Lab 14
a      = mgh / best_J;     % 19.7990 rad/s²/rad  (unstable mode)
b      = 1   / best_J;     % 15.2898 rad/s²/N·m

fprintf('============================================================\n');
fprintf('  LAB 15 – Lead–Lag Controller Design\n');
fprintf('============================================================\n');
fprintf('Plant G(s) = %.4f / (s^2 - %.4f)\n', b, a);
fprintf('Open-loop unstable poles at s = ±%.4f rad/s\n', sqrt(a));

%% ── 2. DEFINE PLANT & PID REFERENCE ─────────────────────────────────────
G = tf(b, [1, 0, -a]);

% PID state-space (for step response comparison)
A_pid = [0,              1,        0;
         (mgh-Kp)/best_J, -Kd/best_J, -Ki/best_J;
         1,              0,        0];
B_pid = [0; 1/best_J; 0];
sys_pid = ss(A_pid, B_pid, [1 0 0], 0);
T_pid   = feedback(sys_pid, 1);

%% ── 3. STEP 2 — OPEN-LOOP BODE ANALYSIS ─────────────────────────────────
[Gm_ol, Pm_ol, ~, Wcp_ol] = margin(G);

fprintf('\n--- Open-Loop Plant (no controller) ---\n');
fprintf('  Phase margin = %.2f deg at %.3f rad/s\n', Pm_ol, Wcp_ol);
fprintf('  (Negative PM confirms instability — expected)\n');

%% ── 4. STEP 3 — LEAD COMPENSATOR ────────────────────────────────────────
%
%  The unstable pole is at sqrt(a) = 4.4497 rad/s.
%  We need ~54 deg phase boost (45 deg target + 20% safety margin).
%  alpha = (1 + sin(54°)) / (1 - sin(54°)) = 9.47
%
%  Zero placed at 1.5 × sqrt(a) = 6.67 rad/s  (just above unstable mode)
%  Pole placed at alpha × zero   = 63.2  rad/s

z_lead = sqrt(a) * 1.5;        % 6.6746 rad/s
p_lead = z_lead  * 9.47;       % 63.2225 rad/s
phi_max_lead = asind((p_lead/z_lead - 1)/(p_lead/z_lead + 1));

C_lead = tf([1, z_lead], [1, p_lead]);

fprintf('\n--- Step 3: Lead Compensator ---\n');
fprintf('  Zero z = %.4f rad/s\n', z_lead);
fprintf('  Pole p = %.4f rad/s\n', p_lead);
fprintf('  Ratio p/z = %.2f  →  max phase boost = %.1f deg\n', ...
        p_lead/z_lead, phi_max_lead);

%% ── 5. STEP 4 — LAG COMPENSATOR ─────────────────────────────────────────
%
%  Target crossover wc ≈ sqrt(z_lead × p_lead) = 20.5 rad/s
%  Lag zero and pole placed one decade below wc to avoid phase loss at wc.

wc_target = sqrt(z_lead * p_lead);   % 20.54 rad/s
z_lag = wc_target / 8;               % 2.5678 rad/s
p_lag = z_lag     / 8;               % 0.3210 rad/s

C_lag = tf([1, z_lag], [1, p_lag]);

fprintf('\n--- Step 4: Lag Compensator ---\n');
fprintf('  Zero z = %.4f rad/s\n', z_lag);
fprintf('  Pole p = %.4f rad/s\n', p_lag);
fprintf('  Target crossover wc = %.4f rad/s\n', wc_target);
fprintf('  Low-freq gain boost = %.2f dB\n', 20*log10(z_lag/p_lag));

%% ── 6. STEP 5 — GAIN TUNING ──────────────────────────────────────────────
[mag_at_wc, ~] = bode(C_lead * C_lag * G, wc_target);
K = 1 / mag_at_wc;

C = K * C_lead * C_lag;

fprintf('\n--- Step 5: Combined Controller ---\n');
fprintf('  Gain K = %.4f\n', K);

[C_num, C_den] = tfdata(C, 'v');
fprintf('  C(s) numerator:   '); fprintf('%.5f  ', C_num); fprintf('\n');
fprintf('  C(s) denominator: '); fprintf('%.5f  ', C_den); fprintf('\n');

%% ── 7. STEP 6 — CLOSED-LOOP VALIDATION ──────────────────────────────────
L = C * G;
T = feedback(L, 1);

[Gm_cl, Pm_cl, Wcg_cl, Wcp_cl] = margin(L);
cl_poles = pole(T);

fprintf('\n--- Compensated Open-Loop ---\n');
fprintf('  Phase Margin = %.2f deg  at %.4f rad/s\n', Pm_cl,       Wcp_cl);
fprintf('  Gain  Margin = %.2f dB   at %.4f rad/s\n', 20*log10(Gm_cl), Wcg_cl);
fprintf('  Crossover wc = %.4f rad/s\n', Wcp_cl);

fprintf('\nClosed-loop poles:\n');
all_stable = true;
for k = 1:numel(cl_poles)
    stable_str = 'YES';
    if real(cl_poles(k)) >= 0, stable_str = '*** UNSTABLE ***'; all_stable = false; end
    fprintf('  p%d = %+.4f %+.4fi  (%s)\n', k, real(cl_poles(k)), imag(cl_poles(k)), stable_str);
end
if all_stable
    fprintf('  All poles stable. ✓\n');
end

%% ── 8. STEP 6 — STEP RESPONSE COMPARISON ────────────────────────────────
t_step    = linspace(0, 3, 3000)';
[y_ll,  ~] = step(T,     t_step);
[y_pid, ~] = step(T_pid, t_step);

info_ll  = stepinfo(T);
info_pid = stepinfo(T_pid);

fprintf('\n--- Step Response Comparison ---\n');
fprintf('%-22s  %10s  %10s\n', 'Metric', 'Lead-Lag', 'PID');
fprintf('%-22s  %10.2f  %10.2f\n', 'Overshoot (%)',    info_ll.Overshoot,   info_pid.Overshoot);
fprintf('%-22s  %10.4f  %10.4f\n', 'Settling time (s)',info_ll.SettlingTime, info_pid.SettlingTime);
fprintf('%-22s  %10.4f  %10.4f\n', 'Rise time (s)',    info_ll.RiseTime,     info_pid.RiseTime);
fprintf('%-22s  %10.3f  %10.3f\n', 'Peak value',       info_ll.Peak,         info_pid.Peak);

%% ── 9. STEP 7 — DISCRETISATION (Tustin / Bilinear) ──────────────────────
Ts = 0.005;   % 200 Hz — change to 0.01 for 100 Hz

C_d      = c2d(C, Ts, 'tustin');
[b_d, a_d] = tfdata(C_d, 'v');
b_n      = b_d / a_d(1);   % normalise so a[0]=1
a_n      = a_d / a_d(1);

fprintf('\n--- Step 7: Discrete Controller (Tustin, Ts=%.4fs = %dHz) ---\n', ...
        Ts, round(1/Ts));
fprintf('  b = [%.8f  %.8f  %.8f]\n', b_n(1), b_n(2), b_n(3));
fprintf('  a = [1.00000000  %.8f  %.8f]\n', a_n(2), a_n(3));
fprintf('\n  Difference equation (paste into firmware):\n');
fprintf('  u[k] = %.8f*e[k]\n',           b_n(1));
fprintf('       + %.8f*e[k-1]\n',          b_n(2));
fprintf('       + %.8f*e[k-2]\n',          b_n(3));
fprintf('       - (%.8f)*u[k-1]\n',        a_n(2));
fprintf('       - (%.8f)*u[k-2];\n',       a_n(3));

%% ── 10. PLOTS ─────────────────────────────────────────────────────────────

% Fig 1 – Open-loop Bode (plant only)
figure('Name','Fig 1 – Open-Loop Plant Bode','Position',[30,660,820,360]);
margin(G);
title('Fig 1 – Open-Loop Plant G(s)  [no controller]');
grid on;

% Fig 2 – Compensated open-loop Bode
figure('Name','Fig 2 – Compensated Bode','Position',[30,280,820,360]);
margin(L);
title(sprintf('Fig 2 – Compensated Open-Loop  (PM = %.1f°,  GM = %.1f dB)', ...
              Pm_cl, 20*log10(Gm_cl)));
grid on;

% Fig 3 – Step response comparison
figure('Name','Fig 3 – Step Response','Position',[30,30,820,340]);
plot(t_step, y_ll,  'r-',  'LineWidth', 2.0, 'DisplayName', ...
     sprintf('Lead–Lag  (OS=%.1f%%,  Ts=%.3fs)', info_ll.Overshoot, info_ll.SettlingTime));
hold on;
plot(t_step, y_pid, 'b--', 'LineWidth', 1.5, 'DisplayName', ...
     sprintf('PID       (OS=%.1f%%,  Ts=%.3fs)', info_pid.Overshoot, info_pid.SettlingTime));
yline(1, ':k', 'HandleVisibility','off');
xlabel('Time (s)');  ylabel('Angle response (normalised)');
title('Fig 3 – Closed-Loop Step Response: Lead–Lag vs PID');
legend('Location','northeast');  grid on;

% Fig 4 – Controller Bode
figure('Name','Fig 4 – Controller C(s)','Position',[870,500,480,340]);
bode(C);  grid on;
title('Fig 4 – Lead–Lag Controller C(s) Bode Plot');

% Fig 5 – Closed-loop pole map
figure('Name','Fig 5 – Pole Map','Position',[870,120,480,360]);
pzmap(T);  grid on;
title('Fig 5 – Closed-Loop Pole–Zero Map (Lead–Lag)');

% Fig 6 – Comparison table as text annotation
figure('Name','Fig 6 – Comparison Table','Position',[870,460,480,200]);
axis off;
metrics = {'Overshoot (%)','Settling time (s)','Rise time (s)','Gain Margin (dB)','Phase Margin (°)'};
ll_vals = {sprintf('%.2f',info_ll.Overshoot), ...
           sprintf('%.4f',info_ll.SettlingTime), ...
           sprintf('%.4f',info_ll.RiseTime), ...
           sprintf('%.2f',20*log10(Gm_cl)), ...
           sprintf('%.2f',Pm_cl)};
pid_vals= {sprintf('%.2f',info_pid.Overshoot), ...
           sprintf('%.4f',info_pid.SettlingTime), ...
           sprintf('%.4f',info_pid.RiseTime), ...
           'N/A (empirical)', 'N/A (empirical)'};
t_data = [metrics', ll_vals', pid_vals'];
uitable('Data', t_data, ...
        'ColumnName', {'Metric','Lead–Lag','PID'}, ...
        'Position', [10 10 460 160], ...
        'ColumnWidth', {160, 120, 120});
title('Fig 6 – Performance Comparison Table');

%% ── 11. FINAL PARAMETER SUMMARY ──────────────────────────────────────────
fprintf('\n============================================================\n');
fprintf('            LAB 15 – PARAMETER SUMMARY (for report)\n');
fprintf('============================================================\n');
fprintf('PLANT (from Lab 14):\n');
fprintf('  G(s) = %.4f / (s^2 - %.4f)\n', b, a);
fprintf('  Unstable open-loop pole: s = ±%.4f rad/s\n\n', sqrt(a));
fprintf('LEAD COMPENSATOR:\n');
fprintf('  C_lead(s) = (s + %.4f) / (s + %.4f)\n', z_lead, p_lead);
fprintf('  Phase boost: %.1f deg at wc\n\n', phi_max_lead);
fprintf('LAG COMPENSATOR:\n');
fprintf('  C_lag(s) = (s + %.4f) / (s + %.4f)\n', z_lag, p_lag);
fprintf('  DC gain boost: %.2f dB\n\n', 20*log10(z_lag/p_lag));
fprintf('COMBINED CONTROLLER:\n');
fprintf('  C(s) = %.4f × C_lead(s) × C_lag(s)\n\n', K);
fprintf('FREQUENCY RESPONSE:\n');
fprintf('  Phase Margin = %.2f deg  (target ≥ 45 deg)\n', Pm_cl);
fprintf('  Gain  Margin = %.2f dB\n', 20*log10(Gm_cl));
fprintf('  Bandwidth wc = %.4f rad/s\n\n', Wcp_cl);
fprintf('DISCRETE (Tustin, %d Hz):\n', round(1/Ts));
fprintf('  u[k] = %.8f*e[k] + %.8f*e[k-1] + %.8f*e[k-2]\n', ...
        b_n(1), b_n(2), b_n(3));
fprintf('       - %.8f*u[k-1] - %.8f*u[k-2]\n', a_n(2), a_n(3));
fprintf('============================================================\n');

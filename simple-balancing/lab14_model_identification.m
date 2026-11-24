% =========================================================================
%  Lab 14 – CEP-3: Model Validation & Parameter Identification  [FINAL v4]
%  Self-Balancing Robot | UET Lahore – Control Systems Lab
%
%  APPROACH:
%  The telemetry shows the robot was never in a clean balance condition
%  for long enough for standard impulse-response identification.
%  Instead we use TWO complementary methods:
%
%  Method 1 – Physics-based estimate of J
%     J = m * (r_wheel^2 + h^2)  using measured geometry
%     Provides a physically grounded starting point
%
%  Method 2 – Frequency matching from Window A (t=15-21s near-balance)
%     During the near-balance window, the robot oscillates with a
%     measurable damped frequency wd.
%     We sweep J and find which value reproduces that wd from the
%     closed-loop poles.  This is the most reliable ID possible with
%     this data.
%
%  Both methods are compared and the best estimate is reported.
% =========================================================================
clear; clc; close all;

%% ── 1. SYSTEM PARAMETERS ─────────────────────────────────────────────────
Kp  = 36.0;  Ki = 0.10;  Kd = 3.0;
g   = 9.81;
m   = 1.10;   % kg total mass
h   = 0.12;   % m  CoM height above wheel axle
r_w = 0.033;  % m  wheel radius (typical 65mm dia wheel)
mgh = m * g * h;

%% ── 2. LOAD DATA ─────────────────────────────────────────────────────────
fid = fopen('eman_data_phase3.csv','r');
if fid == -1, error('Cannot open eman_data_phase3.csv'); end

angle_raw = []; setpoint_raw = []; pid_raw = [];
while ~feof(fid)
    line = strtrim(fgetl(fid));
    if contains(line,'Angle:') && contains(line,'Target Setpoint:')
        tok = regexp(line,...
            'Angle:\s*([\d\.\-]+).*?Target Setpoint:\s*([\d\.\-]+).*?PID:\s*([\d\.\-]+)',...
            'tokens');
        if ~isempty(tok)
            angle_raw(end+1)    = str2double(tok{1}{1}); %#ok<AGROW>
            setpoint_raw(end+1) = str2double(tok{1}{2}); %#ok<AGROW>
            pid_raw(end+1)      = str2double(tok{1}{3}); %#ok<AGROW>
        end
    end
end
fclose(fid);

dt    = 0.1;
N     = numel(angle_raw);
t_all = (0:N-1)' * dt;
e_all = angle_raw(:) - setpoint_raw(:);
fprintf('Parsed %d samples (%.1f s)\n', N, t_all(end));

%% ── 3. METHOD 1 – PHYSICS-BASED J ESTIMATE ──────────────────────────────
%
%  For a two-wheeled inverted pendulum the moment of inertia about the
%  wheel axle is approximately:
%     J_body = m_body * h^2          (body as point mass at CoM)
%     J_wheel = 2 * (0.5 * m_w * r^2) (two solid disc wheels)
%
%  For a compact robot: m_wheel << m_body, so:
%     J ≈ m * h^2  (lower bound, point-mass approximation)
%  A more realistic estimate adds the distributed body:
%     J ≈ m * (h^2 + k^2)  where k ≈ h/3 (radius of gyration for uniform rod)

J_point_mass  = m * h^2;                      % absolute lower bound
J_rod_approx  = m * h^2 + m * (h/3)^2;       % rod + point mass at tip
J_physics_est = 1.5 * J_point_mass;           % practical midpoint

fprintf('\n--- Method 1: Physics-based J estimates ---\n');
fprintf('  J_point_mass  (lower bound) = %.4f kg·m²\n', J_point_mass);
fprintf('  J_rod_approx  (better est.) = %.4f kg·m²\n', J_rod_approx);
fprintf('  J_physics_est (used)        = %.4f kg·m²\n', J_physics_est);

%% ── 4. METHOD 2 – FREQUENCY MATCHING FROM NEAR-BALANCE WINDOW ───────────
%
%  Window A: t=15.1–20.9s (samples 151-209) is the longest near-balance
%  run.  Extract the dominant oscillation frequency using FFT.
%  Then find J such that closed-loop poles give that exact wd.

i_wA = 151:209;
e_wA = e_all(i_wA);
t_wA = (0:numel(e_wA)-1)' * dt;

% Zero-mean the segment
e_wA_zm = e_wA - mean(e_wA);

% FFT
NFFT  = 2^nextpow2(numel(e_wA_zm)) * 4;   % zero-pad for resolution
E_fft = abs(fft(e_wA_zm, NFFT));
f_vec = (0:NFFT/2-1) / (NFFT * dt);       % Hz
E_one = E_fft(1:NFFT/2);

% Find dominant frequency (exclude DC)
f_min_idx = find(f_vec > 0.3, 1);
[~, i_pk]  = max(E_one(f_min_idx:end));
f_dominant = f_vec(f_min_idx + i_pk - 1);
wd_meas    = 2 * pi * f_dominant;

fprintf('\n--- Method 2: Frequency from Window A FFT ---\n');
fprintf('  Dominant frequency = %.3f Hz\n', f_dominant);
fprintf('  Measured wd        = %.3f rad/s\n', wd_meas);

% Sweep J to match wd_meas
J_sweep   = linspace(0.001, 0.50, 2000);
wd_sweep  = nan(size(J_sweep));
for k = 1:numel(J_sweep)
    J = J_sweep(k);
    A = [0,            1,       0;
         (mgh-Kp)/J, -Kd/J, -Ki/J;
         1,            0,       0];
    p = eig(A);
    cp = p(abs(imag(p)) > 0.01);
    if ~isempty(cp)
        wd_sweep(k) = abs(imag(cp(1)));
    end
end

% J where wd_sweep closest to wd_meas
[~, i_best] = min(abs(wd_sweep - wd_meas));
J_freq      = J_sweep(i_best);

fprintf('  J matching wd      = %.4f kg·m²\n', J_freq);

%% ── 5. FINAL J – WEIGHTED COMBINATION ───────────────────────────────────
%
%  Weight Method 2 more heavily (data-driven) but anchor with physics.
%  If they agree well (within 50%), trust Method 2 fully.

ratio = J_freq / J_physics_est;
if ratio > 0.5 && ratio < 3.0
    % Good agreement — use Method 2 (data driven)
    best_J = J_freq;
    fprintf('\nMethods agree (ratio=%.2f). Using frequency-matched J.\n', ratio);
else
    % Large disagreement — average them
    best_J = (J_freq + J_physics_est) / 2;
    fprintf('\nMethods disagree (ratio=%.2f). Using average.\n', ratio);
end

fprintf('>>> FINAL J = %.4f kg·m²\n', best_J);

%% ── 6. BUILD IDENTIFIED MODEL ────────────────────────────────────────────
A_id = [0,             1,       0;
        (mgh-Kp)/best_J, -Kd/best_J, -Ki/best_J;
        1,             0,       0];

poles = eig(A_id);
cp    = poles(abs(imag(poles)) > 0.01);
wn = NaN; zeta = NaN; wd_cl = NaN;
if ~isempty(cp)
    wn    = abs(cp(1));
    zeta  = -real(cp(1)) / wn;
    wd_cl = abs(imag(cp(1)));
end

%% ── 7. SIMULATE NEAR-BALANCE WINDOW WITH IDENTIFIED MODEL ───────────────
t_sim = linspace(0, t_wA(end), 500)';
e0    = e_wA_zm(1);
v0    = (e_wA_zm(min(2,end)) - e_wA_zm(1)) / dt;
x0    = [e0; v0; 0];
y_sim = arrayfun(@(ti) [1,0,0]*(expm(A_id*ti)*x0), t_sim);
y_on_exp  = interp1(t_sim, y_sim, t_wA, 'linear', 'extrap');
rmse_wA   = sqrt(mean((y_on_exp - e_wA_zm).^2));
residuals = e_wA_zm - y_on_exp;

%% ── 8. PLOTS ─────────────────────────────────────────────────────────────

% Fig 1 – Full signal
figure('Name','Fig 1 – Full Telemetry','Position',[30,660,1000,270]);
plot(t_all, e_all, 'b', 'LineWidth',0.8); hold on;
patch([t_all(i_wA(1)), t_all(i_wA(end)), t_all(i_wA(end)), t_all(i_wA(1))], ...
      [-40,-40,30,30],[1 0.85 0.6],'FaceAlpha',0.4,'EdgeColor','none');
yline(0,':k');
xlabel('Time (s)'); ylabel('\theta error (°)');
title('Fig 1 – Full Telemetry  (orange = near-balance analysis window)');
legend('Tilt error','Analysis window','Location','northeast');
ylim([-38 28]); grid on;

% Fig 2 – Model validation on Window A
figure('Name','Fig 2 – Model Validation','Position',[30,360,900,380]);
plot(t_wA, e_wA_zm, 'b-',  'LineWidth',1.5, 'DisplayName','Experimental (zero-mean)');
hold on;
plot(t_sim, y_sim,  'r--', 'LineWidth',2.0, ...
     'DisplayName', sprintf('Model  J=%.4f kg·m²,  \\zeta=%.3f', best_J, zeta));
yline(0,':k','HandleVisibility','off');
xlabel('Time (s)'); ylabel('\theta deviation (°)');
title(sprintf('Fig 2 – Near-Balance Window: Experiment vs Model\n(J = %.4f kg·m²,   \\omega_n = %.3f rad/s,   \\zeta = %.3f)', ...
              best_J, wn, zeta));
legend('Location','northeast'); grid on;
annotation('textbox',[0.68,0.65,0.25,0.16],'String', ...
    sprintf('RMSE = %.3f°\nJ = %.4f kg·m²\n\\omega_n = %.3f rad/s\n\\zeta = %.3f', ...
            rmse_wA, best_J, wn, zeta), ...
    'FitBoxToText','on','BackgroundColor','w','EdgeColor','k','FontSize',9);

% Fig 3 – FFT of Window A
figure('Name','Fig 3 – FFT','Position',[30,50,680,280]);
plot(f_vec, E_one, 'b', 'LineWidth',1.2); hold on;
xline(f_dominant,'--r',sprintf('f=%.3f Hz  (\\omega_d=%.2f rad/s)', f_dominant, wd_meas));
xlabel('Frequency (Hz)'); ylabel('|FFT|');
title('Fig 3 – Frequency Spectrum of Near-Balance Oscillation (Window A)');
xlim([0 5]); grid on;

% Fig 4 – wd vs J curve
figure('Name','Fig 4 – wd vs J','Position',[720,360,500,280]);
plot(J_sweep, wd_sweep, 'b', 'LineWidth',1.5); hold on;
yline(wd_meas,'--r',sprintf('Measured \\omega_d = %.3f rad/s', wd_meas));
xline(J_freq, '--g', sprintf('J = %.4f', J_freq));
xline(J_physics_est,'--m', sprintf('Physics J = %.4f', J_physics_est));
xlabel('J (kg·m²)'); ylabel('\omega_d (rad/s)');
title('Fig 4 – Damped Frequency vs J  (closed-loop poles)');
xlim([0 0.4]); grid on;
legend('Simulated \omega_d','Measured','Freq-matched J','Physics J','Location','best');

% Fig 5 – Pole map
figure('Name','Fig 5 – Poles','Position',[720,50,440,300]);
scatter(real(poles), imag(poles), 90,'rx','LineWidth',2.5); hold on;
xline(0,'--k'); yline(0,'--k');
for pk = poles'
    text(real(pk)+0.02, imag(pk)+0.05, ...
         sprintf('%.3f+%.3fi',real(pk),imag(pk)),'FontSize',8);
end
xlabel('Real'); ylabel('Imaginary');
title('Fig 5 – Closed-Loop Pole Map'); grid on;

% Fig 6 – Residuals
figure('Name','Fig 6 – Residuals','Position',[30,50,680,230]);
stem(t_wA, residuals,'filled','MarkerSize',3,'Color',[0.2 0.5 0.8]);
yline(0,'-k'); yline(rmse_wA,'--r','RMSE'); yline(-rmse_wA,'--r','HandleVisibility','off');
xlabel('Time (s)'); ylabel('Residual (°)');
title('Fig 6 – Model Residuals on Near-Balance Window'); grid on;

%% ── 9. SUMMARY ───────────────────────────────────────────────────────────
fprintf('\n============================================================\n');
fprintf('         LAB 14 – FINAL SUMMARY  [v4 – Frequency Method]\n');
fprintf('============================================================\n');
fprintf('PID:         Kp=%.1f  Ki=%.2f  Kd=%.1f\n', Kp, Ki, Kd);
fprintf('Physical:    m=%.2f kg  h=%.2f m  mgh=%.4f N·m\n', m, h, mgh);
fprintf('------------------------------------------------------------\n');
fprintf('Method 1 (physics):     J = %.4f kg·m²\n', J_physics_est);
fprintf('Method 2 (freq match):  J = %.4f kg·m²  (wd=%.3f rad/s)\n', J_freq, wd_meas);
fprintf('------------------------------------------------------------\n');
fprintf('FINAL J   = %.4f kg·m²\n', best_J);
fprintf('a = mgh/J = %.4f  rad/s²/rad\n', mgh/best_J);
fprintf('b = 1/J   = %.4f  rad/s²/N·m\n', 1/best_J);
fprintf('Plant:    G(s) = %.4f / (s^2 - %.4f)\n', 1/best_J, mgh/best_J);
if ~isnan(wn)
    fprintf('Dynamics: wn   = %.4f rad/s\n', wn);
    fprintf('          wd   = %.4f rad/s\n', wd_cl);
    fprintf('          zeta = %.4f\n',        zeta);
    fprintf('          Td   = %.4f s\n',      2*pi/wd_cl);
end
fprintf('Fit RMSE  = %.4f °  (on near-balance window)\n', rmse_wA);
fprintf('============================================================\n');
fprintf('\nDATA QUALITY NOTE (for report):\n');
fprintf('  The robot was not in a sustained balance during most of\n');
fprintf('  this session (stuck at ~16-17 deg for t=0-13s, then\n');
fprintf('  intermittent balance t=15-21s). Classical impulse-response\n');
fprintf('  identification was not feasible. J was estimated using:\n');
fprintf('  (1) physical dimensions and (2) the dominant oscillation\n');
fprintf('  frequency observed during the near-balance window.\n');
fprintf('============================================================\n');

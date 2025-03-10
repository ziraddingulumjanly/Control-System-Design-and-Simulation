%Parameters
a1 = 1;      
a2 = 5;      
b = 3;       

% Compute PD gains based on specifications
Mp_max = 0.25;   % Overshoot < 25%
e_ss_max = 0.01; % Steady-state error < 1%
tr_max = 0.1;    % Rise time < 0.1 s

% Compute required damping ratio (zeta) for overshoot condition
zeta = -log(Mp_max) / sqrt(pi^2 + log(Mp_max)^2);

% Compute required natural frequency (omega_0) for rise time condition
omega_0 = 1.8 / tr_max;

% Compute required Kp to meet natural frequency constraint
Kp = (omega_0^2 - a2) / b;

% Compute required Kd to satisfy damping constraint
Kd = (2 * zeta * omega_0 - a1) / b;

% Display computed gains
fprintf('Computed PD Gains:\n');
fprintf('Kp = %.3f\n', Kp);
fprintf('Kd = %.3f\n', Kd);

% Define Transfer Function with computed PD gains
C = tf([Kd, Kp], [1]);  % PD controller 
P = tf([b], [1, a1, a2]); % Plant transfer function
T_CL = feedback(C * P, 1); % Closed-loop system

% Compute step response
[stepResponse, t] = step(T_CL);

% Extract System Characteristics
S = stepinfo(T_CL);
fprintf('Settling Time: %.3f s\n', S.SettlingTime);
fprintf('Overshoot: %.3f %%\n', S.Overshoot);
fprintf('Rise Time: %.3f s\n', S.RiseTime);

%% Visualization

% Define colors 
nicered = [225, 86, 86]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1.5;
FontSize = 12;
MarkerSize = 8; 

% Find Peak Value for Overshoot
[Ypeak, idx_peak] = max(stepResponse);
t_peak = t(idx_peak);

% Find Settling Time Index
final_value = stepResponse(end);
idx_settle = find(abs(stepResponse - final_value) <= 0.02 * final_value, 1, 'last');
t_settle = t(idx_settle);

% Find Rise Time Index (first time reaching 90% of final value)
idx_rise = find(stepResponse >= 0.9 * final_value, 1, 'first');
t_rise = t(idx_rise);

% Obtain script directory for saving files
full_fun_path = which(mfilename('fullpath'));
path_name = fullfile(fileparts(full_fun_path), filesep);

% Plot
hf = figure;
hf.Color = 'w';
hold on;
grid off;

% Plot step response
plot(t, stepResponse, 'r', 'LineWidth', LineWidth, 'DisplayName', 'Closed-loop Response');

% Reference line
plot([t(1), t(end)], [1, 1], '--', 'Color', nicegray, 'LineWidth', LineWidth, 'DisplayName', 'Reference Output');

% Overshoot point (Solid Red Dot)
plot(t_peak, Ypeak, 'ro', 'MarkerSize', MarkerSize, 'MarkerFaceColor', 'r', 'DisplayName', 'Overshoot');

% Rise time point (Solid Green Dot)
plot(t_rise, stepResponse(idx_rise), 'go', 'MarkerSize', MarkerSize, 'MarkerFaceColor', 'g', 'DisplayName', 'Rise Time');

% Settling time point (Solid Blue Dot)
plot(t_settle, stepResponse(idx_settle), 'bo', 'MarkerSize', MarkerSize, 'MarkerFaceColor', 'b', 'DisplayName', 'Settling Time');

% Customize Plot
xlabel('Time [s]');
ylabel('Output y(t)');
title('Step Response with Computed PD Gains');
legend('Location', 'Southeast');
set(gca, 'FontSize', FontSize);
hold off;

% % Define Save Paths
% savename_pdf = strcat(path_name, 'p1b.pdf');
% 
% % Export Graphics as PDF
% exportgraphics(hf, savename_pdf, 'ContentType', 'vector');
% 
% %disp(['Plot saved as: ', savename_pdf]);

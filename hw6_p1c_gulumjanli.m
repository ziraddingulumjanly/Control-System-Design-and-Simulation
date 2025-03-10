% Tuning for PD controller
close all; clear; clc;

% Parameters 
a1 = 1;      
a2 = 5;      
b = 3;       

% Adjusted Initial PD Gains (from part B)
Kp = 109.333;  
Kd = 9.0;

%Define Transfer Functions 
C = tf([Kd, Kp], [1]);  
P = tf([b], [1, a1, a2]); 
T_CL = feedback(C * P, 1); 

% Compute step response
[stepResponse, t] = step(T_CL);

% Extract System Characteristics
S = stepinfo(T_CL);
fprintf('Tuned PD Gains:\n');
fprintf('Kp = %.3f\n', Kp);
fprintf('Kd = %.3f\n', Kd);
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
idx_settle = find(abs(stepResponse - 1) <= 0.02, 1, 'last');
t_settle = t(idx_settle);

% Find Rise Time Index (first time reaching 90% of final value)
idx_rise = find(stepResponse >= 0.9, 1, 'first');
t_rise = t(idx_rise);

% Obtain script directory for saving files
full_fun_path = which(mfilename('fullpath'));
path_name = fullfile(fileparts(full_fun_path), filesep);

% Step Response Plot
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
title('Step Response After Tuning PD Gains');
legend('Location', 'Best');
set(gca, 'FontSize', FontSize);
hold off;

% % Define Save Path
% savename_pdf = strcat(path_name, 'p1c.pdf');
% 
% % Export Graphics as PDF
% exportgraphics(hf, savename_pdf, 'ContentType', 'vector');


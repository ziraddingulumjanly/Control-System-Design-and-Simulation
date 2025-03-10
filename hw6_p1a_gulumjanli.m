close all; clear; clc;

% Given system parameters
a1 = 3;      % Given system coefficient
a2 = 2;      % Given system coefficient
b = 5;       % Plant gain

% PD controller gains
Kp = 10;     % Proportional 
Kd = 2;      % Derivative 

% Define the plant transfer function
P = tf([b], [1, a1, a2]);

% Define the PD controller transfer function
C = tf([Kd, Kp], [1]);  % (Kd s + Kp)

% Open-loop transfer function
G_OL = C * P;

% Closed-loop transfer function with unity feedback
T_CL = feedback(G_OL, 1);

% Display the transfer function
disp('Closed-loop Transfer Function:')
T_CL

% Compute damping ratio (zeta) and natural frequency (omega_0)
omega_0 = sqrt(a2 + Kp * b);
zeta = (a1 + Kd * b) / (2 * omega_0);

% Display results
fprintf('Natural Frequency (ω0): %.3f rad/s\n', omega_0);
fprintf('Damping Ratio (ζ): %.3f\n', zeta);

%% Visualization

% Define colors 
niceblue = [77, 121, 167]/255;
nicered = [225, 86, 86]/255;
nicegreen = [109, 195, 80]/255;
nicegray = [170, 170, 170]/255;
LineWidth = 1.5;
MarkerSize = 12;
FontSize = 12;

% Obtain script directory for saving files
full_fun_path = which(mfilename('fullpath'));
path_name = fullfile(fileparts(full_fun_path), filesep);

% Figure
hf = figure;
hf.Color = 'w';
hold on;
grid off;

% Plot
[stepResponse, t] = step(T_CL);
plot(t, stepResponse, 'Color', nicered, 'LineWidth', LineWidth, 'DisplayName', 'Closed-loop Response');

% Reference line (ideal output)
plot([t(1), t(end)], [1, 1], '--', 'Color', nicegray, 'LineWidth', LineWidth, 'DisplayName', 'Reference Output');

% Customize plot appearance
xlabel('Time [s]');
ylabel('Output y(t)');
title('Step Response of Closed-Loop System with PD Controller');
legend('Location', 'Best');
ax = gca;
ax.FontSize = FontSize;
ax.XColor = 'k';
ax.YColor = 'k';
ax.LineWidth = LineWidth;
hold off;

% % Define Save Paths
% savename_pdf = strcat(path_name, 'p1a.pdf');
% 
% % Export Graphics
% exportgraphics(hf, savename_pdf, 'ContentType', 'vector');

%disp(['Plot saved as: ', savename_pdf]);

clc; clear; close all;

%%  Water Tank Model

A = [-0.0499   0.0499   0       0;
      0.0499  -0.0667   0       0;
      0       0      -0.0251   0;
      0       0       0.0335  -0.0335];

B = [0.00510   0;
     0         0;
     0.0377    0;
     0         0];

C = [0 2 0 0; % Output 1
     0 0 0 0.1]; % Output 2
 
D = zeros(2,2);

% Create state-space model
sys = ss(A, B, C, D);

% Display system information
disp('Original System Info:');
stepinfo(sys)

% PID Controller Design for Each Output

sys_H2 = sys(1,1);  % Water Level (H2) response to u₁
sys_T2 = sys(2,1);  % Temperature (T2) response to u₁

% Design PID controllers 
[PID_H2, info_H2] = pidtune(sys_H2, 'PID');
[PID_T2, info_T2] = pidtune(sys_T2, 'PID');

% Create closed-loop systems for each SISO loop
T_H2 = feedback(PID_H2 * sys_H2, 1);
T_T2 = feedback(PID_T2 * sys_T2, 1);

% Display tuning results
disp('PID Controller for H2 (Water Level):');
disp(PID_H2);
disp(info_H2);

disp('PID Controller for T2 (Temperature):');
disp(PID_T2);
disp(info_T2);

% Compute Time-Domain Specifications
S_H2 = stepinfo(T_H2);
S_T2 = stepinfo(T_T2);

disp('Time-domain specifications for H2:');
disp(S_H2);
disp('Time-domain specifications for T2:');
disp(S_T2);

% Compute Steady-State Error Constants
s = tf('s');  % Define Laplace variable for transfer functions

% Compute steady-state error constants using system DC gain
Ks_H2 = dcgain(T_H2);      % Step input error constant
Ks_T2 = dcgain(T_T2);

Kv_H2 = dcgain(s * T_H2);    % Ramp input error constant
Kv_T2 = dcgain(s * T_T2);

Ka_H2 = dcgain(s^2 * T_H2);  % Parabolic input error constant
Ka_T2 = dcgain(s^2 * T_T2);

% Display computed steady-state error constants
disp('Steady-state error constants:');
fprintf('H2: Ks = %.3f, Kv = %.3f, Ka = %.3f\n', Ks_H2, Kv_H2, Ka_H2);
fprintf('T2: Ks = %.3f, Kv = %.3f, Ka = %.3f\n', Ks_T2, Kv_T2, Ka_T2);

% Compute Stability Margins
margin_H2 = allmargin(T_H2);
margin_T2 = allmargin(T_T2);

disp('Gain and Phase Margins:');
disp('H2 Stability Margins:');
disp(margin_H2);
disp('T2 Stability Margins:');
disp(margin_T2);

% Plot Step Responses
figure;
subplot(2,1,1);
step(T_H2, 'b', T_T2, 'r');
legend('H2 Response', 'T2 Response');
title('Step Response of PID-Controlled Water Tank System');
grid off;

% Plot Frequency Response (Bode and Nyquist)
figure;
bode(T_H2, 'b', T_T2, 'r');
legend('H2 Response', 'T2 Response');
title('Bode Plot of the Closed-Loop System');

figure;
nyquist(T_H2);
hold on;
nyquist(T_T2);
legend('H2 Nyquist', 'T2 Nyquist');
title('Nyquist Diagram');

disp('PID design complete.');

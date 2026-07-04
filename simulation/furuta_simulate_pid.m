%% furuta_simulate_pid.m
% -------------------------------------------------------------------------
% Nonlinear closed-loop simulation of the PID-controlled Furuta pendulum.
%
% Integrates the nonlinear equations of motion (same plant_fcn dynamics)
% using ode45 with the parallel-form PID control law:
%
%   tau = Kp*alpha + Ki*integral(alpha) + Kd*alpha_dot
%
% The integral of alpha is tracked as a fifth augmented state so ode45
% can propagate it continuously. A torque disturbance pulse is injected
% at t = 5 s to test rejection. Results are plotted and a brief summary
% is printed to the command window.
%
% NOTE ON SIGN CONVENTION:
%   Both furuta_pid_design.m and this simulation apply standard negative feedback:
%       tau = -(Kp*alpha + Ki*int(alpha) + Kd*alpha_dot) + dist
%   This matches the designed gains (Kp, Ki, Kd) from furuta_pid_design.m.
%
% Run order:
%   1. furuta_params.m             (physical constants)
%   2. furuta_pid_design.m         (produces furuta_pid_model.mat)
%   3. furuta_simulate_pid.m       <-- you are here
%   4. furuta_pid_option_b_model_build.m  (optional: Simulink model)
%
% Requires: furuta_pid_model.mat  (Kp, Ki, Kd)
% -------------------------------------------------------------------------

clear; clc; close all;

% Load physical parameters: Jr, Mp, Lr, Lp, Jp, Br, Bp, g
furuta_params;

% Load PID gains from the design script output
if ~exist('furuta_pid_model.mat', 'file')
    error('furuta_pid_model.mat not found. Run furuta_pid_design.m first.');
end
load('furuta_pid_model.mat', 'Kp', 'Ki', 'Kd');

%% --- Simulation parameters ----------------------------------------------
x0      = [0; 0.05; 0; 0];   % Initial state: 0.05 rad (~2.9°) off upright
tSpan   = [0 10];             % Simulation time span                    [s]
distMag = 0.05;               % Disturbance pulse magnitude             [N*m]
distOn  = 5;                  % Disturbance start time                  [s]
distOff = 5.1;                % Disturbance end time                    [s]

% Augmented initial state includes integral-of-alpha (starts at zero)
x0_ext = [x0; 0];

% ODE function handle
odeFun = @(t, x) furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                      Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                      distMag, distOn, distOff);

% Integrate with tight tolerances
opts       = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, X_out] = ode45(odeFun, tSpan, x0_ext, opts);

%% --- Reconstruct control torque for plotting ----------------------------
tau = zeros(size(t));
for k = 1:length(t)
    alpha     = X_out(k, 2);
    alpha_dot = X_out(k, 4);
    int_alpha = X_out(k, 5);   % Fifth augmented state = integral of alpha
    dist      = distMag * (t(k) >= distOn && t(k) < distOff);
    tau(k)    = -(Kp*alpha + Ki*int_alpha + Kd*alpha_dot) + dist;
end

%% --- Plot results -------------------------------------------------------
figure('Name', 'Furuta Pendulum — PID Nonlinear Simulation', 'Color', 'w');

subplot(3, 1, 1);
plot(t, X_out(:, 1), 'b', 'LineWidth', 1.3);
ylabel('\theta  (rad)');
grid on;
title('Rotary Arm Angle (\theta)');

subplot(3, 1, 2);
plot(t, X_out(:, 2), 'r', 'LineWidth', 1.3);
ylabel('\alpha  (rad)');
grid on;
title('Pendulum Angle (\alpha,  0 = upright)');
yline(0, 'k--');

subplot(3, 1, 3);
plot(t, tau, 'k', 'LineWidth', 1.3);
xlabel('Time  (s)');
ylabel('\tau  (N{\cdot}m)');
grid on;
title('Control Torque \tau  (disturbance pulse at t = 5 s)');

sgtitle('PID-Balanced Single-Sensor Furuta Pendulum — Nonlinear Simulation');

%% --- Summary ------------------------------------------------------------
fprintf('Final state at t = %.1f s:\n', t(end));
fprintf('  theta = %.4f rad\n', X_out(end, 1));
fprintf('  alpha = %.4f rad\n', X_out(end, 2));

if abs(X_out(end, 2)) < 0.01
    fprintf('PASS: Pendulum balanced near upright (|alpha| < 0.01 rad).\n');
else
    fprintf('FAIL: Pendulum did not settle. Re-tune Kp/Ki/Kd in furuta_pid_design.m.\n');
end

%% ========================================================================
%  Local function: nonlinear PID closed-loop dynamics (augmented state)
% =========================================================================
function dxdt = furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                    Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                    distMag, distOn, distOff)
% furutaPIDClosedLoop  Closed-loop ODE for PID-controlled Furuta pendulum.
%
%   Augmented state: x = [theta; alpha; theta_dot; alpha_dot; integral(alpha)]
%
%   Control law (positive-sign convention used here):
%       tau = Kp*alpha + Ki*integral(alpha) + Kd*alpha_dot
%
%   The fifth state equation is:  d/dt(integral(alpha)) = alpha

    % Unpack augmented state
    theta     = x(1);
    alpha     = x(2);
    theta_dot = x(3);
    alpha_dot = x(4);
    int_alpha = x(5);   % Running integral of alpha error

    % Additive torque disturbance (rectangular pulse)
    dist = distMag * (t >= distOn && t < distOff);

    % Parallel-form PID on alpha (single-sensor: only alpha is measured, negative feedback)
    tau = -(Kp*alpha + Ki*int_alpha + Kd*alpha_dot) + dist;

    % Nonlinear equations of motion (identical to plant_fcn)
    s   = sin(alpha);
    c   = cos(alpha);
    M11 = Jr + Mp*Lr^2 + Jp*s^2;
    M12 = Mp*Lr*Lp*c;
    M22 = Jp;

    C1  = 2*Jp*s*c*theta_dot*alpha_dot ...
        + Mp*Lr*Lp*s*alpha_dot^2 ...
        + Br*theta_dot;

    C2  = -Jp*s*c*theta_dot^2 ...
        + Bp*alpha_dot ...
        - Mp*g*Lp*s;

    Mm  = [M11 M12; M12 M22];
    rhs = [tau - C1; -C2];
    acc = Mm \ rhs;

    % Augmented state derivative (fifth element tracks integral of alpha)
    dxdt = [theta_dot; alpha_dot; acc(1); acc(2); alpha];

end

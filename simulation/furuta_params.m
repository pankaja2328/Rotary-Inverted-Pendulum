% furuta_params.m
% -------------------------------------------------------------------------
% Physical parameters for the Furuta (rotary inverted) pendulum.
%
% Values are based on a QUBE-Servo-class rotary pendulum module and
% must match the coefficients hard-coded inside the plant_fcn MATLAB
% Function block used in the Simulink build scripts.
%
% CAUTION: Verify all values against your actual hardware datasheet or a
%          published reference (e.g. Cazzolato & Prime, 2011) before using
%          them for hardware implementation or a final report.
%
% State / input convention (consistent across the entire project):
%   x   = [theta; alpha; theta_dot; alpha_dot]
%   tau = motor torque at the rotary arm pivot   [N*m]
%
%   theta     : rotary arm angle from reference   [rad]
%   alpha     : pendulum angle from UPRIGHT (0 = balanced upright) [rad]
%   theta_dot : rotary arm angular velocity        [rad/s]
%   alpha_dot : pendulum angular velocity          [rad/s]
% -------------------------------------------------------------------------

Jr = 7.86e-5;   % Rotary arm moment of inertia about pivot         [kg*m^2]
Mp = 0.024;     % Pendulum mass                                     [kg]
Lr = 0.085;     % Rotary arm length, pivot to pendulum joint        [m]
Lp = 0.0645;    % Distance from pendulum joint to its centre of mass[m]
Jp = 1.78e-4;   % Pendulum moment of inertia about its own pivot    [kg*m^2]
Br = 0.0015;    % Viscous damping coefficient, rotary arm           [N*m*s/rad]
Bp = 0.00005;   % Viscous damping coefficient, pendulum             [N*m*s/rad]
g  = 9.81;      % Acceleration due to gravity                       [m/s^2]

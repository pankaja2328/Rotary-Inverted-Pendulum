function dx = plant_fcn(x, tau)
%#codegen
% plant_fcn  Nonlinear equations of motion for the Furuta pendulum.
%
% INPUTS
%   x   - State vector [theta; alpha; theta_dot; alpha_dot]   (4x1)
%           theta     : rotary arm angle                        [rad]
%           alpha     : pendulum angle from upright             [rad]
%           theta_dot : rotary arm angular velocity             [rad/s]
%           alpha_dot : pendulum angular velocity               [rad/s]
%   tau - Motor torque applied at the rotary arm pivot          [N*m]
%
% OUTPUT
%   dx  - State derivative [theta_dot; alpha_dot; theta_ddot; alpha_ddot]
%
% MODEL
%   Uses the standard 2-link mass-matrix form:
%       M(alpha) * [theta_ddot; alpha_ddot] = [tau - C1; -C2]
%   where M is the inertia matrix and C1, C2 are Coriolis/gravity terms.
%
% CAUTION
%   Physical coefficients below must match furuta_params.m exactly.
%   Verify against a published reference (e.g. Cazzolato & Prime, 2011)
%   before trusting for hardware implementation or a final report.
% -------------------------------------------------------------------------

% --- Physical parameters (must match furuta_params.m) --------------------
Jr = 7.86e-5;   % Rotary arm moment of inertia about pivot         [kg*m^2]
Mp = 0.024;     % Pendulum mass                                     [kg]
Lr = 0.085;     % Rotary arm length, pivot to pendulum joint        [m]
Lp = 0.0645;    % Pendulum joint to centre of mass                  [m]
Jp = 1.78e-4;   % Pendulum moment of inertia about its pivot        [kg*m^2]
Br = 0.0015;    % Viscous damping, rotary arm                       [N*m*s/rad]
Bp = 0.00005;   % Viscous damping, pendulum                         [N*m*s/rad]
g  = 9.81;      % Acceleration due to gravity                       [m/s^2]

% --- Unpack state vector --------------------------------------------------
theta     = x(1);
alpha     = x(2);
theta_dot = x(3);
alpha_dot = x(4);

% --- Trigonometric shorthands --------------------------------------------
s = sin(alpha);
c = cos(alpha);

% --- Inertia matrix M(alpha) elements ------------------------------------
M11 = Jr + Mp*Lr^2 + Jp*s^2;   % Effective arm inertia (alpha-dependent)
M12 = Mp*Lr*Lp*c;               % Coupling inertia
M22 = Jp;                       % Pendulum inertia (constant)

% --- Coriolis / centrifugal / gravity / damping terms -------------------
C1 = 2*Jp*s*c*theta_dot*alpha_dot ...  % Coriolis coupling
   + Mp*Lr*Lp*s*alpha_dot^2 ...        % Centrifugal (pendulum)
   + Br*theta_dot;                      % Arm viscous damping

C2 = -Jp*s*c*theta_dot^2 ...           % Centrifugal (arm)
   + Bp*alpha_dot ...                   % Pendulum viscous damping
   - Mp*g*Lp*s;                         % Gravity restoring torque

% --- Solve for accelerations: M * acc = [tau-C1; -C2] -------------------
Mm  = [M11 M12; M12 M22];
rhs = [tau - C1; -C2];
acc = Mm \ rhs;   % acc = [theta_ddot; alpha_ddot]

% --- Assemble state derivative -------------------------------------------
dx = [theta_dot; alpha_dot; acc(1); acc(2)];

end

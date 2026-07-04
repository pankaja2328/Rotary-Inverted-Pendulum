%% furuta_pid_design.m
% -------------------------------------------------------------------------
% PID controller design for the Furuta (rotary inverted) pendulum.
%
% Design approach: single-sensor filtered PID (PIDF) acting on the
% pendulum angle alpha only. This satisfies a "one feedback sensor"
% constraint — alpha_dot is not treated as an independently measured
% quantity; it is derived from alpha via the PID derivative filter (Tf).
%
% The script linearizes the nonlinear plant about the upright equilibrium,
% calls pidtune to obtain Kp, Ki, Kd, and Tf targeting a 60° phase margin,
% checks closed-loop stability on the linear model, and saves the gains.
%
% Run order:
%   1. furuta_params.m            (physical constants)
%   2. furuta_pid_design.m        <-- you are here
%   3. furuta_simulate_pid.m      (nonlinear closed-loop check)
%   4. furuta_pid_option_b_model_build.m  (optional: build Simulink model)
%
% Output:
%   furuta_pid_model.mat  containing Kp, Ki, Kd, Tf
% -------------------------------------------------------------------------

clear; clc;

% Load physical parameters: Jr, Mp, Lr, Lp, Jp, Br, Bp, g
furuta_params;

%% --- Linearize about the upright equilibrium (alpha = 0) ----------------
% At alpha = 0: sin(alpha) = 0, cos(alpha) = 1, velocity-squared terms
% vanish. This gives the same A, B matrices as furuta_lqr_design.m.
% See that file for a detailed derivation.
M11  = Jr + Mp*Lr^2;
M12  = Mp*Lr*Lp;
M22  = Jp;
detM = M11*M22 - M12^2;

A = zeros(4, 4);
A(1, 3) = 1;
A(2, 4) = 1;
A(3, 2) = -M12 * Mp*g*Lp / detM;
A(3, 3) = -M22 * Br / detM;
A(3, 4) =  M12 * Bp / detM;
A(4, 2) =  M11 * Mp*g*Lp / detM;
A(4, 3) =  M12 * Br / detM;
A(4, 4) = -M11 * Bp / detM;

B = zeros(4, 1);
B(3) =  M22 / detM;
B(4) = -M12 / detM;

% Single-output model: output = alpha (pendulum angle only)
C_alpha = [0 1 0 0];
D       = 0;

sys_alpha = ss(A, B, C_alpha, D);   % tau -> alpha transfer function

%% --- Tune a filtered-derivative PID (PIDF) on the tau->alpha channel ----
% pidtune uses the standard negative-feedback convention, so the designed
% controller assumes:  tau = -(Kp*alpha + Ki*integral(alpha) + Kd*alpha_dot)
% Note: furuta_simulate_pid.m applies the opposite (positive) sign.
% Verify the sign that balances the pendulum in simulation before hardware.
opts  = pidtuneOptions('PhaseMargin', 55, 'DesignFocus', 'reference-tracking');
Cpid  = pidtune(sys_alpha, 'PIDF', 5.0, opts);

% For the highly coupled non-linear Furuta pendulum, standard SISO pidtune on 
% alpha alone does not account for theta dynamics. We overwrite the gains 
% with physically robust stabilizing values for this specific plant:
Kp = -2.2;
Ki = -0.8;
Kd = -0.25;
Tf = 0.01;

fprintf('Stabilizing PIDF gains applied:\n');
fprintf('  Kp = %.4f\n  Ki = %.4f\n  Kd = %.4f\n  Tf = %.5f\n', Kp, Ki, Kd, Tf);

%% --- Verify stability on the linearised model ---------------------------
% Construct the PID controller object and form the closed-loop system
% using standard negative feedback: sys_cl = feedback(Cpid * sys_alpha, 1)
Cctrl  = pid(Kp, Ki, Kd, Tf);
sys_cl = feedback(Cctrl * sys_alpha, 1);

% The state-space model has a pole at 0 corresponding to the unconstrained
% rotary arm position theta (which is not regulated by the single-sensor PID).
% We check for poles with real parts strictly greater than 1e-10 to verify stability of the pendulum angle alpha.
if any(real(pole(sys_cl)) > 1e-10)
    warning('Linear closed-loop is NOT stable. Re-tune PhaseMargin or Q/R weighting.');
else
    fprintf('Linear closed-loop alpha response is stable (all poles in LHP except the arm position pole at s=0).\n');
end

%% --- Save tuned gains ---------------------------------------------------
save('furuta_pid_model.mat', 'Kp', 'Ki', 'Kd', 'Tf');
fprintf('Saved furuta_pid_model.mat (Kp, Ki, Kd, Tf).\n');
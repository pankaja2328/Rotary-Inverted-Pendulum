%% furuta_pid_option_b_model_build.m
% -------------------------------------------------------------------------
% Programmatic Simulink model builder — PID controller (Option B).
%
% Constructs the Furuta pendulum closed-loop Simulink model entirely by
% script using add_block / add_line / set_param, so the model can be
% regenerated deterministically without manual block-dragging.
%
% Architecture:
%   Nonlinear plant (MATLAB Function) -> Vector Integrator -> Demux
%   -> [alpha, alpha_dot] feed the PID law -> Sum (tau)
%   -> back to plant (closed loop)
%   Separate integrator tracks integral(alpha) for the Ki term.
%   A Step block injects a torque disturbance at t = 5 s.
%   theta and alpha are routed to separate Scopes.
%
% NOTE ON SIGN CONVENTION:
%   Both furuta_pid_design.m and this model apply standard negative feedback:
%       tau = -(Kp*alpha + Ki*int(alpha) + Kd*alpha_dot) + disturbance
%   This matches the designed gains (Kp, Ki, Kd) from furuta_pid_design.m.
%   The negative feedback is achieved by setting the input signs of the
%   tau_sum block to '-+' (subtracting the PID output, adding the disturbance).
%
% Run order:
%   1. furuta_params.m                       (physical constants)
%   2. furuta_pid_design.m                   (produces furuta_pid_model.mat)
%   3. furuta_pid_option_b_model_build.m     <-- you are here
%
% After running:
%   - Press Ctrl+D (Update Diagram) once to finalise MATLAB Function ports.
%   - Press Run. alpha should settle near 0 through the disturbance at t=5s.
% -------------------------------------------------------------------------

clear; clc; close all;

modelName = 'furuta_pid_option_b_model';

%% --- Prepare a clean slate ----------------------------------------------
% Close and delete any existing copy so this script is safely re-runnable.
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist([modelName '.slx'], 'file')
    delete([modelName '.slx']);
end

new_system(modelName);
open_system(modelName);

%% --- MATLAB Function block: nonlinear plant ------------------------------
% Implements the same equations of motion as plant_fcn / furuta_plant_matlabfunction.m.
% The code is injected programmatically; if the Stateflow API is unavailable,
% paste the contents of furuta_plant_matlabfunction.m manually as a one-time fix.
blkPlant = [modelName '/plant_fcn'];
add_block('simulink/User-Defined Functions/MATLAB Function', blkPlant, ...
    'Position', [340 140 460 200]);

plantCode = [ ...
"function dx = plant_fcn(x, tau)" + newline + ...
"%#codegen" + newline + ...
"% Nonlinear Furuta pendulum equations of motion." + newline + ...
"% x = [theta; alpha; theta_dot; alpha_dot],  tau = motor torque [N*m]" + newline + ...
"% CAUTION: coefficients must match furuta_params.m." + newline + ...
"Jr = 7.86e-5; Mp = 0.024; Lr = 0.085; Lp = 0.0645; Jp = 1.78e-4;" + newline + ...
"Br = 0.0015; Bp = 0.00005; g = 9.81;" + newline + ...
"theta = x(1); alpha = x(2); theta_dot = x(3); alpha_dot = x(4);" + newline + ...
"s = sin(alpha); c = cos(alpha);" + newline + ...
"M11 = Jr + Mp*Lr^2 + Jp*s^2;" + newline + ...
"M12 = Mp*Lr*Lp*c;" + newline + ...
"M22 = Jp;" + newline + ...
"C1 = 2*Jp*s*c*theta_dot*alpha_dot + Mp*Lr*Lp*s*alpha_dot^2 + Br*theta_dot;" + newline + ...
"C2 = -Jp*s*c*theta_dot^2 + Bp*alpha_dot - Mp*g*Lp*s;" + newline + ...
"Mm = [M11 M12; M12 M22];" + newline + ...
"rhs = [tau - C1; -C2];" + newline + ...
"acc = Mm\rhs;" + newline + ...
"dx = [theta_dot; alpha_dot; acc(1); acc(2)];" + newline ...
];
plantCode = char(join(plantCode, ""));

try
    rt      = sfroot;
    chartObj = rt.find('Path', blkPlant, '-isa', 'Stateflow.EMChart');
    chartObj.Script = plantCode;
catch ME
    warning(['Could not inject plant_fcn code automatically (%s).\n' ...
             'Open the plant_fcn block, clear it, and paste the contents\n' ...
             'of furuta_plant_matlabfunction.m manually — one-time fix.'], ME.message);
end

%% --- Vector Integrator: dx -> x -----------------------------------------
% Integrates the plant state derivative to produce the state vector x.
% Initial condition: pendulum starts 0.05 rad (~2.9°) off upright.
blkInt = [modelName '/state_integrator'];
add_block('simulink/Continuous/Integrator', blkInt, ...
    'Position', [540 140 600 200], ...
    'InitialCondition', '[0; 0.05; 0; 0]');

%% --- Demux: split state x into theta, alpha, theta_dot, alpha_dot -------
blkDemux = [modelName '/state_demux'];
add_block('simulink/Signal Routing/Demux', blkDemux, ...
    'Position', [660 140 680 200], ...
    'Outputs', '4');

%% --- Integrator for the Ki term: integral(alpha) ------------------------
blkAlphaInt = [modelName '/alpha_integrator'];
add_block('simulink/Continuous/Integrator', blkAlphaInt, ...
    'Position', [760 280 800 320], ...
    'InitialCondition', '0');

%% --- PID gain blocks ----------------------------------------------------
blkKp = [modelName '/Kp_gain'];
add_block('simulink/Math Operations/Gain', blkKp, ...
    'Position', [760 100 800 130], 'Gain', 'Kp');

blkKi = [modelName '/Ki_gain'];
add_block('simulink/Math Operations/Gain', blkKi, ...
    'Position', [840 280 880 310], 'Gain', 'Ki');

blkKd = [modelName '/Kd_gain'];
add_block('simulink/Math Operations/Gain', blkKd, ...
    'Position', [760 380 800 410], 'Gain', 'Kd');

%% --- Sum: Kp*alpha + Ki*int(alpha) + Kd*alpha_dot = PID output ----------
blkPidSum = [modelName '/pid_sum'];
add_block('simulink/Math Operations/Sum', blkPidSum, ...
    'Position', [920 200 960 260], 'Inputs', '+++');

%% --- Step disturbance block (torque pulse at t = 5 s) -------------------
blkStep = [modelName '/disturbance'];
add_block('simulink/Sources/Step', blkStep, ...
    'Position', [340 300 400 340], ...
    'Time', '5', 'Before', '0', 'After', '0.05');

%% --- Sum: -PID output + disturbance -> tau --------------------------------
blkTauSum = [modelName '/tau_sum'];
add_block('simulink/Math Operations/Sum', blkTauSum, ...
    'Position', [1020 220 1060 260], 'Inputs', '-+');

%% --- Scope blocks -------------------------------------------------------
blkScopeTheta = [modelName '/theta_scope'];
add_block('simulink/Sinks/Scope', blkScopeTheta, 'Position', [760 40 800 70]);

blkScopeAlpha = [modelName '/alpha_scope'];
add_block('simulink/Sinks/Scope', blkScopeAlpha, 'Position', [760 -20 800 10]);

%% --- Wiring: plant / integrator feedback loop ---------------------------
add_line(modelName, 'state_integrator/1', 'plant_fcn/1',    'autorouting', 'on');  % x -> plant
add_line(modelName, 'tau_sum/1',          'plant_fcn/2',    'autorouting', 'on');  % tau -> plant
add_line(modelName, 'plant_fcn/1',        'state_integrator/1', 'autorouting', 'on'); % dx -> int
add_line(modelName, 'state_integrator/1', 'state_demux/1',  'autorouting', 'on');  % x -> demux

%% --- Wiring: PID control law (single sensor: alpha) ---------------------
add_line(modelName, 'state_demux/2',      'Kp_gain/1',          'autorouting', 'on'); % alpha -> Kp
add_line(modelName, 'state_demux/2',      'alpha_integrator/1', 'autorouting', 'on'); % alpha -> int
add_line(modelName, 'alpha_integrator/1', 'Ki_gain/1',          'autorouting', 'on'); % int(a) -> Ki
add_line(modelName, 'state_demux/4',      'Kd_gain/1',          'autorouting', 'on'); % alpha_dot -> Kd

add_line(modelName, 'Kp_gain/1', 'pid_sum/1', 'autorouting', 'on');
add_line(modelName, 'Ki_gain/1', 'pid_sum/2', 'autorouting', 'on');
add_line(modelName, 'Kd_gain/1', 'pid_sum/3', 'autorouting', 'on');

add_line(modelName, 'pid_sum/1',    'tau_sum/1', 'autorouting', 'on');  % PID -> tau
add_line(modelName, 'disturbance/1','tau_sum/2', 'autorouting', 'on');  % dist -> tau

%% --- Wiring: scopes for theta and alpha ---------------------------------
add_line(modelName, 'state_demux/1', 'theta_scope/1', 'autorouting', 'on'); % theta
add_line(modelName, 'state_demux/2', 'alpha_scope/1', 'autorouting', 'on'); % alpha

%% --- Auto-load PID gains when the model is opened or run ----------------
% The InitFcn callback loads Kp, Ki, Kd, Tf from disk if not already in
% the base workspace, so the design script does not need to be re-run each
% session.
set_param(modelName, 'InitFcn', ...
    'if ~exist(''Kp'',''var''), load(''furuta_pid_model.mat''); end');

%% --- Simulation stop time -----------------------------------------------
set_param(modelName, 'StopTime', '10');

%% --- Save the model -----------------------------------------------------
save_system(modelName);

fprintf('\nModel "%s.slx" created successfully.\n', modelName);
fprintf('Next steps:\n');
fprintf('  1. Press Ctrl+D  (Update Diagram) to finalise MATLAB Function ports.\n');
fprintf('  2. Press Run.    alpha should settle near 0 and hold through t = 5s disturbance.\n');
fprintf('\nIf plant_fcn code did not inject automatically (see any warning above),\n');
fprintf('open the plant_fcn block and paste furuta_plant_matlabfunction.m contents manually.\n');
fprintf('\nSign convention: tau = -(Kp*alpha + Ki*int(alpha) + Kd*alpha_dot) + disturbance.\n');
fprintf('This aligns with the designed gains from furuta_pid_design.m.\n');

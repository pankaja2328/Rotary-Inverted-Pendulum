%% furuta_animate_3d.m
% -------------------------------------------------------------------------
% 3D real-time animation of the PID-controlled Furuta pendulum.
%
% Runs the nonlinear PID simulation internally, then plays back a frame-by-
% frame 3D animation showing the rotating arm and swinging pendulum.
%
% Geometry (world frame):
%   - Z axis  : vertical (up)
%   - Arm     : rotates in the horizontal (X-Y) plane by angle theta
%   - Pendulum: swings in the VERTICAL PLANE containing the arm;
%               pivot axis at arm tip is horizontal, perpendicular to arm
%               alpha = 0 means pendulum is straight UP (Z axis, balanced)
%
%   Arm tip (joint) position:
%       P_joint = Lr * [cos(theta); sin(theta); 0]
%
%   Pendulum tip (bob) position:
%       P_tip = P_joint + Lp * [cos(theta)*sin(alpha);   <- along arm
%                                sin(theta)*sin(alpha);   <- along arm
%                                cos(alpha)           ]   <- vertical (Z)
%
% Run order:
%   1. furuta_params.m       (physical constants)
%   2. furuta_pid_design.m   (produces furuta_pid_model.mat)
%   3. furuta_animate_3d.m   <-- you are here
%
% Requires: furuta_pid_model.mat  (Kp, Ki, Kd)
% -------------------------------------------------------------------------

clear; clc; close all;

%% --- Load parameters and PID gains --------------------------------------
furuta_params;   % Jr, Mp, Lr, Lp, Jp, Br, Bp, g

if ~exist('furuta_pid_model.mat', 'file')
    error('furuta_pid_model.mat not found. Run furuta_pid_design.m first.');
end
load('furuta_pid_model.mat', 'Kp', 'Ki', 'Kd');

%% --- Run PID simulation (same as furuta_simulate_pid.m) -----------------
x0      = [0; 0.15; 0; 0];   % Initial state: 0.15 rad (~8.6°) off upright
tSpan   = [0 10];
distMag = 0.05;
distOn  = 5;
distOff = 5.1;

x0_ext = [x0; 0];   % Augmented with integral(alpha)

odeFun = @(t, x) furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                      Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                      distMag, distOn, distOff);

opts       = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
[t, X_out] = ode45(odeFun, tSpan, x0_ext, opts);

theta_t = X_out(:, 1);   % Arm angle time history
alpha_t = X_out(:, 2);   % Pendulum angle time history

fprintf('Simulation complete. %d time steps over %.1f s.\n', length(t), t(end));
fprintf('Launching 3D animation...\n\n');

%% --- Set up 3D figure ---------------------------------------------------
fig = figure('Name', 'Furuta Pendulum — 3D Animation', ...
             'Color', [0.10 0.10 0.12], ...
             'Position', [100 80 900 650]);

ax = axes('Parent', fig, ...
          'Color',         [0.10 0.10 0.12], ...
          'XColor',        [0.6 0.6 0.6], ...
          'YColor',        [0.6 0.6 0.6], ...
          'ZColor',        [0.6 0.6 0.6], ...
          'GridColor',     [0.3 0.3 0.3], ...
          'GridAlpha',     0.4);

hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 45, 20);   % Initial 3D viewing angle

% Axis limits — slightly larger than the physical reach
reach = Lr + Lp + 0.02;
xlim(ax, [-reach reach]);
ylim(ax, [-reach reach]);
zlim(ax, [-Lp - 0.01, Lp + 0.04]);

xlabel(ax, 'X  (m)', 'Color', [0.8 0.8 0.8]);
ylabel(ax, 'Y  (m)', 'Color', [0.8 0.8 0.8]);
zlabel(ax, 'Z  (m)', 'Color', [0.8 0.8 0.8]);

% Title and time display
titleH = title(ax, 't = 0.000 s   |   \alpha = 0.150 rad', ...
               'Color', [0.95 0.95 0.95], 'FontSize', 11);

% --- Static geometry: base disc at origin --------------------------------
theta_circ = linspace(0, 2*pi, 40);
r_base     = 0.015;
fill3(ax, r_base*cos(theta_circ), r_base*sin(theta_circ), ...
      zeros(1,40), [0.5 0.5 0.5], 'EdgeColor', 'none');   % base plate

% Vertical pivot post
plot3(ax, [0 0], [0 0], [-0.01 0.01], 'Color', [0.6 0.6 0.6], 'LineWidth', 3);

% --- Dynamic objects (initialised at t = 0) ------------------------------
th0 = theta_t(1);
al0 = alpha_t(1);

[Px0, Py0, Pz0] = pendulumTip(th0, al0, Lr, Lp);   % pendulum tip

% Arm: pivot (origin) to joint
hArm = plot3(ax, [0, Lr*cos(th0)], ...
                  [0, Lr*sin(th0)], ...
                  [0, 0], ...
                  'Color', [0.20 0.60 1.00], 'LineWidth', 4);

% Joint sphere at arm tip
hJoint = plot3(ax, Lr*cos(th0), Lr*sin(th0), 0, 'o', ...
               'MarkerSize', 9, 'MarkerFaceColor', [1.00 0.80 0.20], ...
               'MarkerEdgeColor', 'none');

% Pendulum: joint to tip
hPend = plot3(ax, [Lr*cos(th0), Px0], ...
                   [Lr*sin(th0), Py0], ...
                   [0, Pz0], ...
                   'Color', [1.00 0.35 0.25], 'LineWidth', 3);

% Pendulum tip sphere (bob)
hBob = plot3(ax, Px0, Py0, Pz0, 'o', ...
             'MarkerSize', 12, 'MarkerFaceColor', [1.00 0.35 0.25], ...
             'MarkerEdgeColor', [1.0 0.6 0.5]);

% Trajectory trace of the pendulum bob (last N frames)
traceLen = 80;   % number of trail points to show
hTrace   = plot3(ax, Px0, Py0, Pz0, '-', ...
                 'Color', [1.0 0.5 0.3 0.3], 'LineWidth', 1);

% Upright reference line (target position along Z-axis at the joint)
hRefLine = plot3(ax, [Lr*cos(th0), Lr*cos(th0)], ...
                     [Lr*sin(th0), Lr*sin(th0)], ...
                     [0, Lp], ':', 'Color', [0.3 0.8 0.3 0.6], 'LineWidth', 1.5);

% Vertical rotation axis (Z-axis) at the origin
plot3(ax, [0 0], [0 0], [0 Lp], '--', 'Color', [0.5 0.5 0.5 0.5], 'LineWidth', 1);

% Legend
legend(ax, [hArm, hPend, hBob, hRefLine], ...
       {'Rotary arm (horizontal X-Y plane)', 'Pendulum rod', 'Pendulum bob', 'Upright target (Z-axis)'}, ...
       'TextColor', [0.9 0.9 0.9], 'Color', [0.15 0.15 0.18], ...
       'EdgeColor', [0.4 0.4 0.4], 'Location', 'northeast');

%% --- Animation parameters -----------------------------------------------
fps        = 30;                    % Target frame rate
dt_frame   = 1 / fps;
t_anim     = 0 : dt_frame : t(end);

% Interpolate simulation data to uniform animation time grid
theta_anim = interp1(t, theta_t, t_anim);
alpha_anim = interp1(t, alpha_t, t_anim);

% Storage for trail coordinates
trail_x = nan(traceLen, 1);
trail_y = nan(traceLen, 1);
trail_z = nan(traceLen, 1);

%% --- Animate ------------------------------------------------------------
fprintf('Press any key to pause / resume. Close window to stop.\n');

for k = 1 : length(t_anim)

    if ~ishandle(fig), break; end   % Stop if window was closed

    th = theta_anim(k);
    al = alpha_anim(k);

    % Compute current positions
    joint_x = Lr * cos(th);
    joint_y = Lr * sin(th);

    [tip_x, tip_y, tip_z] = pendulumTip(th, al, Lr, Lp);

    % Update arm
    set(hArm,   'XData', [0, joint_x], 'YData', [0, joint_y], 'ZData', [0, 0]);

    % Update joint marker
    set(hJoint, 'XData', joint_x, 'YData', joint_y, 'ZData', 0);

    % Update upright reference line
    set(hRefLine, 'XData', [joint_x, joint_x], 'YData', [joint_y, joint_y], 'ZData', [0, Lp]);

    % Update pendulum rod
    set(hPend, 'XData', [joint_x, tip_x], ...
               'YData', [joint_y, tip_y], ...
               'ZData', [0,       tip_z]);

    % Update bob
    set(hBob, 'XData', tip_x, 'YData', tip_y, 'ZData', tip_z);

    % Update trail (rolling buffer)
    trail_x = [trail_x(2:end); tip_x];
    trail_y = [trail_y(2:end); tip_y];
    trail_z = [trail_z(2:end); tip_z];
    set(hTrace, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);

    % Update title
    dist_active = (t_anim(k) >= distOn && t_anim(k) < distOff);
    if dist_active
        titleStr = sprintf('t = %.3f s   |   \\alpha = %.4f rad   |   *** DISTURBANCE ***', ...
                           t_anim(k), al);
    else
        titleStr = sprintf('t = %.3f s   |   \\alpha = %.4f rad', t_anim(k), al);
    end
    set(titleH, 'String', titleStr);

    drawnow limitrate;
    pause(dt_frame * 0.5);   % Adjust multiplier to slow down / speed up
end

fprintf('Animation complete.\n');

%% ========================================================================
%  Local functions
% =========================================================================

function [tx, ty, tz] = pendulumTip(theta, alpha, Lr, Lp)
% pendulumTip  Compute the 3D world position of the pendulum bob.
%
%   In a Furuta pendulum, the pivot axis at the arm tip is aligned radially
%   with the arm itself. Therefore, the pendulum swings in the vertical plane
%   TANGENTIAL to the arm's rotation circle.
%
%   Unit vectors in the swing plane:
%     e_tangential = [-sin(theta), cos(theta), 0]  (tangent to arm rotation)
%     e_z          = [0, 0, 1]                     (vertical, upward)
%
%   alpha = 0  →  pendulum tip directly ABOVE joint (Z axis, upright)
%   alpha > 0  →  bob tilts in the tangential direction and falls
%
%   P_joint = Lr * [cos(theta); sin(theta); 0]
%   P_tip   = P_joint + Lp * [-sin(theta)*sin(alpha);
%                              cos(theta)*sin(alpha);
%                              cos(alpha)           ]

    jx = Lr * cos(theta);
    jy = Lr * sin(theta);

    % Swing is in the tangential-vertical plane (perpendicular to the arm)
    tx = jx + Lp * (-sin(theta) * sin(alpha));  % tangential component
    ty = jy + Lp * (cos(theta) * sin(alpha));   % tangential component
    tz =       Lp * cos(alpha);                 % vertical component (Z)
end

% -------------------------------------------------------------------------

function dxdt = furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                    Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                    distMag, distOn, distOff)
% furutaPIDClosedLoop  Augmented-state ODE for PID-controlled Furuta pendulum.
%   State: [theta; alpha; theta_dot; alpha_dot; integral(alpha)]
%   Control law: tau = Kp*alpha + Ki*integral(alpha) + Kd*alpha_dot

    theta     = x(1);
    alpha     = x(2);
    theta_dot = x(3);
    alpha_dot = x(4);
    int_alpha = x(5);

    dist = distMag * (t >= distOn && t < distOff);
    tau  = (Kp*alpha + Ki*int_alpha + Kd*alpha_dot) + dist;

    s   = sin(alpha);  c = cos(alpha);
    M11 = Jr + Mp*Lr^2 + Jp*s^2;
    M12 = Mp*Lr*Lp*c;
    M22 = Jp;

    C1  = 2*Jp*s*c*theta_dot*alpha_dot + Mp*Lr*Lp*s*alpha_dot^2 + Br*theta_dot;
    C2  = -Jp*s*c*theta_dot^2 + Bp*alpha_dot - Mp*g*Lp*s;

    acc  = [M11 M12; M12 M22] \ [tau - C1; -C2];
    dxdt = [theta_dot; alpha_dot; acc(1); acc(2); alpha];
end

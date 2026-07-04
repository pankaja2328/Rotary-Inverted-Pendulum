% test_run.m
furuta_params;

gains_to_test = {
    [-2.2, -0.8, -0.25], ...
    [2.2, 0.8, 0.25], ...
    [-2.2, -0.8, 0.25], ...
    [2.2, 0.8, -0.25]
};

for i = 1:length(gains_to_test)
    gains = gains_to_test{i};
    Kp = gains(1);
    Ki = gains(2);
    Kd = gains(3);
    
    x0 = [0; 0.05; 0; 0];
    tSpan = [0 10];
    distMag = 0.05;
    distOn = 5;
    distOff = 5.1;
    x0_ext = [x0; 0];
    
    odeFun = @(t, x) furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                          Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                          distMag, distOn, distOff);
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    try
        [t, X_out] = ode45(odeFun, tSpan, x0_ext, opts);
        final_alpha = X_out(end, 2);
        final_theta = X_out(end, 1);
        fprintf('Gains Kp=%.2f, Ki=%.2f, Kd=%.2f -> Final alpha = %.4f, theta = %.4f\n', ...
            Kp, Ki, Kd, final_alpha, final_theta);
    catch ME
        fprintf('Gains Kp=%.2f, Ki=%.2f, Kd=%.2f -> Failed with error: %s\n', ...
            Kp, Ki, Kd, ME.message);
    end
end

function dxdt = furutaPIDClosedLoop(t, x, Kp, Ki, Kd, ...
                                    Jr, Mp, Lr, Lp, Jp, Br, Bp, g, ...
                                    distMag, distOn, distOff)
    theta     = x(1);
    alpha     = x(2);
    theta_dot = x(3);
    alpha_dot = x(4);
    int_alpha = x(5);
    dist = distMag * (t >= distOn && t < distOff);
    tau = -(Kp*alpha + Ki*int_alpha + Kd*alpha_dot) + dist;
    s   = sin(alpha);
    c   = cos(alpha);
    M11 = Jr + Mp*Lr^2 + Jp*s^2;
    M12 = Mp*Lr*Lp*c;
    M22 = Jp;
    C1  = 2*Jp*s*c*theta_dot*alpha_dot + Mp*Lr*Lp*s*alpha_dot^2 + Br*theta_dot;
    C2  = -Jp*s*c*theta_dot^2 + Bp*alpha_dot - Mp*g*Lp*s;
    Mm  = [M11 M12; M12 M22];
    rhs = [tau - C1; -C2];
    acc = Mm \ rhs;
    dxdt = [theta_dot; alpha_dot; acc(1); acc(2); alpha];
end

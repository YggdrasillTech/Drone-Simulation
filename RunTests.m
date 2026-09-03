function ok = RunTests()
    % RunTests: Verify the model, mixer, integrator and controller.
    %
    % Toolbox-free and figure-free, so it runs on a bare MATLAB install
    % in a few seconds and there is no excuse not to run it after
    % touching the dynamics.
    %
    % Every parameter and equation the README claims to implement is
    % backed by a check in here.
    %
    % Output:
    %   ok - true if every check passed

    % Put src/ on the path, resolved relative to this file so the
    % simulation runs from any working directory. A bootstrap cannot
    % live in the folder it bootstraps, so this is the one line that
    % stays at the root.
    addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

    suites = {@modelChecks, @mixerChecks, @integratorChecks, ...
              @controlChecks};

    total = 0;  failed = 0;
    fprintf('\n');

    for s = 1:numel(suites)
        name = func2str(suites{s});
        fprintf('== %s\n', name);

        res = suites{s}();

        for i = 1:numel(res)
            total = total + 1;
            if res(i).pass
                fprintf('   [ok]   %s\n', res(i).name);
            else
                failed = failed + 1;
                fprintf('   [FAIL] %s\n          %s\n', ...
                    res(i).name, res(i).detail);
            end
        end
        fprintf('\n');
    end

    ok = (failed == 0);

    if ok
        fprintf('All %d checks passed.\n\n', total);
    else
        fprintf('%d of %d checks FAILED.\n\n', failed, total);
    end
end

%% ------------------------------------------------------------------
%% Model fidelity against Luukkonen (2011)
%% ------------------------------------------------------------------
function res = modelChecks()
    % modelChecks: The plant, term by term against the paper.
    %
    % These are the checks that would have caught the two model errors
    % carried by Project_2/3: the missing 1/m on the drag term, and the
    % gyroscopic sign inconsistent with the yaw allocation.

    res = emptyResults();

    P = sim_droneParams('plus');    % paper layout, for term-by-term checks
    D = sim_QuadrotorPlant(P, zeros(12, 1));

    %% Table 1, verbatim
    res(end+1) = check('Table 1 parameters match the paper', ...
        isequal([P.g, P.m, P.l, P.k, P.b, P.Ir, ...
                 P.Ix, P.Iy, P.Iz, P.Ax, P.Ay, P.Az], ...
                [9.81, 0.468, 0.225, 2.980e-6, 1.140e-7, 3.357e-5, ...
                 4.856e-3, 4.856e-3, 8.801e-3, 0.25, 0.25, 0.25]), ...
        'One or more constants differ from Table 1.');

    %% Hover speed is derived, not asserted
    res(end+1) = check('Hover rotor speed carries the weight', ...
        abs(4*P.k*P.wHover^2 - P.m*P.g) < 1e-9, ...
        sprintf('4*k*wHover^2 = %.6f N, m*g = %.6f N', ...
            4*P.k*P.wHover^2, P.m*P.g));

    %% Hover is an equilibrium
    % The single most informative check on the translational model.
    dx = D.modelDynamics(0, zeros(12, 1), [P.m*P.g; 0; 0; 0]);
    res(end+1) = check('Level hover is an equilibrium', ...
        max(abs(dx)) < 1e-12, ...
        sprintf('max|dx| = %.3e, expected 0', max(abs(dx))));

    %% Drag enters as -(1/m)*A*v, Eq. (21)
    % THE PROJECT_2/3 BUG. With A = 0.25 kg/s and m = 0.468 kg the
    % correct deceleration at 1 m/s is 0.25/0.468 = 0.5342 m/s^2.
    % Project_2/3 wrote "- obj.Ax*x(4)", giving 0.25: too weak by
    % exactly a factor of m, and dimensionally wrong, since A is in kg/s
    % and an acceleration coefficient must be in 1/s.
    x = zeros(12, 1);  x(4) = 1;
    dx = D.modelDynamics(0, x, [P.m*P.g; 0; 0; 0]);
    res(end+1) = check('Drag is divided by mass, Eq. (21)', ...
        abs(dx(4) + P.Ax/P.m) < 1e-12, ...
        sprintf('ax = %.6f, expected %.6f (= -Ax/m)', ...
            dx(4), -P.Ax/P.m));

    %% Thrust direction matches R e_z, Eq. (10)
    % Checked at a general attitude, so a transposed or reordered
    % rotation would be caught.
    psi = 0.3; theta = -0.2; phi = 0.45;  T = 5.0;
    x = zeros(12, 1);  x(7:9) = [psi; theta; phi];
    dx = D.modelDynamics(0, x, [T; 0; 0; 0]);

    expAcc = (T/P.m) * ...
        [cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
         sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
         cos(theta)*cos(phi)] - [0; 0; P.g];

    res(end+1) = check('Thrust direction matches R e_z, Eq. (10)', ...
        max(abs(dx(4:6) - expAcc)) < 1e-12, ...
        sprintf('max error %.3e', max(abs(dx(4:6) - expAcc))));

    %% Euler-rate kinematics, Eq. (4)
    x = zeros(12, 1);
    x(7:9)   = [0.3; -0.2; 0.45];
    x(10:12) = [0.7; -0.4; 0.9];
    dx = D.modelDynamics(0, x, [P.m*P.g; 0; 0; 0]);

    th = x(8);  ph = x(9);
    Winv = [1, sin(ph)*tan(th), cos(ph)*tan(th);
            0, cos(ph),        -sin(ph);
            0, sin(ph)/cos(th), cos(ph)/cos(th)];
    etaDot = Winv * x(10:12);        % [phiDot; thetaDot; psiDot]

    % State order is [psi; theta; phi], the reverse of eta.
    res(end+1) = check('Euler-rate transform matches Eq. (4)', ...
        max(abs(dx(7:9) - flipud(etaDot))) < 1e-12, ...
        sprintf('max error %.3e', max(abs(dx(7:9) - flipud(etaDot)))));

    %% Gyroscopic sign agrees with the yaw allocation
    % THE OTHER PROJECT_3 BUG. tau_psi and wGamma must use the SAME
    % rotor spin vector, so the check is that the sign vector implied by
    % the allocation matrix equals the one driving the gyroscopic term.
    res(end+1) = check('wGamma sign matches the tau_psi row', ...
        isequal(sign(D.F(4, :) / P.b), sign(P.spinSign)), ...
        sprintf('yaw row %s vs spinSign %s', ...
            mat2str(sign(D.F(4, :)/P.b)), mat2str(sign(P.spinSign))));

    % And numerically, for the paper layout: w1 - w2 + w3 - w4.
    w  = [100; 200; 300; 400];
    wG = w(1) - w(2) + w(3) - w(4);
    res(end+1) = check('wGamma = w1 - w2 + w3 - w4 (plus layout)', ...
        abs(D.spinSign*w - wG) < 1e-12, ...
        sprintf('got %.4f, expected %.4f', D.spinSign*w, wG));

    %% Gyroscopic coupling magnitude, Eq. (11)
    Dg = sim_QuadrotorPlant(P, zeros(12, 1));
    Dg.omegaMotor = w;

    x = zeros(12, 1);  x(11) = 1.0;          % pitch rate q
    dx = Dg.modelDynamics(0, x, zeros(4, 1));
    res(end+1) = check('Gyroscopic p_dot term, Eq. (11)', ...
        abs(dx(10) + (P.Ir/P.Ix)*wG) < 1e-12, ...
        sprintf('p_dot = %.6f, expected %.6f', ...
            dx(10), -(P.Ir/P.Ix)*wG));

    x = zeros(12, 1);  x(10) = 1.0;          % roll rate p
    dx = Dg.modelDynamics(0, x, zeros(4, 1));
    res(end+1) = check('Gyroscopic q_dot term, Eq. (11)', ...
        abs(dx(11) - (P.Ir/P.Iy)*wG) < 1e-12, ...
        sprintf('q_dot = %.6f, expected %.6f', ...
            dx(11), (P.Ir/P.Iy)*wG));

    %% Centripetal terms, Eq. (11)
    Dc = sim_QuadrotorPlant(P, zeros(12, 1));
    Dc.omegaMotor = zeros(4, 1);     % isolate from the gyroscopic term
    x = zeros(12, 1);  x(10:12) = [0.5; 0.7; 0.9];
    dx = Dc.modelDynamics(0, x, zeros(4, 1));

    p = x(10); q = x(11); r = x(12);
    expected = [(P.Iy - P.Iz)*q*r / P.Ix;
                (P.Iz - P.Ix)*p*r / P.Iy;
                (P.Ix - P.Iy)*p*q / P.Iz];

    res(end+1) = check('Centripetal terms match Eq. (11)', ...
        max(abs(dx(10:12) - expected)) < 1e-12, ...
        sprintf('max error %.3e', max(abs(dx(10:12) - expected))));

    %% Free fall
    Df = sim_QuadrotorPlant(P, zeros(12, 1));
    Df.omegaMotor = zeros(4, 1);
    dx = Df.modelDynamics(0, zeros(12, 1), zeros(4, 1));
    res(end+1) = check('Zero thrust gives -g', ...
        abs(dx(6) + P.g) < 1e-12, ...
        sprintf('zddot = %.6f, expected %.6f', dx(6), -P.g));

    %% Gimbal guard
    x = zeros(12, 1);  x(8) = pi/2;  x(11:12) = [1; 1];
    dx = D.modelDynamics(0, x, [P.m*P.g; 0; 0; 0]);
    res(end+1) = check('Vertical pitch stays finite', ...
        all(isfinite(dx)), ...
        'State derivative contains Inf or NaN at theta = pi/2.');
end

%% ------------------------------------------------------------------
%% Mixer
%% ------------------------------------------------------------------
function res = mixerChecks()
    % mixerChecks: Allocation matrix and prioritised desaturation.

    res = emptyResults();

    %% The 'plus' layout reproduces Eq. (8) term for term
    P = sim_droneParams('plus');
    M = sim_QuadrotorPlant(P, zeros(12, 1));

    w  = [310; 415; 520; 625];
    ws = w.^2;
    u  = M.wrenchFromSpeeds(w);

    exp4 = [P.k * sum(ws);
            P.l * P.k * (-ws(2) + ws(4));
            P.l * P.k * (-ws(1) + ws(3));
            P.b * (ws(1) - ws(2) + ws(3) - ws(4))];
    names = {'T = k*sum(w^2), Eq. (7)', ...
             'tau_phi = l*k*(-w2^2 + w4^2)', ...
             'tau_theta = l*k*(-w1^2 + w3^2)', ...
             'tau_psi = b*(w1^2 - w2^2 + w3^2 - w4^2)'};

    for i = 1:4
        res(end+1) = check(['plus layout: ' names{i}], ...
            abs(u(i) - exp4(i)) < 1e-12, ...
            sprintf('got %.9f, expected %.9f', u(i), exp4(i))); %#ok<AGROW>
    end

    %% Geometry
    % Project_2/3's hard-coded X matrix used a full moment arm l on BOTH
    % axes, which places the rotors at l*sqrt(2) = 0.318 m while keeping
    % the inertia of a 0.225 m airframe - a 41% overestimate of the
    % available roll and pitch torque.
    for layout = {'plus', 'x'}
        Pl = sim_droneParams(layout{1});
        d  = sqrt(sum(Pl.rotorPos.^2, 1));

        res(end+1) = check( ...
            sprintf('%s layout: every rotor is l from the CoM', ...
                layout{1}), ...
            max(abs(d - Pl.l)) < 1e-12, ...
            sprintf('arms %s, expected %.4f', ...
                mat2str(round(d, 4)), Pl.l)); %#ok<AGROW>

        res(end+1) = check( ...
            sprintf('%s layout: spin directions are balanced', ...
                layout{1}), ...
            sum(Pl.spinSign) == 0, ...
            sprintf('spinSign %s does not sum to zero', ...
                mat2str(Pl.spinSign))); %#ok<AGROW>
    end

    %% Round trip inside the envelope
    P = sim_droneParams('x');
    M = sim_QuadrotorPlant(P, zeros(12, 1));

    uReq = [P.m*P.g; 0.02; -0.015; 0.004];
    [w, uReal] = M.speedsFromWrench(uReq);

    res(end+1) = check('Unsaturated wrench round-trips exactly', ...
        max(abs(uReal - uReq)) < 1e-9, ...
        sprintf('max error %.3e', max(abs(uReal - uReq))));

    res(end+1) = check('Round-trip speeds stay inside the envelope', ...
        all(w >= P.wMin - 1e-9) && all(w <= P.wMax + 1e-9), ...
        sprintf('speeds %s outside [%.1f, %.1f]', ...
            mat2str(round(w, 2)), P.wMin, P.wMax));

    %% Saturation preserves the ROLL/PITCH direction
    % Naive clipping - what Project_2/3 did - fails this, because each
    % rotor clips by a different amount and the resulting moment points
    % somewhere else entirely.
    uBig = [P.m*P.g; 8.0; -6.0; 0.5];
    [wBig, uRealBig] = M.speedsFromWrench(uBig);

    d1 = uBig(2:3)     / norm(uBig(2:3));
    d2 = uRealBig(2:3) / norm(uRealBig(2:3));

    res(end+1) = check('Saturation preserves roll/pitch direction', ...
        norm(d1 - d2) < 1e-6, ...
        sprintf('requested %s, produced %s', ...
            mat2str(round(d1, 4)), mat2str(round(d2, 4))));

    res(end+1) = check('Saturated speeds stay inside the envelope', ...
        all(wBig >= P.wMin - 1e-9) && all(wBig <= P.wMax + 1e-9), ...
        sprintf('speeds %s outside [%.1f, %.1f]', ...
            mat2str(round(wBig, 2)), P.wMin, P.wMax));

    res(end+1) = check('Saturation reduces rather than inflates torque', ...
        norm(uRealBig(2:3)) <= norm(uBig(2:3)) + 1e-9, ...
        'Produced torque exceeds the requested torque.');

    %% Yaw is sacrificed before roll and pitch
    % This airframe makes only ~0.09 N m of yaw torque at hover thrust,
    % so yaw saturates routinely and must not steal tilt authority.
    uYaw = [P.m*P.g; 0.05; -0.04; 5.0];
    uNo  = [P.m*P.g; 0.05; -0.04; 0.0];

    [~, rYaw] = M.speedsFromWrench(uYaw);
    [~, rNo]  = M.speedsFromWrench(uNo);

    res(end+1) = check('Yaw saturation does not steal roll/pitch', ...
        max(abs(rYaw(2:3) - rNo(2:3))) < 1e-9, ...
        sprintf('roll/pitch %s with yaw demand vs %s without', ...
            mat2str(round(rYaw(2:3), 6)), mat2str(round(rNo(2:3), 6))));

    res(end+1) = check('Modest roll/pitch demand is met exactly', ...
        max(abs(rNo(1:3) - uNo(1:3))) < 1e-9, ...
        sprintf('max error %.3e', max(abs(rNo(1:3) - uNo(1:3)))));

    %% Envelope clamping
    wLow = M.speedsFromWrench([-50; 0; 0; 0]);
    res(end+1) = check('Negative thrust clamps to idle', ...
        all(abs(wLow - P.wMin) < 1e-9), ...
        sprintf('speeds %s, expected all %.2f', ...
            mat2str(round(wLow, 3)), P.wMin));

    wHigh = M.speedsFromWrench([1e4; 0; 0; 0]);
    res(end+1) = check('Excess thrust clamps to maximum', ...
        all(abs(wHigh - P.wMax) < 1e-9), ...
        sprintf('speeds %s, expected all %.2f', ...
            mat2str(round(wHigh, 3)), P.wMax));

    %% Envelope plausibility
    tw = 4 * P.k * P.wMax^2 / (P.m * P.g);
    res(end+1) = check('Thrust-to-weight ratio is in a sane range', ...
        tw > 1.5 && tw < 4.0, ...
        sprintf('T/W = %.2f, expected between 1.5 and 4', tw));

    res(end+1) = check('Finv is the inverse of F', ...
        max(max(abs(M.F * M.Finv - eye(4)))) < 1e-9, ...
        'F * Finv is not the identity.');
end

%% ------------------------------------------------------------------
%% Integration
%% ------------------------------------------------------------------
function res = integratorChecks()
    % integratorChecks: Order of accuracy and cost, measured.
    %
    % Defends the claim that RK4 at the physics rate is both more
    % accurate and cheaper than the Euler sub-stepping of Project_2/3.

    res = emptyResults();

    % y' = -2y, y(0) = 1, so y(t) = exp(-2t).
    f = @(t, y) -2*y;
    T = 1.0;
    exact = exp(-2*T);

    res(end+1) = check('sim_rk4Step converges on a linear problem', ...
        stepError(@sim_rk4Step, f, 1, T, 2000, exact) < 1e-12, ...
        sprintf('error %.3e at 2000 steps', ...
            stepError(@sim_rk4Step, f, 1, T, 2000, exact)));

    %% Measured order of accuracy
    % Halving the step should divide the error by 2 for Euler, 16 for RK4.
    pE = orderOf(@eulerStep, f, 1, T, exact, 200);
    res(end+1) = check('forward Euler is first order', ...
        abs(pE - 1) < 0.15, ...
        sprintf('measured order %.3f, expected 1', pE));

    pR = orderOf(@sim_rk4Step, f, 1, T, exact, 40);
    res(end+1) = check('sim_rk4Step is fourth order', ...
        abs(pR - 4) < 0.30, ...
        sprintf('measured order %.3f, expected 4', pR));

    %% RK4 at the new rate beats Euler at the old sub-step rate
    % Project_2 took 50 Euler sub-steps of 0.16 ms per 8 ms control
    % period: 6250 derivative evaluations per simulated second. The
    % replacement takes 500 RK4 steps of 2 ms: 2000 evaluations.
    eOld = stepError(@eulerStep, f, 1, T, 6250, exact);
    eNew = stepError(@sim_rk4Step,       f, 1, T,  500, exact);

    res(end+1) = check('RK4 at 500 Hz beats Euler at 6.25 kHz', ...
        eNew < eOld, ...
        sprintf('RK4 error %.3e vs Euler error %.3e', eNew, eOld));

    res(end+1) = check('RK4 does it with fewer derivative calls', ...
        4*500 < 6250, ...
        sprintf('%d calls vs %d', 4*500, 6250));

    %% Shape and hover stability under integration
    P = sim_droneParams('x');
    D = sim_QuadrotorPlant(P, zeros(12, 1));
    u = [P.m*P.g; 0; 0; 0];
    fd = @(t, x) D.modelDynamics(t, x, u);

    x1 = sim_rk4Step(fd, 0, 0.002, zeros(12, 1));
    res(end+1) = check('sim_rk4Step returns a 12x1 state', ...
        isequal(size(x1), [12 1]), ...
        sprintf('size %s', mat2str(size(x1))));

    % A drone at hover integrated for 2 s must not drift. Catches sign
    % errors that a single derivative call misses.
    x = zeros(12, 1);
    for i = 1:1000
        x = sim_rk4Step(fd, (i-1)*0.002, 0.002, x);
    end

    res(end+1) = check('Hover holds over 2 s of integration', ...
        max(abs(x)) < 1e-9, ...
        sprintf('max|x| = %.3e after 2 s', max(abs(x))));
end

%% ------------------------------------------------------------------
%% Closed loop
%% ------------------------------------------------------------------
function res = controlChecks()
    % controlChecks: Cascade behaviour, end to end.
    %
    % Runs the real loop headless - same plant, same controller, same
    % rates as RunSingleDrone. An end-to-end check is the only kind
    % that catches a sign error which is self-consistent within a module.

    res = emptyResults();

    P   = sim_droneParams('x');
    cfg = sim_config();
    C   = sim_CascadeController(P);

    %% Loop bandwidth separation
    res(end+1) = check('Attitude loop is faster than position loop', ...
        C.wnAtt >= 5 * C.wnPos, ...
        sprintf('wnAtt %.1f, wnPos %.1f, ratio %.1f (want >= 5)', ...
            C.wnAtt, C.wnPos, C.wnAtt/C.wnPos));

    res(end+1) = check('Attitude loop is faster than altitude loop', ...
        C.wnAtt >= 3 * C.wnAlt, ...
        sprintf('wnAtt %.1f, wnAlt %.1f, ratio %.1f (want >= 3)', ...
            C.wnAtt, C.wnAlt, C.wnAtt/C.wnAlt));

    %% Control rate supports the attitude bandwidth
    % Rule of thumb: sample at least 20x the closed-loop bandwidth.
    % Project_2 ran 125 Hz against 84 rad/s (13.4 Hz), a ratio of 9.
    ratio = cfg.physicsRate / (C.wnAtt / (2*pi));
    res(end+1) = check('Control rate is >= 20x attitude bandwidth', ...
        ratio >= 20, ...
        sprintf('rate ratio %.1f, want >= 20', ratio));

    %% Hover trim
    D = sim_QuadrotorPlant(P, [0; 0; 3; zeros(9, 1)]);
    C = sim_CascadeController(P);
    ref = struct('pos', [0; 0; 3], 'vel', zeros(3, 1), 'yaw', 0);

    C.outerLoop(D.xState, ref, 0.01);
    u = C.innerLoop(D);

    res(end+1) = check('On-reference thrust equals weight', ...
        abs(u(1) - P.m*P.g) < 1e-9, ...
        sprintf('T = %.6f N, expected %.6f N', u(1), P.m*P.g));

    res(end+1) = check('On-reference torques are zero', ...
        max(abs(u(2:4))) < 1e-9, ...
        sprintf('max|tau| = %.3e', max(abs(u(2:4)))));

    %% Convergence from the Project_2 initial condition
    % Yaw -90 deg, pitch 25.7 deg, roll 30 deg, commanded 3 m away and
    % 4 m up.
    x0  = [0; 0; 0; 0; 0; 0; -pi/2; pi/7; pi/6; 0; 0; 0];
    ref = struct('pos', [3; 2; 4], 'vel', zeros(3, 1), 'yaw', 0);
    r = flyHeadless(P, cfg, x0, 25, ref, []);

    res(end+1) = check('Closed loop produces no NaN or Inf', ...
        all(isfinite(r.x(:))), 'State went non-finite.');

    err = norm(r.x(1:3, end) - ref.pos);
    res(end+1) = check('Converges to the commanded position', ...
        err < 0.15, ...
        sprintf('final [%.3f %.3f %.3f], error %.3f m', ...
            r.x(1, end), r.x(2, end), r.x(3, end), err));

    res(end+1) = check('Levels off at the target', ...
        max(abs(r.x(8:9, end))) < deg2rad(2), ...
        sprintf('final pitch %.2f deg, roll %.2f deg', ...
            rad2deg(r.x(8, end)), rad2deg(r.x(9, end))));

    res(end+1) = check('Settles to rest', ...
        norm(r.x(4:6, end)) < 0.1, ...
        sprintf('final speed %.4f m/s', norm(r.x(4:6, end))));

    res(end+1) = check('Rotor speeds stay inside the envelope', ...
        all(r.w(:) >= P.wMin - 1e-6) && all(r.w(:) <= P.wMax + 1e-6), ...
        sprintf('speeds spanned [%.1f, %.1f], envelope [%.1f, %.1f]', ...
            min(r.w(:)), max(r.w(:)), P.wMin, P.wMax));

    peakTilt = max(max(abs(r.x(8:9, :))));
    res(end+1) = check('Tilt stays within the flight envelope', ...
        peakTilt < P.maxTilt + deg2rad(8), ...
        sprintf('peak tilt %.1f deg, limit %.1f deg', ...
            rad2deg(peakTilt), rad2deg(P.maxTilt)));

    res(end+1) = check('Altitude overshoot is under 25%', ...
        max(r.x(3, :)) < ref.pos(3)*1.25, ...
        sprintf('peak altitude %.2f m for a %.2f m command', ...
            max(r.x(3, :)), ref.pos(3)));

    % Starting at -90 deg with a 0 deg reference, the drone must turn
    % 90 deg, not 270. Project_2 omitted the wrap on the yaw error.
    res(end+1) = check('Yaw converges the short way round', ...
        abs(sim_wrapAngle(r.x(7, end))) < deg2rad(3), ...
        sprintf('final yaw %.2f deg', rad2deg(r.x(7, end))));

    res(end+1) = check('Altitude never goes below ground', ...
        min(r.x(3, :)) >= -1e-9, ...
        sprintf('minimum altitude %.4f m', min(r.x(3, :))));

    %% Manual mode with centred sticks is a stationary hover
    r = flyHeadless(P, cfg, [0; 0; 3; zeros(9, 1)], 8, [], sim_PilotInput.neutral());

    res(end+1) = check('Manual mode holds altitude on centred sticks', ...
        abs(r.x(3, end) - 3) < 0.25, ...
        sprintf('drifted from 3.00 m to %.3f m in 8 s', r.x(3, end)));

    res(end+1) = check('Manual mode stays level on centred sticks', ...
        max(abs(r.x(8:9, end))) < deg2rad(2), ...
        sprintf('final pitch %.2f deg, roll %.2f deg', ...
            rad2deg(r.x(8, end)), rad2deg(r.x(9, end))));
end

%% ------------------------------------------------------------------
%% Helpers
%% ------------------------------------------------------------------
function r = flyHeadless(P, cfg, x0, duration, ref, cmd)
    % flyHeadless: Run the cascade against the plant, no graphics.
    %
    % Mirrors the loop body of RunSingleDrone - inner loop every physics
    % step, outer loop every outerDivider steps - so the test exercises
    % the shipped rates rather than a simplified stand-in.
    %
    % Pass ref for automatic mode, or cmd for manual mode.

    D = sim_QuadrotorPlant(P, x0);
    C = sim_CascadeController(P);

    n = round(duration / cfg.dt);
    r.x = zeros(12, n);
    r.w = zeros(4, n);

    dtOuter = cfg.dt * cfg.outerDivider;

    for i = 1:n
        t = (i-1) * cfg.dt;

        if mod(i-1, cfg.outerDivider) == 0
            if isempty(cmd)
                C.outerLoop(D.xState, ref, dtOuter);
            else
                C.manualLoop(cmd, D.xState, dtOuter);
            end
        end

        D.advance(t, cfg.dt, C.innerLoop(D), cfg.groundLevel);

        r.x(:, i) = D.xState;
        r.w(:, i) = D.omegaMotor;
    end
end

function y = eulerStep(f, t, h, y)
    % eulerStep: One forward Euler step, kept only so the tests can
    % measure what Project_2/3's integrator actually cost.

    y = y + h * f(t, y);
end

function e = stepError(stepper, f, y0, T, nSteps, exact)
    % stepError: Absolute error after nSteps fixed steps.

    h = T / nSteps;
    y = y0;
    for i = 1:nSteps
        y = stepper(f, (i-1)*h, h, y);
    end
    e = abs(y - exact);
end

function p = orderOf(stepper, f, y0, T, exact, nSteps)
    % orderOf: log2 of the error ratio when the step is halved.

    e1 = stepError(stepper, f, y0, T, nSteps,   exact);
    e2 = stepError(stepper, f, y0, T, 2*nSteps, exact);
    p  = log2(e1 / e2);
end

function s = emptyResults()
    % emptyResults: 0x0 result struct array, so res(end+1) grows cleanly.

    s = struct('name', {}, 'pass', {}, 'detail', {});
end

function r = check(name, condition, detail)
    % check: Build one test result.
    %
    % Inputs:
    %        name - What is being asserted
    %   condition - Logical result of the assertion
    %      detail - Message shown on failure, ideally with actual and
    %               expected values

    r = struct('name', name, 'pass', logical(condition), ...
        'detail', detail);
end

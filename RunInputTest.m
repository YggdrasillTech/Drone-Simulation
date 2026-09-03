function RunInputTest()
    % RunInputTest: Live diagnostic for the pilot input device.
    %
    % Successor to Project_2/3's Controller_Test.m, which opened a
    % sim3d.io.Joystick, printed raw axis numbers in an infinite loop and
    % could only be stopped with Ctrl+C. It errored outright without the
    % toolbox, and printing raw numbers does not tell you which index is
    % which stick - the thing you opened it to find out.
    %
    % Push each stick or hold each key and confirm the matching bar moves
    % the way you expect. If a joystick axis is wrong, edit AXIS_MAP or
    % AXIS_SIGN in src/sim_PilotInput.m. Close the window to exit.

    % Put src/ on the path, resolved relative to this file so the
    % simulation runs from any working directory. A bootstrap cannot
    % live in the folder it bootstraps, so this is the one line that
    % stays at the root.
    addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

    fig = figure('Name', 'Input Device Test', 'NumberTitle', 'off', ...
        'MenuBar', 'none', 'ToolBar', 'none', ...
        'Position', [200 200 620 400]);

    try
        theme(fig, 'dark');
    catch
    end

    pilot  = sim_PilotInput(fig, 'auto');
    labels = {'roll', 'pitch', 'yawRate', 'throttle'};
    n = numel(labels);

    ax = axes('Parent', fig, 'Position', [0.13 0.32 0.80 0.56]);
    hold(ax, 'on');
    grid(ax, 'on');

    % One bar per axis, redrawn by moving its XData rather than by
    % clearing and replotting - the same reason the simulation uses
    % hgtransform instead of redrawing the drone.
    bars = gobjects(1, n);
    for i = 1:n
        bars(i) = patch(ax, 'XData', [0 0 0 0], ...
            'YData', [i-0.35, i+0.35, i+0.35, i-0.35], ...
            'FaceColor', [0.30 0.60 0.95], 'EdgeColor', 'none');
    end
    plot(ax, [0 0], [0.5 n+0.5], 'k-', 'LineWidth', 1);

    set(ax, 'XLim', [-1.1 1.1], 'YLim', [0.5 n+0.5], ...
        'YTick', 1:n, 'YTickLabel', labels, 'YDir', 'reverse');
    title(ax, sprintf('Device: %s', pilot.name), 'Interpreter', 'none');
    xlabel(ax, 'Normalised command [-1, 1]');

    txt = annotation(fig, 'textbox', [0.06 0.02 0.90 0.24], ...
        'String', '', 'EdgeColor', 'none', ...
        'FontName', 'monospaced', 'FontSize', 9, ...
        'Interpreter', 'none', 'VerticalAlignment', 'top');

    fprintf(['Move the sticks or press the keys. ' ...
             'Close the window to finish.\n']);

    % Polled at 50 Hz, the rate the simulation itself uses - the
    % original's fixed pause(0.5) was far too slow to see a stick move.
    while isvalid(fig)
        cmd = pilot.read();
        v = [cmd.roll, cmd.pitch, cmd.yawRate, cmd.throttle];

        for i = 1:n
            bars(i).XData = [0, 0, v(i), v(i)];
        end

        txt.String = sprintf([ ...
            'roll %+5.2f   pitch %+5.2f   yaw %+5.2f   thr %+5.2f\n' ...
            'events: mode=%d  reset=%d  quit=%d\n' ...
            'keys: arrows = pitch/roll, W/S = climb, A/D = yaw,\n' ...
            '      M = mode, R = reset, Q = quit'], ...
            v(1), v(2), v(3), v(4), ...
            cmd.modeToggle, cmd.reset, cmd.quit);

        if cmd.quit
            break;
        end

        drawnow limitrate;
        pause(0.02);
    end

    pilot.close();

    if isvalid(fig)
        close(fig);
    end
end

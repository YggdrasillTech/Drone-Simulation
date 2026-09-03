classdef sim_Window < handle
    % sim_Window: Figure, 3D view and scrolling telemetry.
    %
    % Data-driven. Project_2's version declared 25 individually named
    % animatedline properties (hAnimXPos, hAnimYPos, hAnimZPos,
    % hAnimPsiAng, ...) and repeated every operation once per handle: 25
    % addpoints calls, a 20-line clearpoints block and seven near
    % identical XLim/XTick updates. Adding one signal meant editing four
    % places. Here a plot is a row of a spec table, so creating,
    % scrolling and updating are each written once.
    %
    % TWO THINGS THIS FILE GETS RIGHT THAT ARE EASY TO GET WRONG.
    %
    % 1. Rotation is enabled through axes Interactions, NOT rotate3d.
    %    rotate3d(ax,'on') activates a figure interaction MODE, and while
    %    one is active MATLAB refuses to set KeyPressFcn:
    %      "Setting the KeyPressFcn property is not permitted while this
    %       mode is active"
    %    so keyboard flight silently does nothing. Interactions give the
    %    same drag-to-rotate without owning the figure's key handling.
    %
    % 2. The time window JUMPS rather than sliding. Sliding it means
    %    writing XLim on seven axes on every plot update, and each write
    %    forces that axes to re-render. At 20 Hz that was 140 relayouts a
    %    second and it was the single reason a one-drone run could not
    %    hold real time while a nine-drone run could. Jumping writes XLim
    %    three times in a 30 s run instead of 4200.

    properties (SetAccess = private)
        wnd         % Figure handle
        axSim       % 3D simulation axes
    end

    properties (Access = private)
        plots       % Struct array: ax, lines, idx, scale
        hTrace      % Flight trace animatedline
        hStatus     % Heads-up text object
        tWindow     % Current right edge of the plot window [s]
        hasPlots
    end

    properties (Constant, Access = private)
        T_SPAN = 10;        % [s] plot window width
        MAX_PTS = 400;

        % Trace colours by pilot mode
        COLOR_AUTO   = [0.95 0.75 0.30];
        COLOR_MANUAL = [0.72 0.50 0.95];

        % Palette chosen to read on both dark and light backgrounds -
        % 'y' and 'c' in the original are close to invisible on white.
        C1 = [0.95 0.45 0.45];
        C2 = [0.45 0.85 0.55];
        C3 = [0.40 0.68 1.00];
        C4 = [0.95 0.78 0.35];
    end

    methods
        %% Construction
        function obj = sim_Window(mode, titleText, themeName)
            % Create the figure and its axes.
            %
            % Inputs:
            %        mode - 'full' for 3D plus telemetry, 'sim' for the
            %               3D view alone (swarm runs, where per-drone
            %               telemetry is meaningless)
            %   titleText - Figure name
            %   themeName - 'dark' or 'light'

            arguments (Input)
                mode      (1,:) char = 'full'
                titleText (1,:) char = 'Drone Simulation'
                themeName (1,:) char = 'dark'
            end

            obj.wnd = figure('Name', titleText, 'NumberTitle', 'off', ...
                'MenuBar', 'none', 'ToolBar', 'none');
            obj.wnd.WindowState = 'maximized';

            % MATLAB's own theme, so every axes, label and tick follows
            % it without being coloured by hand. R2025a and later.
            try
                theme(obj.wnd, themeName);
            catch
                % Older release: leave the default look alone.
            end

            obj.hasPlots = strcmpi(mode, 'full');
            obj.tWindow  = obj.T_SPAN;

            if obj.hasPlots
                grid_ = [7 3];
                simCells = [1 2 4 5 7 8 10 11 13 14 16 17 19 20];
            else
                grid_ = [1 1];
                simCells = 1;
            end

            if obj.hasPlots
                obj.buildPlots(grid_);
            end

            obj.buildSimAxes(grid_, simCells);
        end

        %% Telemetry update
        function update(obj, t, x, err, omega, pilotMode)
            % update: Push one telemetry sample to the plots.
            %
            % Inputs:
            %           t - Simulated time [s]
            %           x - 12x1 state vector
            %         err - 12x1 state error
            %       omega - 4x1 rotor speeds [rad/s]
            %   pilotMode - 0 auto, 1 manual (selects trace colour)

            if ~isvalid(obj.wnd)
                return;
            end

            if pilotMode == 1
                c = obj.COLOR_MANUAL;
            else
                c = obj.COLOR_AUTO;
            end

            % Writing a graphics property costs far more than comparing
            % one, and this runs on every plot update.
            if ~isequal(obj.hTrace.Color, c)
                obj.hTrace.Color = c;
            end

            addpoints(obj.hTrace, x(1), x(2), x(3));

            if ~obj.hasPlots
                return;
            end

            % One flat sample vector so each plot names its own slice.
            sample = [x(:); err(:); omega(:)];

            % Jump the window only when time leaves it. Points older than
            % the window are dropped, which is also what keeps memory
            % bounded without MaximumNumPoints doing all the work.
            if t > obj.tWindow
                obj.tWindow = t + obj.T_SPAN;
                for i = 1:numel(obj.plots)
                    obj.plots(i).ax.XLim = ...
                        [t, obj.tWindow];
                    for h = obj.plots(i).lines
                        clearpoints(h);
                    end
                end
            end

            for i = 1:numel(obj.plots)
                pl = obj.plots(i);
                for j = 1:numel(pl.idx)
                    addpoints(pl.lines(j), t, sample(pl.idx(j))*pl.scale);
                end
            end
        end

        %% Heads-up text
        function setStatus(obj, txt)
            % setStatus: Replace the heads-up text.
            %
            % A text object inside the axes, not an annotation textbox:
            % annotations re-lay-out the whole figure on every string
            % change and were costing several milliseconds a frame.

            if isvalid(obj.wnd)
                obj.hStatus.String = txt;
            end
        end

        %% Reset
        function clearTrace(obj)
            % clearTrace: Drop the drawn trajectory.
            clearpoints(obj.hTrace);
        end

        %% Window state
        function tf = isOpen(obj)
            % isOpen: Whether the simulation figure still exists.
            tf = isvalid(obj.wnd);
        end
    end

    methods (Access = private)
        %% 3D axes
        function buildSimAxes(obj, grid_, cells)
            % buildSimAxes: The 3D world, its lighting and its ground.

            ax = subplot(grid_(1), grid_(2), cells);
            obj.axSim = ax;

            title(ax, 'Drone 3D Simulation', 'Interpreter', 'latex', ...
                'FontSize', 9);

            axis(ax, 'manual');
            set(ax, 'XLim', [-10 10], 'YLim', [-10 10], 'ZLim', [0 10], ...
                'XTick', -10:2:10, 'YTick', -10:2:10, 'ZTick', 0:2:10, ...
                'XLimMode', 'manual', 'YLimMode', 'manual', ...
                'ZLimMode', 'manual');

            view(ax, 3);
            % Set reverse direction for XY
            set(ax, 'XDir', 'reverse', 'YDir', 'reverse');
            grid(ax, 'on');
            hold(ax, 'on');

            % Drag to rotate, scroll to zoom - WITHOUT taking over the
            % figure's key handling the way rotate3d would. See the note
            % in the class header.
            try
                ax.Interactions = [rotateInteraction, zoomInteraction];
            catch
                % Older release: fall back to the default interactivity.
            end

            % Apply ilumination
            camlight(ax, 'headlight');
            lighting(ax, 'gouraud');

            %% Inertial coordinate axes at the origin
            line(ax, [0 0.5], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);
            line(ax, [0 0], [0 0.5], [0 0], 'Color', 'g', 'LineWidth', 2);
            line(ax, [0 0], [0 0], [0 0.5], 'Color', 'b', 'LineWidth', 2);

            % Ground plane, so altitude reads at a glance. Mid grey with
            % low alpha works against either theme.
            patch(ax, 'XData', [-10 10 10 -10], ...
                'YData', [-10 -10 10 10], 'ZData', zeros(1, 4), ...
                'FaceColor', [0.5 0.55 0.6], 'FaceAlpha', 0.18, ...
                'EdgeColor', 'none');

            obj.hTrace = animatedline('Color', obj.COLOR_AUTO, ...
                'LineWidth', 1.5, 'MaximumNumPoints', 1200, 'Parent', ax);

            obj.hStatus = text(ax, 0.015, 0.985, '', ...
                'Units', 'normalized', 'FontName', 'monospaced', ...
                'FontSize', 9, 'VerticalAlignment', 'top', ...
                'Interpreter', 'none');
        end

        %% Telemetry plots
        function buildPlots(obj, grid_)
            % buildPlots: Create every scrolling plot from one spec.
            %
            % Indices refer to the concatenated sample vector
            %   [ state(1:12); error(1:12); omega(1:4) ]
            % so 1:12 is the state, 13:24 the error, 25:28 rotor speeds.

            rgb = [obj.C1; obj.C2; obj.C3];
            mot = [obj.C1; obj.C2; obj.C3; obj.C4];

            % Columns: cell, title, y-label, ylim, ytick, sample
            % indices, colour rows, unit scale.
            spec = { ...
                3,  '$XYZ$ Position', 'Position [m]', ...
                    [-10 10], -10:5:10, 1:3, rgb, 1; ...
                6,  '$\psi \, \theta \, \phi$ Angle', 'Angle [deg]', ...
                    [-180 180], -180:90:180, 7:9, rgb, 180/pi; ...
                9,  '$XYZ$ Velocity', 'Velocity [m/s]', ...
                    [-10 10], -10:5:10, 4:6, rgb, 1; ...
                12, '$p \, q \, r$ Angular Velocity', 'Rate [deg/s]', ...
                    [-360 360], -360:180:360, 10:12, rgb, 180/pi; ...
                15, '$\omega$ Motor Velocity', 'Speed [rad/s]', ...
                    [0 1000], 0:250:1000, 25:28, mot, 1; ...
                18, '$XYZ$ Error', 'Error [m]', ...
                    [-10 10], -10:5:10, 13:15, rgb, 1; ...
                21, '$\psi \, \theta \, \phi$ Angle Error', ...
                    'Error [deg]', ...
                    [-180 180], -180:90:180, 19:21, rgb, 180/pi };

            n = size(spec, 1);
            obj.plots = repmat( ...
                struct('ax', [], 'lines', [], 'idx', [], 'scale', 1), 1, n);

            for i = 1:n
                ax = subplot(grid_(1), grid_(2), spec{i, 1});

                title(ax, spec{i, 2}, 'Interpreter', 'latex', ...
                    'FontSize', 8);
                ylabel(ax, spec{i, 3}, 'FontSize', 7);
                xlabel(ax, 'Time [s]', 'FontSize', 7);

                set(ax, 'YLim', spec{i, 4}, 'YTick', spec{i, 5}, ...
                    'XLim', [0 obj.T_SPAN], ...
                    'XLimMode', 'manual', 'YLimMode', 'manual');
                grid(ax, 'on');
                hold(ax, 'on');

                % Interaction callbacks are a real bottleneck on axes
                % that redraw many times a second, and these are for
                % reading, not poking at.
                ax.Interactions = [];
                ax.PickableParts = 'none';
                ax.HitTest = 'off';

                idx    = spec{i, 6};
                colors = spec{i, 7};

                lines = gobjects(1, numel(idx));
                for j = 1:numel(idx)
                    lines(j) = animatedline('Color', colors(j, :), ...
                        'MaximumNumPoints', obj.MAX_PTS, 'Parent', ax);
                end

                obj.plots(i).ax    = ax;
                obj.plots(i).lines = lines;
                obj.plots(i).idx   = idx;
                obj.plots(i).scale = spec{i, 8};
            end
        end
    end
end

classdef sim_PilotInput < handle
    % sim_PilotInput: Manual flight input, from a joystick or the keyboard.
    %
    %   Arrow Up / Down     pitch forward / backward
    %   Arrow Left / Right  roll left / right
    %   W / S               climb / descend
    %   A / D               yaw left / right
    %   M                   toggle manual <-> auto
    %   R                   reset
    %   Q or Escape         quit
    %
    % One class rather than an abstract base plus two subclasses. The
    % polymorphism was only ever used in two call sites and there will
    % never be a third input source here, so the hierarchy cost three
    % files to buy nothing.
    %
    % read() always returns the same normalised command struct, so the
    % controller neither knows nor cares which source is active:
    %
    %   roll      [-1, 1]  right positive
    %   pitch     [-1, 1]  forward positive
    %   yawRate   [-1, 1]  clockwise-from-above positive
    %   throttle  [-1, 1]  climb positive, 0 = hold altitude
    %   modeToggle / reset / quit   logical, true only on the frame the
    %                               key or button went down
    %
    % Two problems have to be solved for a keyboard to work as a flight
    % control at all, and both are handled here.
    %
    %   1. Auto-repeat. Holding a key does not give one KeyPress and one
    %      KeyRelease; X11 synthesises a release/press pair for every
    %      repeat, so clearing the key on KeyRelease makes a held stick
    %      stutter. A release is only believed if no press follows it
    %      within RELEASE_DEBOUNCE seconds.
    %
    %   2. A key is binary but an attitude command is not. Each axis
    %      ramps toward its target rather than snapping, and springs back
    %      to centre when released, which makes an on/off key
    %      proportional enough to hover with.

    properties (SetAccess = private)
        name = 'none'       % Human-readable device description
        usingJoystick = false
    end

    properties (Access = private)
        fig                 % Figure the keyboard callbacks live on
        joy                 % sim3d.io.Joystick handle, if any
        prevButtons         % Button state at the previous read

        keyDown             % containers.Map, key -> logical held
        releaseAt           % containers.Map, key -> pending release time
        clockStart
        lastRead = 0

        ax = zeros(1, 4)    % Smoothed [roll pitch yaw throttle]
        pending = false(1, 3)   % [modeToggle reset quit] latches
    end

    properties (Constant, Access = private)
        RELEASE_DEBOUNCE = 0.06;   % [s] auto-repeat gap tolerance
        RAMP_RATE        = 3.0;    % [1/s] toward a held direction
        CENTER_RATE      = 5.0;    % [1/s] back to centre when released
        DEADZONE         = 0.08;   % joystick only
        EXPO             = 0.35;   % joystick only

        % Standard twin-stick gamepad layout: [roll pitch yaw throttle]
        % axis indices, and the sign each needs to match the convention
        % above. Most sticks report forward/up as negative. Run RunInputTest
        % to see live values if your controller differs.
        AXIS_MAP  = [1 2 3 4];
        AXIS_SIGN = [1 -1 1 -1];

        % [modeToggle reset quit] button indices
        BUTTON_MAP = [1 2 8];
    end

    methods
        %% Construction
        function obj = sim_PilotInput(fig, preference)
            % Open the best available input device.
            %
            % Inputs:
            %          fig - Figure handle for keyboard callbacks
            %   preference - 'auto' (joystick if present, else keyboard),
            %                'joystick', or 'keyboard'
            %
            % Manual control is never simply unavailable: Project_2/3
            % called sim3d.io.Joystick with no try/catch, so without that
            % toolbox and a physical stick the simulation threw before
            % the first frame and the manual path went untested.

            arguments (Input)
                fig
                preference (1,:) char = 'auto'
            end

            obj.fig        = fig;
            obj.clockStart = tic;

            if ~strcmpi(preference, 'keyboard')
                obj.usingJoystick = obj.openJoystick();
                if ~obj.usingJoystick && strcmpi(preference, 'joystick')
                    error('sim_PilotInput:noJoystick', ...
                        'No joystick found (sim3d.io.Joystick).');
                end
            end

            if obj.usingJoystick
                obj.name = 'Joystick (sim3d.io.Joystick)';
                obj.prevButtons = false(1, 32);
            else
                obj.name = 'Keyboard (WASD + arrows)';
                obj.attachKeyboard();
            end

            fprintf('Pilot input: %s\n', obj.name);
        end

        %% Poll
        function cmd = read(obj)
            % read: Sample the device and return the command struct.
            %
            % Output:
            %   cmd - Normalised command struct, see the class header

            if obj.usingJoystick
                obj.readJoystick();
            else
                obj.readKeyboard();
            end

            cmd = struct( ...
                'roll',       obj.ax(1), ...
                'pitch',      obj.ax(2), ...
                'yawRate',    obj.ax(3), ...
                'throttle',   obj.ax(4), ...
                'modeToggle', obj.pending(1), ...
                'reset',      obj.pending(2), ...
                'quit',       obj.pending(3));

            obj.pending(:) = false;
        end

        %% Release resources
        function close(obj)
            % close: Detach callbacks and release the device.

            if obj.usingJoystick
                try
                    delete(obj.joy);
                catch
                    % Already gone; nothing useful to do.
                end
            elseif isgraphics(obj.fig)
                set(obj.fig, 'KeyPressFcn', '', 'KeyReleaseFcn', '');
            end
        end
    end

    methods (Static)
        %% Neutral command
        function cmd = neutral()
            % neutral: Command struct with everything centred.

            cmd = struct('roll', 0, 'pitch', 0, 'yawRate', 0, ...
                'throttle', 0, 'modeToggle', false, ...
                'reset', false, 'quit', false);
        end
    end

    methods (Access = private)
        %% Joystick setup
        function ok = openJoystick(obj)
            % openJoystick: Probe for a joystick without noise.
            %
            % Warnings are suppressed around the probe because failing to
            % find a device is a normal outcome here, not something the
            % user needs three stack traces about.

            ok = false;
            ws = warning('off', 'all');
            cleanup = onCleanup(@() warning(ws));

            try
                obj.joy = sim3d.io.Joystick();
                % Constructing can succeed with no device attached, so
                % prove it answers before committing to it.
                read(obj.joy);
                ok = true;
            catch
                obj.joy = [];
            end
        end

        %% Joystick poll
        function readJoystick(obj)
            % readJoystick: Sample axes and edge-detect buttons.
            %
            % A read failure - controller unplugged mid-flight - leaves
            % the axes centred rather than raising, so losing the stick
            % makes the drone level off instead of crashing the run.

            try
                [axes, buttons, ~] = read(obj.joy);
            catch
                obj.ax(:) = 0;
                return;
            end

            for i = 1:4
                idx = obj.AXIS_MAP(i);
                if idx <= numel(axes)
                    obj.ax(i) = obj.shapeAxis( ...
                        obj.AXIS_SIGN(i) * double(axes(idx)));
                else
                    obj.ax(i) = 0;
                end
            end

            b = false(1, numel(obj.prevButtons));
            n = min(numel(buttons), numel(b));
            b(1:n) = logical(buttons(1:n));

            % Edge-triggered, so holding a button does not fire the
            % action every frame.
            for i = 1:3
                idx = obj.BUTTON_MAP(i);
                obj.pending(i) = obj.pending(i) || ...
                    (idx <= numel(b) && b(idx) && ~obj.prevButtons(idx));
            end

            obj.prevButtons = b;
        end

        %% Axis conditioning
        function v = shapeAxis(obj, v)
            % shapeAxis: Deadzone and exponential response.
            %
            % The deadzone is rescaled rather than subtracted, so the
            % axis still reaches +-1 at full deflection. Expo trades
            % precision near centre against reach at the extremes, which
            % is what makes a coarse stick usable for fine hovering.

            v = sim_saturate(v, -1, 1);
            mag = abs(v);

            if mag < obj.DEADZONE
                v = 0;
                return;
            end

            mag = (mag - obj.DEADZONE) / (1 - obj.DEADZONE);
            mag = (1 - obj.EXPO)*mag + obj.EXPO*mag^3;

            v = sign(v) * mag;
        end

        %% Keyboard setup
        function attachKeyboard(obj)
            % attachKeyboard: Wire the figure key callbacks.
            %
            % These are refused outright while a figure interaction MODE
            % is active - rotate3d, pan, zoom all set one. That is why
            % sim_Window enables rotation through axes Interactions
            % instead: with rotate3d(ax,'on') MATLAB warns
            % "Setting the KeyPressFcn property is not permitted while
            % this mode is active" and keyboard flight silently does
            % nothing.

            obj.keyDown   = containers.Map('KeyType', 'char', ...
                'ValueType', 'logical');
            obj.releaseAt = containers.Map('KeyType', 'char', ...
                'ValueType', 'double');

            set(obj.fig, ...
                'KeyPressFcn',   @(~, e) obj.onKeyPress(e), ...
                'KeyReleaseFcn', @(~, e) obj.onKeyRelease(e));
        end

        %% Keyboard poll
        function readKeyboard(obj)
            % readKeyboard: Settle releases and advance the axis ramps.

            tNow = toc(obj.clockStart);
            dt   = sim_saturate(tNow - obj.lastRead, 0, 0.1);
            obj.lastRead = tNow;

            % Clear keys whose release has outlived the auto-repeat gap.
            keys = obj.releaseAt.keys;
            for i = 1:numel(keys)
                k = keys{i};
                if tNow - obj.releaseAt(k) >= obj.RELEASE_DEBOUNCE
                    obj.keyDown(k) = false;
                    remove(obj.releaseAt, k);
                end
            end

            target = [ obj.held('rightarrow') - obj.held('leftarrow'), ...
                       obj.held('uparrow')    - obj.held('downarrow'), ...
                       obj.held('d')          - obj.held('a'), ...
                       obj.held('w')          - obj.held('s') ];

            % No deadzone or expo here: those condition a noisy analogue
            % axis, and a ramped key is neither. The ramp is the
            % conditioning. Springing back to centre faster than it
            % deflects makes the control feel like a stick, not a latch.
            for i = 1:4
                if target(i) == 0
                    step = obj.CENTER_RATE * dt;
                else
                    step = obj.RAMP_RATE * dt;
                end

                d = target(i) - obj.ax(i);
                if abs(d) <= step
                    obj.ax(i) = target(i);
                else
                    obj.ax(i) = obj.ax(i) + sign(d)*step;
                end
            end
        end

        %% Held test
        function tf = held(obj, k)
            % held: 1 if key k is currently down, 0 otherwise.

            tf = double(isKey(obj.keyDown, k) && obj.keyDown(k));
        end

        %% Key press callback
        function onKeyPress(obj, e)
            % onKeyPress: Latch a key down and cancel any pending
            % release, which is what defeats auto-repeat.

            obj.keyDown(e.Key) = true;
            if isKey(obj.releaseAt, e.Key)
                remove(obj.releaseAt, e.Key);
            end

            % Edge-triggered actions fire on press, not on hold.
            switch e.Key
                case 'm'
                    obj.pending(1) = true;
                case 'r'
                    obj.pending(2) = true;
                case {'q', 'escape'}
                    obj.pending(3) = true;
            end
        end

        %% Key release callback
        function onKeyRelease(obj, e)
            % onKeyRelease: Record a provisional release, acted on only
            % once RELEASE_DEBOUNCE has passed with no press.

            obj.releaseAt(e.Key) = toc(obj.clockStart);
        end
    end
end

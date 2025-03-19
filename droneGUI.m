classdef droneGUI < handle
    properties
        % Window handler
        wnd
    end
    
    properties(Access = private)
        % Window Structure
        gridSize
        % Time Windows in Simulation
        t_span
        t_wnd
        % Subplot for Position and Euler Angles
        axXYZPos
        axAng
        % Subplot for Linear and Angular Velocities
        axXYZVel
        axAngVel
        % Subplot for Motor Speed
        axOmegaMotor
        % 3D Simulation
        axSim
        % Animated Line for Position
        hAnimXPos
        hAnimYPos
        hAnimZPos
        % Animated Line for Euler Angles
        hAnimPsiAng
        hAnimThetaAng
        hAnimPhiAng
        % Animated Line for Linear Velocities
        hAnimXVel
        hAnimYVel
        hAnimZVel
        % Animated Line for Angular Velocities
        hAnimPAngVel
        hAnimQAngVel
        hAnimRAngVel
        % Animated Line for Motor Speeds
        hAnimOmegaMotor1
        hAnimOmegaMotor2
        hAnimOmegaMotor3
        hAnimOmegaMotor4
        % Drone trace trajectory
        hAnimXYZPos
    end

    methods
        function obj = droneGUI(~)
            %% Create Figure and Subplots for Simulation and Graphs
            obj.wnd = figure('Name', 'Drone Simulation',...
                'NumberTitle', 'off');
            obj.wnd.WindowState = 'maximized';

            % Use OpenGL renderer for improved performance
            set(gcf, 'Renderer', 'OpenGL');
            % Disable the resize callback if not needed
            set(gcf, 'SizeChangedFcn', '');

            % Disable ToolBar and MenuBar
            set(obj.wnd, 'MenuBar', 'none');
            set(obj.wnd, 'ToolBar', 'none');

            %% Setting Plot Layout
            obj.gridSize = [5 3];

            %% Time intervals
            obj.t_span = 10; % Span in which plot are going to be drawn
            obj.t_wnd  = 10; % Initial time window for the plots
            
            %% Create Plots
            % XY Position Graph
            obj.axXYZPos = createPlot(obj, 3, '$XYZ$ Position',...
                'Position', [-10 10], -10:5:10, 'Time',...
                [0, obj.t_wnd], 0:1:obj.t_wnd);
            % Euler Angle Graph
            obj.axAng = createPlot(obj, 6,...
                '$\psi \, \theta \, \phi$ Angle',...
                'Angle', [-pi pi], -pi:pi/2:pi, 'Time',...
                [0, obj.t_wnd], 0:1:obj.t_wnd);
            % XYZ Velocity Graph
            obj.axXYZVel = createPlot(obj, 9, '$XYZ$ Velocity',...
                'Velocity', [-10 10], -10:5:10, 'Time',...
                [0, obj.t_wnd], 0:1:obj.t_wnd);
            % Angular Velocity Graph
            obj.axAngVel = createPlot(obj, 12,...
                '$p \, q \, r$ Angular Velocity',...
                'Angular Velocity', [-2*pi 2*pi], -2*pi:pi:2*pi,...
                'Time', [0, obj.t_wnd], 0:1:obj.t_wnd);
            % Motor Speed Graph
            obj.axOmegaMotor = createPlot(obj, 15,...
                '$\omega$ Motor Velocity',...
                'Motor Speed', [250 950], 250:175:950,...
                'Time', [0, obj.t_wnd], 0:1:obj.t_wnd);
            % Robot Simulation Graph (showing the robot and its path)
            obj.axSim = createPlot(obj, [1 2 4 5 7 8 10 11 13 14],...
                'Drone 3D Simulation', '', [-10 10], -10:2:10, '',...
                [-10 10], -10:2:10);
            % Set the view to 3D
            view(obj.axSim, 3);
            % Set reverse direction for XY
            set(obj.axSim, 'XDir', 'reverse', 'YDir', 'reverse');

            %% Set Z-Axis Properties
            % Set the Z-axis limits, label, and tick marks.
            obj.axSim.ZLim = [0 10];
            % Assign ticks in Z
            obj.axSim.ZTick = 0:1:10;
            % Format Z-axis tick labels to one decimal place.
            ztickformat(obj.axSim, "%.0f"); 
            % Use a tight limit method to improve visualization 
            % of the data.
            obj.axSim.ZLimitMethod = 'tight';
            % Set manual Z axis handling.
            set(obj.axSim, 'ZLimMode', 'manual');

            %% Draw coordinate axes for the drone.
            % Draw x-axis (red)
            line(obj.axSim, [0, 0.5], [0, 0], [0, 0],...
                'Color', 'r', 'LineWidth', 2);
            % Draw y-axis (green)
            line(obj.axSim, [0, 0], [0, 0.5], [0, 0],...
                'Color', 'g', 'LineWidth', 2);
            % Draw z-axis (blue)
            line(obj.axSim, [0, 0], [0, 0], [0, 0.5],...
                'Color', 'b', 'LineWidth', 2);
            
            %% Creating Animated Lines for plotting State Information
            % Animated Line for Position
            obj.hAnimXPos = animatedline('Color', 'r',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZPos);
            obj.hAnimYPos = animatedline('Color', 'g',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZPos);
            obj.hAnimZPos = animatedline('Color', 'b',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZPos);
            % Animated Line for Euler Angles
            obj.hAnimPsiAng = animatedline('Color', 'c',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAng);
            obj.hAnimThetaAng = animatedline('Color', 'y',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAng);
            obj.hAnimPhiAng = animatedline('Color', 'm',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAng);
            % Animated Line for Linear Velocities
            obj.hAnimXVel = animatedline('Color', 'r',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZVel);
            obj.hAnimYVel = animatedline('Color', 'g',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZVel); 
            obj.hAnimZVel = animatedline('Color', 'b',...
                'MaximumNumPoints', 1400, 'Parent', obj.axXYZVel);
            % Animated Line for Angular Velocities
            obj.hAnimPAngVel = animatedline('Color', 'm',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAngVel);
            obj.hAnimQAngVel = animatedline('Color', 'y',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAngVel);
            obj.hAnimRAngVel = animatedline('Color', 'c',...
                'MaximumNumPoints', 1400, 'Parent', obj.axAngVel);

            % Animated Line for Motor Speeds
            obj.hAnimOmegaMotor1 = animatedline('Color', 'm',...
                'MaximumNumPoints', 1400, 'Parent', obj.axOmegaMotor);
            obj.hAnimOmegaMotor2 = animatedline('Color', 'y',...
                'MaximumNumPoints', 1400, 'Parent', obj.axOmegaMotor);
            obj.hAnimOmegaMotor3 = animatedline('Color', 'c',...
                'MaximumNumPoints', 1400, 'Parent', obj.axOmegaMotor);
            obj.hAnimOmegaMotor4 = animatedline('Color', 'r',...
                'MaximumNumPoints', 1400, 'Parent', obj.axOmegaMotor);

            % Initialize robot trace animation
            obj.hAnimXYZPos = animatedline('Color',...
                [0.9290 0.6940 0.1250], 'MaximumNumPoints', 700,...
                'Parent', obj.axSim);
        end

        % Update Graphics
        function updateGraphics(obj, t, x, omega, pilotMode)
            % Check if the simulation window is still open
            if ishandle(obj.wnd)
                %
                obj.flightTrail(pilotMode);
                % Dynamically adjust the time window of the plots 
                % if necessary
                if t > obj.t_wnd
                    obj.t_wnd = t + obj.t_span;
                    % Ste new time wdindow fro plots
                    obj.axXYZPos.XLim = [t, obj.t_wnd];
                    obj.axAng.XLim = [t, obj.t_wnd];
                    obj.axXYZVel.XLim = [t, obj.t_wnd];
                    obj.axAngVel.XLim = [t, obj.t_wnd];
                    obj.axOmegaMotor.XLim = [t, obj.t_wnd];
                    % Set new ticks
                    obj.axXYZPos.XTick = t:1:obj.t_wnd;
                    obj.axAng.XTick = t:1:obj.t_wnd;
                    obj.axXYZVel.XTick = t:1:obj.t_wnd;
                    obj.axAngVel.XTick = t:1:obj.t_wnd;
                    obj.axOmegaMotor.XTick = t:1:obj.t_wnd;
                    %% Clear Points since are no longer visible
                    % Clear Position
                    clearpoints(obj.hAnimXPos);
                    clearpoints(obj.hAnimYPos);
                    clearpoints(obj.hAnimZPos);
                    % Clear Euler Angles
                    clearpoints(obj.hAnimPsiAng);
                    clearpoints(obj.hAnimThetaAng);
                    clearpoints(obj.hAnimPhiAng);
                    % Clear Linear Velocities
                    clearpoints(obj.hAnimXVel);
                    clearpoints(obj.hAnimYVel);
                    clearpoints(obj.hAnimZVel);
                    % Clear Angular Velocities
                    clearpoints(obj.hAnimPAngVel);
                    clearpoints(obj.hAnimQAngVel);
                    clearpoints(obj.hAnimRAngVel);
                end
                % Append new data points to the animated lines 
                % Plot Position
                addpoints(obj.hAnimXPos, t, x(1));
                addpoints(obj.hAnimYPos, t, x(2));
                addpoints(obj.hAnimZPos, t, x(3));
                % Plot Euler Angles
                addpoints(obj.hAnimPsiAng, t, x(7));
                addpoints(obj.hAnimThetaAng, t, x(8));
                addpoints(obj.hAnimPhiAng, t, x(9));
                % Plot Linear Velocities
                addpoints(obj.hAnimXVel, t, x(4));
                addpoints(obj.hAnimYVel, t, x(5));
                addpoints(obj.hAnimZVel, t, x(6));
                % Plot Angular Velocities
                addpoints(obj.hAnimPAngVel, t, x(10));
                addpoints(obj.hAnimQAngVel, t, x(11));
                addpoints(obj.hAnimRAngVel, t, x(12));
                % Plot Motor Speed
                addpoints(obj.hAnimOmegaMotor1, t, omega(1));
                addpoints(obj.hAnimOmegaMotor2, t, omega(2));
                addpoints(obj.hAnimOmegaMotor3, t, omega(3));
                addpoints(obj.hAnimOmegaMotor4, t, omega(4));
                % Plot Trace Line for the Drone
                addpoints(obj.hAnimXYZPos, x(1), x(2), x(3));
            end
        end
        
        % Change Trace color Function
        function flightTrail(obj, driveMode)
            % Identify driving mode
            if driveMode == 0
                obj.hAnimXYZPos.Color = [0.9290 0.6940 0.1250];
            else
                obj.hAnimXYZPos.Color = [0.4940 0.1840 0.5560];
            end
        end
        
        % Get Window Handler
        function axSim = getSimHandler(obj)
            axSim = obj.axSim;
        end
    end

    methods (Access = private)
        %% Encapsulation for Creating Plots
        function ax = createPlot(obj, gridPos, titleText,...
                yLabelText, yLimits, yTicks, xLabelText, xLimits, xTicks)
            % This function encapsulates the creation of a subplot 
            % with specific settings, including title, axis labels, 
            % limits, tick configuration, and grid display. It supports
            % LaTeX formatting for the title and Y-axis label. 
            % Additionally, it disables interactive features and 
            % sets manual limits to optimize rendering performance.
            %
            % Inputs:
            %     gridSize - Dimensions of the subplot grid.
            %      gridPos - Position index for the subplot.
            %    titleText - Title of the plot.
            %   yLabelText - Label for the Y-axis.
            %      yLimits - Y-axis limits [y_min, y_max].
            %       yTicks - Y-axis tick positions.
            %      xLimits - X-axis limits [x_min, x_max].
            %
            % Output:
            %           ax - Handle to the created subplot.
        
            % Validate input arguments
            arguments (Input)
                obj        droneGUI
                gridPos    (1,:) double
                titleText  (1,:) char
                yLabelText (1,:) char
                yLimits    (1,2) double
                yTicks     (1,:) double
                xLabelText (1,:) char
                xLimits    (1,2) double
                xTicks     (1,:) double
            end
        
            %% Create the Subplot Axes
            % Create subplot axes based on the specified grid size
            %  and position.
            ax = subplot(obj.gridSize(1), obj.gridSize(2), gridPos);
            
            % Set the subplot title with LaTeX formatting.
            title(ax, titleText, 'Interpreter', 'latex', 'FontSize', 8);
            
            %% Disable Interactive Features for Performance
            % This disables callbacks triggered by interactions,
            % which can be a performance bottleneck in rapidly updating 
            % simulations.
            ax.Interactions = [];
            ax.PickableParts = 'none';
            ax.HitTest = 'off';
            
            %% Set X-Axis Properties
            % Set the X-axis limits, label, and tick marks.
            ax.XLim = xLimits;
            % Assign ticks in X
            ax.XTick = xTicks;
            % Format X-axis tick labels to one decimal place.
            xtickformat(ax, "%.0f"); 
            % Use LaTeX for the X-axis label.
            xlabel(ax, xLabelText, 'Interpreter', 'latex', 'FontSize', 8); 
            % Use a tight limit method to improve visualization 
            % of the data.
            ax.XLimitMethod = 'tight';
            
            %% Set Y-Axis Properties 
            % Set the Y-axis limits, label, and tick marks.
            ax.YLim = yLimits;
            % Assign ticks in Y
            ax.YTick = yTicks;
            % Format Y-axis tick labels to one decimal place.
            ytickformat(ax, "%.0f");  
            % Use LaTeX for the Y-axis label.
            ylabel(ax, yLabelText, 'Interpreter', 'latex', 'FontSize', 8); 
            % Use a tight limit method to improve visualization 
            % of the data.
            ax.YLimitMethod = 'tight';
            
            %% Fix Axes Layout to Prevent Automatic Re-Layout
            % Fix the axes layout by setting the active position 
            % property to 'position' and forcing manual control of 
            % the axis limits.
            set(ax, 'ActivePositionProperty', 'position');
            set(ax, 'XLimMode', 'manual', 'YLimMode', 'manual');
            
            %% Enable Grid Display
            % Turn on the grid to improve readability of the plot.
            grid(ax, 'on');
            % Hold Plot, no more things are going to be added
            hold(ax, "on");
        end
    end
end
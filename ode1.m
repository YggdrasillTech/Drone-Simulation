%% ODE1 Integration
function [t_out, y_out] = ode1(F, t0, h, tfinal, y0)
    % Uses Euler's method with a fixed step size h over the 
    % interval [t0, tfinal] to solve dy/dt = F(t, y) with y(t0) = y0.
    %
    % Inputs:
    %        F - function handle: F(t, y)
    %       t0 - initial time
    %        h - step size
    %   tfinal - final time
    %       y0 - initial condition (row or column vector)
    %
    % Outputs:
    %    t_out - column vector of time points
    %    y_out - solution matrix, each row represents y 
    %            at the corresponding time in t_out
    
    %% Input Argument Validation
    arguments (Input)
        F      function_handle   % F must be a function handle
        t0     (1,1) double      % t0 must be a scalar of type double
        h      (1,1) double      % h must be a scalar of type double
        tfinal (1,1) double      % tfinal must be a scalar of type double
        y0     double            % y0 must be an array of type double
    end

    %% Output Argument Validation
    arguments (Output)
        t_out double             % t_out must be an array of type double
        y_out double             % y_out must be an array of type double
    end

    % Ensure y0 is a column vector
    y_step = y0(:);
    
    %% Calculate the number of steps
    n_steps = ((tfinal - t0) / h) + 1;
    % Rounding may cause issues in some cases, as it might
    % generate an inconsistent number of steps for regular intervals.
    n_steps = round(n_steps);

    %% Preallocate output arrays
    t_out = zeros(n_steps, 1);
    y_out = zeros(length(y0), n_steps);

    %% Initialization
    t_out(1) = t0;
    y_out(:, 1) = y_step;

    %% Time-stepping loop
    for idx = 2:n_steps
        t = t0 + (idx - 2) * h;  % Current time before step
        s = F(t, y_step);        % Compute derivative
        y_step = y_step + h * s; % Euler step
        y_out(:, idx) = y_step;  % Store result
        
        % Update the time vector
        t_out(idx) = t0 + (idx - 1) * h;
    end
end
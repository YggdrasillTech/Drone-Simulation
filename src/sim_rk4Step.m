function y = sim_rk4Step(f, t, h, y)
    % sim_rk4Step: One classical Runge-Kutta step of dy/dt = f(t, y).
    %
    % Local truncation error O(h^5), global error O(h^4). Four derivative
    % evaluations per step buy several orders of magnitude more accuracy
    % than the 50 forward-Euler sub-steps Project_2 took per control
    % period, at roughly a third of the cost. RunTests measures both.
    %
    % Inputs:
    %   f - Function handle f(t, y) returning dy/dt
    %   t - Current time
    %   h - Step size
    %   y - Current state (column vector)
    %
    % Output:
    %   y - State after the step

    h2 = h / 2;

    k1 = f(t,      y);
    k2 = f(t + h2, y + h2 * k1);
    k3 = f(t + h2, y + h2 * k2);
    k4 = f(t + h,  y + h  * k3);

    y = y + (h / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

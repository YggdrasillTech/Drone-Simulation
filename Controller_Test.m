% Create a joystick object using the sim3d.io.Joystick library
joystick = sim3d.io.Joystick();

% Retrieve the capabilities or configuration settings of the joystick
joystickCapabilities = caps(joystick);

% Infinite loop to continuously read and display joystick state.
% To exit the loop, press Ctrl+C in the MATLAB command window.
while true    
    % Read the current state of the joystick:
    %      axes - positions of the joystick axes
    %   buttons - status of the buttons (pressed/released)
    %       pov - position of the point-of-view (hat) control
    [axes, buttons, pov] = read(joystick);

    % Display the joystick capabilities and current state values
    disp(joystickCapabilities);
    disp(axes);
    disp(buttons);
    disp(pov);

    % Pause for 0.5 seconds to avoid excessive CPU usage
    pause(0.5);
end

# Drone Simulation and Control in MATLAB

This repository contains MATLAB code for simulating and controlling a quadrotor (drone). The project features a complete dynamic model—based on Newton–Euler equations—that includes aerodynamic drag, gyroscopic effects due to rotor inertia, and PD-based controllers for attitude (roll, pitch, yaw) and altitude. In addition, a 3D visualization is provided using STL models for the drone body and propellers, and there is support for manual control via a joystick.

## Features

- **Comprehensive Dynamic Model:**
  - Implements quadrotor dynamics using Newton–Euler equations.
  - Includes aerodynamic drag in x, y, and z directions.
  - Incorporates gyroscopic effects from rotor inertia (Ir).
  - Configurable physical parameters (mass, dimensions, inertia, motor constants, etc.).

- **Attitude and Altitude Control:**
  - PD controllers for roll, pitch, and yaw stabilization with gravity compensation.
  - Additional PD control loop for altitude.
  - Motor speed saturation (e.g., maxOmega set to 950 rad/s) to ensure realistic operation.

- **Position Control in the XY Plane:**
  - An outer-loop PD controller that generates desired pitch and roll references from global position errors.
  - Includes a coordinate transformation from the global (inertial) frame to the drone’s body frame using the current yaw, so that the drone can move toward a target point independently of its orientation.
  - Saturation is applied to the generated pitch and roll commands.

- **3D Visualization:**
  - Real-time animation using MATLAB’s `hgtransform` objects.
  - STL models for the drone body and propellers (located in the `media/` folder).
  - Visualization includes coordinate axes and additional graphical adjustments for better display.

- **Manual Joystick Control:**
  - Functions to process joystick input (via `sim3d.io.Joystick`) for manual control in open-loop mode.

## Requirements

- MATLAB (tested on R2018b and later)
- STL files in the `media/` folder:
  - `DroneBody.stl`
  - `DronePropeller.stl`
- (Optional) A joystick connected and recognized by MATLAB using the `sim3d.io.Joystick` class.

## Project Structure

- **`robotDrone.m`**
  Contains the class definition for the drone simulation, including:
  - Properties for physical parameters, motor speeds, and control limits.
  - Methods for initializing and updating 3D animations.
  - The dynamics model (`modelDynamics`) with gyroscopic terms.
  - Control functions:
    - `manualControl`: Open-loop control based on joystick input.
    - `attitudeAltitudeControl`: PD control for attitude and altitude.
    - `positionXYControlSaturated`: Outer-loop XY position controller that transforms errors from the global to the body frame and saturates the desired pitch and roll commands.

- **`runDroneSimulation.m`**
  The main script that initializes the simulation, processes joystick inputs, and performs state integration (using an Euler method via an `ode1` integrator).

- **Additional Utility Functions:**
  Functions for joystick post-processing, simulation stepping, and other support tasks.

## How to Run the Simulation

1. **Clone the Repository:**

    ```bash
    git clone https://github.com/YggdrasillTech/Drone-Simulation.git
    cd drone-simulation
    ```

2. **Setup:**
   - Ensure that the `media/` folder is in the repository root and contains the required STL files.
   - Add the repository folder to your MATLAB path.

3. **Run the Simulation:**
   - Open MATLAB and execute:
     ```matlab
     runDroneSimulation();
     ```
   - The simulation will launch with a 3D visualization. If a joystick is connected, you can use it for manual control; otherwise, the simulation will run in open-loop mode.

## Customization

- **Controller Gains:**
  PD gains for attitude, altitude, and position control (e.g., `Kp_phi`, `Kd_phi`, `Kp_z`, etc.) are defined in `robotDrone.m` and can be tuned as needed.

- **Physical Parameters:**
  Modify the physical constants (mass, inertia, motor constants, aerodynamic coefficients) in the class properties to adapt the model to different drone configurations.

- **Motor Speed Saturation:**
  The `maxOmega` property (set to 950 rad/s) limits the maximum motor speed. Adjust this value according to your motor and propeller specifications.

- **Extensions and Future Enhancements:**
  - Extend the simulation to include multi-agent formation control.
  - Incorporate advanced control techniques such as adaptive or trajectory-tracking controllers.
  - Add obstacle avoidance or environmental disturbances for more complex scenarios.

## Future Enhancements

- **Multi-Agent Formation:**
  Implement formation control and collision avoidance to simulate multiple drones operating as a swarm.

- **Advanced Dynamics:**
  Enhance the model by including more detailed motor dynamics and actuator delays.

- **Experimental Validation:**
  Compare simulation results with real-world measurements to fine-tune the parameters and control algorithms.

## License

This project is licensed under the MIT License.

## Acknowledgments

- The dynamic model and control strategies are inspired from Modelling and control of quadcopter by Teppo Luukkonen.
- STL models used for visualization are provided for educational purposes.
- Thanks to the MATLAB community for the tools and documentation that support projects like this.

# Relative Distance Control Between Two Robots

This repository contains the implementation of various control systems to regulate the distance between two robots, R1 and R2. The project was developed as part of an individual coursework submission during my time at UCL.

## Project Overview

The goal of this project is to control the distance `d(t)` between a robot R1 and a robot R2. The velocity and acceleration of R1 are adjusted to follow a reference distance `d_r(t)` relative to R2, which has its own predefined motion profile. The control strategies implemented include both continuous and discrete PID controllers, as well as a lead compensator.

## Key Observations

- **System Modeling**:
  - The system was modeled in state-space form where the input is the velocity or acceleration of R1, and the output is the distance `d(t)` between R1 and R2.
  - The transfer function derived for R1 when velocity is the control input is `1/s`.

- **Continuous PID Controller**:
  - A continuous PID controller was tuned to ensure the distance `d(t)` follows the reference `d_r(t)` with minimal steady-state error.
  - Stability analysis through pole placement showed that a PI controller was sufficient for maintaining stability, as adding a derivative term caused instability.

- **Discrete PID Controller**:
  - The continuous PID controller was discretized using the Tustin method with a sampling time of 0.2 seconds.
  - The discrete controller performed similarly to the continuous one but exhibited slightly delayed responses due to the sampling rate.

- **Noise Analysis**:
  - Adding Gaussian noise with a standard deviation of 0.01 to the control signal and sensor measurements affected the system performance, introducing fluctuations in the distance `d(t)`.
  - Despite the noise, the overall shape of the control signal was maintained due to the feedback nature of the controller.

- **Lead Compensator for Acceleration Control**:
  - When the control input was the acceleration of R1, a lead compensator was designed to stabilize the system.
  - The lead compensator achieved a stable response with a phase margin of 45 degrees, but with minor oscillations due to the absence of a derivative component in the controller.

## Files in the Repository

- `Q1b.slx`: Simulink model for continuous PID controller.
- `Q1d.slx`: Simulink model for discrete PID controller.
- `Q1e.slx`: Simulink model for discrete PID controller with noise.
- `Q1f.m`: MATLAB script re-implementing the noisy system simulation.
- `Q2c.slx`: Simulink model for lead compensator design and analysis.

## Usage

To use the provided Simulink and MATLAB files:

1. Clone the repository to your local machine.
2. Open the desired `.slx` or `.m` file in MATLAB/Simulink.
3. Run the simulations and observe the results.

## Conclusion

This project demonstrates the application of control theory to robotic systems, focusing on maintaining a precise distance between two moving robots. The simulations explore the effects of controller discretization, noise, and different control strategies (PID and lead compensators) on system performance.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

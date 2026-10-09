# ARL Autonomous Vehicle Control Track

**Student:** Raghad Mohamed
**University:** Ain Shams University
**id:** 2500099
**Project Scope:** Milestones 1–5.2

---

## 1. Project Overview

This project focuses on developing and understanding the main components of an autonomous vehicle control system using ROS 2 and a simulated kinematic bicycle model.

The project starts with exploring the ROS 2 communication system and understanding the vehicle's state and actuator commands. It then progresses to implementing a bicycle model, connecting keyboard teleoperation to the simulated vehicle, developing longitudinal PID control, generating curvature-based target speeds, and implementing lateral PID steering control.

The objective is to understand how vehicle modeling, feedback control, and path-related information work together to control a simulated autonomous vehicle.

**The current project scope ends at Milestone 5.2, Lateral PID Control.** Pure Pursuit, Model Predictive Control (MPC), lap analysis, and later milestones are outside the scope of the current implementation.

## 2. Project Objectives

* Understand ROS 2 nodes, topics, messages, and communication between system components.
* Explore the vehicle simulation and its state and actuator interfaces.
* Understand and implement the kinematic bicycle model.
* Control the simulated vehicle through keyboard teleoperation.
* Implement longitudinal PID control for speed regulation.
* Generate target speeds based on road curvature and lateral acceleration limits.
* Implement lateral PID control to reduce the vehicle's cross-track error.
* Test the implemented components and document their behavior and limitations.

## 3. System Architecture

The project is organized into three ROS 2 packages.

| Package             | Main responsibility                                                  |
| ------------------- | -------------------------------------------------------------------- |
| `bicycle_sim`       | Vehicle simulation and kinematic bicycle model                       |
| `bicycle_control`   | Teleoperation, longitudinal PID, velocity profiling, and lateral PID |
| `track_environment` | Track representation and related environment functionality           |

### Main Components

**1. Vehicle Model**

The vehicle is represented using a kinematic bicycle model. The model updates the vehicle's position and orientation based on its motion and steering inputs.

**2. Teleoperation Bridge**

The teleoperation bridge connects keyboard input to the vehicle's control interface, allowing the vehicle to be driven manually.

**3. Longitudinal PID Controller**

The longitudinal controller adjusts the throttle command based on the difference between target speed and actual speed.

**4. Velocity Profiler**

The velocity profiler limits the target speed according to the road curvature and a configured lateral acceleration limit.

**5. Lateral PID Controller**

The lateral controller uses cross-track error to calculate a steering correction that helps the vehicle follow the reference path.

## 4. Implementation

### Milestone 1 — ROS 2 Exploration

This milestone focuses on understanding the existing ROS 2 system before modifying the control behavior.

The main tasks include inspecting available nodes and topics, identifying the vehicle state and actuator interfaces, and understanding how messages move between components.

Useful ROS 2 commands include:

```bash
ros2 node list
ros2 topic list
ros2 topic info /state
ros2 topic info /throttle
ros2 topic info /steer
ros2 topic echo /state
```

These commands help identify the available interfaces and inspect the information exchanged by the simulation.

### Milestone 2 — Kinematic Bicycle Model

The kinematic bicycle model approximates a vehicle using its wheelbase, longitudinal velocity, steering angle, position, and heading.

For a simplified rear-axle reference model, the continuous-time equations are:

$$
\dot{x}=v\cos(\psi)
$$

$$
\dot{y}=v\sin(\psi)
$$

$$
\dot{\psi}=\frac{v}{L}\tan(\delta)
$$

Where:

* \(x,y\): Vehicle position in the global coordinate frame.
* \(v\): Longitudinal velocity.
* \(\psi\): Vehicle heading angle.
* \(L\): Wheelbase.
* \(\delta\): Front-wheel steering angle.

The equations describe how the vehicle moves forward and changes its heading when steering. The model can be numerically integrated using Euler integration:

$$
x_{k+1}=x_k+\dot{x}_k\Delta t
$$

$$
y_{k+1}=y_k+\dot{y}_k\Delta t
$$

$$
\psi_{k+1}=\psi_k+\dot{\psi}_k\Delta t
$$

Here, \(\Delta t\) is the simulation time step.

### Milestone 3 — Teleoperation

This milestone introduces manual control of the simulated vehicle through keyboard commands.

The teleoperation bridge converts user input into commands for vehicle motion, allowing basic forward, reverse, and steering behavior to be explored through the available control interfaces.

Manual control provides a baseline for understanding how throttle and steering affect the vehicle before introducing autonomous feedback controllers.

### Milestone 4 — Longitudinal PID Control

Longitudinal control regulates the vehicle's speed by comparing the target speed with the measured speed.

The speed error is:

$$
e_v(t)=v_{\text{target}}(t)-v_{\text{actual}}(t)
$$

The PID controller computes its control output using proportional, integral, and derivative terms:

$$
u(t)=K_p e_v(t)+K_i\int_0^t e_v(\tau)\,d\tau+K_d\frac{de_v(t)}{dt}
$$

Where:

* \(K_p\): Proportional gain.
* \(K_i\): Integral gain.
* \(K_d\): Derivative gain.
* \(u(t)\): Controller output used to adjust the longitudinal command.

The proportional term responds to the current speed error, the integral term accounts for accumulated error, and the derivative term responds to how quickly the error changes.

The purpose is to make the vehicle track a desired speed instead of relying entirely on manually chosen throttle commands.

### Milestone 5.1 — Curvature-Based Velocity Profiling

The velocity profiler calculates a target speed based on road curvature and a lateral acceleration limit.

For a vehicle following a curved path, lateral acceleration can be approximated by:

$$
a_y=v^2|\kappa|
$$

Where:

* \(a_y\): Lateral acceleration.
* \(v\): Vehicle speed.
* \(\kappa\): Path curvature.

Applying a maximum allowed lateral acceleration gives the curvature-based speed limit:

$$
v_{\text{curve}}=\sqrt{\frac{a_{y,\max}}{|\kappa|}}
$$

A tighter curve has a larger curvature magnitude and therefore requires a lower speed to respect the same lateral acceleration limit.

For approximately straight sections, curvature approaches zero and this formula does not impose a finite speed limit. The profiler therefore uses the configured maximum speed for sufficiently small curvature.

The target speed can also be limited by a fallback speed, when supplied:

$$
v_{\text{target}}=
\min(v_{\text{curve}},v_{\text{fallback}},v_{\max})
$$

This expression applies when a fallback speed is provided and curvature is sufficiently large. Otherwise, the profiler follows its corresponding straight-section or fallback logic.

### Milestone 5.2 — Lateral PID Control

Lateral control aims to reduce the distance between the vehicle and its reference path.

The lateral PID controller uses the cross-track error (CTE) to determine a steering correction.

A general PID expression is:

$$
u(t)=K_p e(t)+K_i\int_0^t e(\tau)\,d\tau+K_d\frac{de(t)}{dt}
$$

Where \(e(t)\) represents the controller's error signal and \(u(t)\) represents its control output. In lateral control, the error is associated with the vehicle's deviation from the reference path.

The three terms have different roles:

* **Proportional:** Responds to the current deviation.
* **Integral:** Accounts for accumulated error.
* **Derivative:** Responds to the rate of change of the error.

The resulting steering correction is used to guide the vehicle toward the reference path. Controller behavior depends on the gain values, vehicle dynamics, and the way the error signal is defined.

## 5. Software Structure

The main implementation files are organized as follows:

```text
Control_Project/
├── bicycle_control/
│   ├── bicycle_control/
│   │   ├── controller_node.py
│   │   ├── lateral_pid.py
│   │   ├── longitudinal_pid.py
│   │   ├── teleop_bridge.py
│   │   └── velocity_profiler.py
│   └── test/
│       ├── test_lateral_pid.py
│       ├── test_longitudinal_pid.py
│       └── test_velocity_profiler.py
├── bicycle_sim/
│   ├── bicycle_sim/
│   │   └── bicycle_model.py
│   └── test/
│       └── test_bicycle_model.py
├── track_environment/
│   ├── track_environment/
│   └── test/
└── README.md
```

The structure separates vehicle simulation, control algorithms, and track-related functionality. This makes the components easier to understand and maintain independently.

## 6. Building the Workspace

The project uses ROS 2 and `colcon` to build its packages.

From the project root, run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The build command compiles the packages in the workspace and creates the environment needed to run them.

## 7. Running the Simulation

To launch the base vehicle simulation:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py
```

To launch the simulation with the lateral PID controller, use the controller configuration supported by the project's launch file:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid
```

The lateral PID configuration is the final control milestone included in this project report.

## 8. Testing and Validation

Testing is used to check individual components and identify implementation issues.

The project includes test files for the bicycle model, longitudinal PID, lateral PID, and velocity profiler, as well as tests for additional controllers.

The recorded test results include:

* The targeted lateral PID test passed.
* A broader test run of `bicycle_control` reported three failures and one skipped test. That run included tests for controllers outside the current project scope, so the complete test suite cannot be described as passing.

These results are reported separately to avoid confusing a successful targeted test with complete validation of the whole package.

A successful build confirms that the workspace can be built; it does not by itself prove that the vehicle follows the reference path correctly under every condition.

## 9. Current Scope and Limitations

The current implementation and report cover the project through Milestone 5.2.

| Milestone | Topic                                      | Scope status          |
| --------- | ------------------------------------------ | --------------------- |
| 1         | ROS 2 exploration                          | Included              |
| 2         | Kinematic bicycle model                    | Included              |
| 3         | Teleoperation bridge                       | Included              |
| 4         | Longitudinal PID                           | Included              |
| 5.1       | Curvature-based velocity profiling         | Included              |
| 5.2       | Lateral PID                                | Included              |
| 5.3       | Pure Pursuit                               | Outside current scope |
| 5.4       | Model Predictive Control (MPC)             | Outside current scope |
| 5.5       | Lap analysis and benchmarking              | Outside current scope |
| 6 onward  | Further exploration and later deliverables | Outside current scope |

The current report does not claim measured lap times, completed autonomous laps, benchmark comparisons, or performance improvements without corresponding results.

## 10. Conclusion

This project develops an understanding of autonomous vehicle control by progressing from ROS 2 system exploration to vehicle modeling, manual control, and feedback-based control.

The kinematic bicycle model provides a simplified description of vehicle motion. The longitudinal PID controller regulates speed, the velocity profiler relates road curvature to a safe target speed, and the lateral PID controller calculates steering corrections based on path deviation.

Together, these components establish the foundation for an autonomous vehicle control pipeline. Further controller implementations and performance evaluation remain outside the current project scope.

---

## Author

**Raghad Mohamed**
Ain Shams University
Autonomous Racing Lab (ARL) — Individual Control Project

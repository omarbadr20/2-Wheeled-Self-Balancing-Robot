# 2-Wheeled-Self-Balancing-Robot
# System Overview
## Purpose and Goal
The primary objective of this project is to design and implement a stabilizing control system for a two-wheeled self-balancing robot. Theoretically modelled as an inverted pendulum on a cart, this system is inherently unstable; its centre of mass is located above its pivot point, meaning it will naturally fall without active control.

The goal of this project is to maintain a vertical pitch angle ($\theta \approx 0^\circ$) by applying corrective torque to the wheels in the direction of the fall. This project implements and compares two distinct control strategies—PID Control and State-Space Control—using National Instruments LabVIEW as the controller and an Arduino Nano as the data acquisition interface.

## Electrical Connections
The system utilizes an Arduino Nano to interface with the sensors and actuators. Power is managed through separate paths for the high-current motor load and low-power logic electronics to prevent interference. Key components include an MPU-6500 IMU for sensing and an L298N Dual H-Bridge driver to control the two JGA25-370 DC geared motors.
![alt text](https://github.com/omarbadr20/2-Wheeled-Self-Balancing-Robot/blob/main/Related%20Media/self_balancing_connections.jpeg)

## Mechanical Structure
The robot chassis is constructed from laser-cut wood in a dual-plate configuration. To increase the system's rotational inertia and slow down its falling dynamics, the heavy Li-Ion battery pack is mounted on the top plate, raising the Center of Mass (CoM). The motors are directly coupled to the wheels to minimize backlash.
![alt text](https://github.com/omarbadr20/2-Wheeled-Self-Balancing-Robot/blob/main/Related%20Media/Robot_Structure.jpeg)

# Controllers' Design
## State-Space Controller Design
The robot is modelled as a linear system around the upright equilibrium point. Since the robot lacks wheel encoders, the design uses a 2-state model focusing exclusively on Attitude Control.
- State Vector: $x = [\theta, \dot{\theta}]^T$ (Tilt Angle, Angular Velocity).
- Control Law: $u = -Kx$.
- Gain Calculation: The feedback gain matrix $K = [k_{\theta}, k_{\dot{\theta}}]$ is computed using the Pole Placement method in LabVIEW to place the system poles in the stable Left Half Plane.

## PID Controller Design
A classical PID controller operates on the error signal defined as the difference between the desired setpoint ($\theta_{set}$) and the measured angle ($\theta_{measured}$).

- Error: $e(t) = \theta_{set} - \theta(t)$.
- Control Law: $u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$.
- Implementation: Parallel branches for Proportional, Integral, and Derivative terms are summed to generate the final PWM signal.

# Sensing, Calibration, and Feedback
## Sensor Integration
The system uses an MPU-6500 IMU to determine orientation:
- Accelerometer: Calculates tilt ($\theta_{acc}$) using gravity projection but suffers from vibration.
- Gyroscope: Measures angular velocity ($\omega$) but suffers from integration drift over time.

## Complementary Filter
To fuse the sensor data, a digital Complementary Filter is implemented in LabVIEW. It combines the stable (but noisy) accelerometer data with the responsive (but drifting) gyroscope integration:

$$\theta(t) = 0.98 \cdot (\theta(t-1) + \omega_{gyro} \cdot dt) + 0.02 \cdot \theta_{acc}$$

This filter assigns 98% of its weight to the gyroscope for short-term responsiveness and 2% to the accelerometer to correct drift.

## Sensor Calibration
A separate calibration VI is used to calculate the gyroscope offset. The robot is held vertically, and 500 samples are averaged to find the bias, which is then subtracted from the raw readings during operation

![alt text](https://github.com/omarbadr20/2-Wheeled-Self-Balancing-Robot/blob/main/Related%20Media/Calibration_Snippet.png)

# LabVIEW Implementation and Integration
## State-Feedback Implementation
The State-Feedback Control Loop runs on the host PC.
1. Angle Estimation: Raw IMU data is read via serial and processed through the complementary filter.
2. State Construction: The state vector $x$ is formed using the filtered angle $\theta$ and the derived rate $\dot{\theta}$.
3. Feedback Computation: The vector is multiplied by the gain matrix $K$ and summed to produce the control effort $u$.

![alt text](https://github.com/omarbadr20/2-Wheeled-Self-Balancing-Robot/blob/main/Related%20Media/2_State_Modelling_Snippet.png)


## PID Implementation
The PID logic is built using standard LabVIEW numeric blocks.
1. Error Calculation: The measured angle is subtracted from the user-defined setpoint.
2. PID Processing: The error is distributed to parallel P, I, and D branches.
3. Output: The summed output is mapped to a PWM duty cycle for the motors.
![alt text](https://github.com/omarbadr20/2-Wheeled-Self-Balancing-Robot/blob/main/Related%20Media/PID_Modelling_Snippet.png)

# Results And Analysis
## Steps to Run
1. Calibrate: Run the calibration VI to determine the Gyro offset (essential for each power cycle).
2. Set Parameters: Input offset and desired setpoint.
3. Adjust Gains according to your system ($K_p, K_i, K_d$ or $K_\theta, K_{\dot{\theta}}$).

## Summary of Results
State-Space vs. PID:

 - State-Space: Achieved a faster settling time (1.0s) with aggressive corrections. It effectively drove the steady-state error to zero. Preferred for dynamic balancing.

 - PID: Slower settling time (1.5s) with a smooth but sluggish response. It struggled to correct rapid disturbances compared to the State-Space controller.

# Conclusion 

This project successfully stabilized a two-wheeled robot using a remote Hardware-in-the-Loop architecture. By optimizing communication latency and implementing a robust state-space controller, the system overcame the inherent instability of the inverted pendulum model. The comparison highlighted the superior dynamic performance of modern control methods (State-Space) over classical ones (PID) for this specific application. For State Feedback, the project performed well on different surfaces (Porcelain Floor, Carpets, and Desktops) with the same system gains or with minimal tuning, while for the PID controller, the system required different tuned parameters for each surface, and even then, the performance varied.

# Demo
Check out the robot implementing the 2 controllers: [Video Link](https://youtu.be/8AiDUAj7NyE)


# Future Work
 - Wheel Encoders: Add encoders to enable position control and full state feedback ($x, \dot{x}$).
 - Kalman Filter: Replace the Complementary Filter with a Kalman Filter for optimal noise rejection.
 - LQR: Implement a Linear Quadratic Regulator to optimize the trade-off between error and energy consumption.
 - Embedded Control: Switch to an ESP32 to run algorithms directly on-chip, eliminating USB latency and enabling loop rates >100 Hz.

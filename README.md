# 6-DOF UR5 Industrial Manipulator: Spatial Multi-Body Dynamics & Control in 20-sim

A first-principles, multi-body physical model of the **Universal Robots UR5** robotic arm developed in **20-sim** using spatial **Bond Graph** theory.

Unlike simplified kinematic models, this simulation captures full 3D spatial kinetics, nonlinear actuator dynamics (harmonic drive compliance, inertia, and friction), and dynamic load propagation down the kinematic chain.

---

## Project Overview

This repository contains the complete physical model, kinematic parameterization, and control architecture for the 6-DOF UR5 manipulator:
- **Spatial Multi-Body Kinetics**: 6-DOF Newton-Euler momentum formulation in rotating local frames.
- **Natural Coriolis & Centrifugal Coupling**: Evaluated locally via cross-product momentum terms ($\boldsymbol{\omega} \times \mathbf{p}$ and $\boldsymbol{\omega} \times \mathbf{h}$), eliminating complex symbolic Christoffel derivations.
- **Harmonic Drive Actuators**: Piecewise nonlinear torsional stiffness (3 deflection regimes), rotor inertia, and Coulomb/viscous friction.
- **Decentralized PID & Gravity Compensation**: Trajectory-following control augmented with analytical feedforward gravity compensation on vertical load joints.

---

## Kinematics & Denavit-Hartenberg (DH) Parameters

The robot kinematic chain is parameterized using standard Denavit-Hartenberg conventions:

| Link ($i$) | Joint Angle ($\theta_i$) | Twist Angle ($\alpha_i$) [rad] | Link Length ($a_i$) [m] | Link Offset ($d_i$) [m] | Mass ($m_i$) [kg] |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **1** | $\theta_1$ | $\pi / 2$ | $0.0000$ | $0.089159$ | $3.700$ |
| **2** | $\theta_2$ | $0$ | $-0.4250$ | $0.000000$ | $8.393$ |
| **3** | $\theta_3$ | $0$ | $-0.39225$ | $0.000000$ | $2.275$ |
| **4** | $\theta_4$ | $\pi / 2$ | $0.0000$ | $0.109150$ | $1.219$ |
| **5** | $\theta_5$ | $-\pi / 2$ | $0.0000$ | $0.094650$ | $1.219$ |
| **6** | $\theta_6$ | $0$ | $0.0000$ | $0.082300$ | $0.1879$ |

### Initial Transformations

Initial positions and orientations in base coordinates are initialized using homogeneous transformation matrices $\mathbf{T}_{0i,\text{init}}$:

$$\mathbf{p}_{OGi,\text{init}} = \mathbf{T}_{0i,\text{init}}[1:3, 1:3] \cdot \mathbf{r}_{Gi} + \mathbf{T}_{0i,\text{init}}[1:3, 4]$$

$$\mathbf{R}_{\text{init}, i} = \mathbf{T}_{0i,\text{init}}[1:3, 1:3]$$

---

## Mathematical & Bond Graph Formulation

### 1. Link Dynamics (Euler Submodel `EE`)

Each link is modeled as a 3D rigid body with two vector momentum state variables defined at its Center of Mass (CoM, Point $G$):

- **Linear Momentum**:
  
$$\dot{\mathbf{p}}_i = \mathbf{F}_{\text{net}, i} - \boldsymbol{\omega}_i \times \mathbf{p}_i \quad \Longrightarrow \quad \mathbf{v}_{\text{CoM}} = \frac{1}{m_i} \mathbf{p}_i$$

- **Angular Momentum**:
  
$$\dot{\mathbf{h}}_i = \boldsymbol{\tau}_{\text{net}, i} - \boldsymbol{\omega}_i \times \mathbf{h}_i \quad \Longrightarrow \quad \boldsymbol{\omega}_i = \mathbf{J}_i^{-1} \mathbf{h}_i$$

Dynamic orientation is continuously integrated:

$$\dot{\mathbf{R}}_i = \mathbf{R}_i \cdot \text{skew}(\boldsymbol{\omega}_i)$$

Global gravity is transformed into each link's local moving body frame via the transpose of its rotation matrix:

$$\mathbf{F}_{\text{gravity, body}} = \mathbf{R}_i^T \cdot \begin{bmatrix} 0 \\ 0 \\ -9.81 \cdot m_i \end{bmatrix}$$

### 2. Joint Modeling & Causality (`MTF`)

Each joint submodel performs power-conserving spatial transformations:

- **Forward Flow (Velocities)**:
  
$$\boldsymbol{\omega}_{i} = \mathbf{R}_{\text{joint}}(\theta_i) \cdot \boldsymbol{\omega}_{i-1}$$

- **Backward Effort (Forces & Torques)**:
  
$$\boldsymbol{\tau}_{i-1} = \mathbf{R}_{\text{joint}}^T(\theta_i) \cdot \boldsymbol{\tau}_{i}$$

- **Constraint & Drive Multiplexing**: Blends 2D transverse constraint torques with 1D drive torque from the harmonic drive along the active $Z$-axis.

### 3. Harmonic Drive Non-Linearities (`H_Drive`)

- **Torsional Compliance ($C$)**: Piecewise nonlinear elasticity across three deflection regimes ($\phi_1$, $\phi_2$) with stiffness parameters $k_1$, $k_2$, and $k_3$.
- **Rotor & Gearbox Inertia ($I$)**: High-speed motor inertia reflected through reduction ratios.
- **Friction Dissipation ($R$)**: Mixed Coulomb breakaway torque and viscous damping.

### 4. Control & Gravity Compensation

- **Decentralized PID**: Joint-level error feedback driven by dynamic reference trajectories streamed via an external library (`DataFromFile.dll`).
- **Feedforward Gravity Compensation**: Analytical gravity torques calculated for vertical joints (Joints 2 and 3) to cancel static gravitational droop and eliminate steady-state position error:

$$\tau_{\text{command}} = \tau_{\text{PID}} + G_i(q)$$

---

## Repository Structure

```text
├── Documents.zip             # Compressed archive of data sheets & design specifications
├── UR5 inverse kinematics.pdf # Inverse kinematics derivations and notes
├── UR5 Dynamic.emx           # Complete 20-sim multi-body dynamic model
├── joint_1_with_tau.csv      # Joint 1 simulation and torque data logs
├── joint_2_with_tau.csv      # Joint 2 simulation and torque data logs
├── joint_3_with_tau.csv      # Joint 3 simulation and torque data logs
├── joint_4_with_tau.csv      # Joint 4 simulation and torque data logs
├── joint_5_with_tau.csv      # Joint 5 simulation and torque data logs
├── joint_6_with_tau.csv      # Joint 6 simulation and torque data logs
└── README.md                 # Project documentation

```

```

```

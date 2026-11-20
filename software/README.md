# Part II: Symbolic Kinematics, Workspace Synthesis & Dynamics

This directory contains the MATLAB implementation for the analytical modeling, workspace synthesis, and inverse dynamics of the **3-DOF Delta Parallel Robot**.

## 📁 Files in This Folder
* `Mechanism.mlx` — Main script for symbolic derivations, numeric validation, workspace scanning, trajectory generation, and 3D animation.
* `delta_trajectory.mat` — Exported time-series dataset (`t`, `pth`, `ink`, `V`, `Ava`, `TAU`) used to drive and cross-validate **Part III (Simscape Multibody)**.
* `Mechanism.avi` — Recorded 3D kinematic and torque animation of the mechanism.
* `Symbolic Math` - Display the code used for writing.

---

## 1. Geometric Parameters (Matched with CAD & Part III)
| Parameter | Symbol | Value | Description |
| :--- | :---: | :---: | :--- |
| Base Radius | $R$ | $40.72\text{ mm}$ | Center of base to servo rotation axis |
| Platform Radius | $r$ | $29.64\text{ mm}$ | Center of moving platform to ball-joint midpoint |
| Upper Arm Length | $L$ | $40.0\text{ mm}$ | Servo horn length |
| Forearm Link Length | $l$ | $82.0\text{ mm}$ | Parallelogram link length (ball-to-ball) |
| Servo Orientation | $\phi_i$ | $[90^\circ, -30^\circ, -150^\circ]$ | Azimuthal angles around Z-axis |
| Moving Mass | $m$ | $0.10\text{ kg}$ | Combined mass of platform and payload |

---

## 2. Analytical Formulations (Position, Velocity, Acceleration & Force)

### A. Position Analysis (Loop-Closure & Closed-Form IK)
For each kinematic chain $i \in \{1, 2, 3\}$, the elbow position $\mathbf{c}_i(\theta_i)$ and end-effector position $\mathbf{p} = [x, y, z]^T$ satisfy the vector loop-closure constraint:

$$g_i(\mathbf{p}, \theta_i) = \|\mathbf{p} - \mathbf{c}_i(\theta_i)\|^2 - l^2 = 0$$

Expanding this constraint yields the tangent half-angle equation $E_i\cos\theta_i + F_i\sin\theta_i + G_i = 0$, giving the closed-form Inverse Kinematics (elbow-out branch):

$$
\theta_i = 2\mathrm{atan2}\left(-F_i - \sqrt{E_i^2 + F_i^2 - G_i^2},\; G_i - E_i\right)
$$

### B. Velocity Analysis & Singularity Conditions
Differentiating the loop constraints with respect to time yields the analytical velocity mapping $\dot{\mathbf{p}} = \mathbf{J}\dot{\theta}$, where the $3 \times 3$ Jacobian matrix is:

$$\mathbf{J} = \mathbf{J}_x^{-1}\mathbf{J}_\theta$$

* **Type 1 Singularity ($\det(\mathbf{J}_\theta) = 0$):** Occurs when an upper arm and forearm become collinear (workspace boundary).
* **Type 2 Singularity ($\det(\mathbf{J}_x) = 0$):** Occurs when all three forearms become coplanar (loss of stiffness).

### C. Acceleration & Force Analysis (Principle of Virtual Work)
End-effector acceleration $\ddot{\mathbf{p}}$ is evaluated along the trajectory to determine the required Cartesian force vector $\mathbf{F} = m(\ddot{\mathbf{p}} - \mathbf{g})$. Applying the Principle of Virtual Work maps this force to the required servo torques:

$$\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F} = \mathbf{J}^T m(\ddot{\mathbf{p}} - \mathbf{g})$$

---

## 3. Minimum-Jerk Trajectory & Cross-Validation with Part III
To cross-validate against **Part III (Simscape Multibody)**, the script executes the identical **7.3 s minimum-jerk trajectory** ($s(x) = 10x^3 - 15x^4 + 6x^5$): moving from `Home [0, 0, -119.55] mm` to a $R = 30\text{ mm}$ circle at $z = -81\text{ mm}$ (2 laps in 4 s) and returning home.

| Metric | Part II (Analytical / MATLAB) | Part III (Simscape Multibody) | Notes |
| :--- | :---: | :---: | :--- |
| **FK / IK Round-Trip Error** | $3.39 \times 10^{-14}\text{ mm}$ | $\le 0.71\text{ mm}$ (path dev.) | Exact closed-form consistency |
| **Usable Workspace Volume** | $429.9\text{ cm}^3$ | Full circle $r = 43\text{ mm}$ at $z = -81\text{ mm}$ | Encloses the $R = 30\text{ mm}$ test circle within $\theta \in [0^\circ, 110^\circ]$ |
| **Servo Angle Range ($\theta_1, \theta_2, \theta_3$)** | $2.5^\circ \dots 80.6^\circ$ | $2.5^\circ \dots 80.6^\circ$ | Exact match across all three joints |
| **Max Joint Velocity ($\dot{\theta}_{\max}$)** | $153.8 / 152.4 / 150.2\text{ }^\circ/\text{s}$ | $157.4 / 151.8 / 150.0\text{ }^\circ/\text{s}$ | Agrees within $3.6\text{ }^\circ/\text{s}$ |
| **Max Joint Acceleration ($\ddot{\theta}_{\max}$)** | $883 / 934 / 786\text{ }^\circ/\text{s}^2$ | $892 / 937 / 787\text{ }^\circ/\text{s}^2$ | Agrees within $9\text{ }^\circ/\text{s}^2$ |
| **Max End-Effector Speed / Accel** | $175.7\text{ mm/s}$ / $1027.4\text{ mm/s}^2$ | $176.7\text{ mm/s}$ / $1041\text{ mm/s}^2$ | Discrete $30\text{ fps}$ vs continuous solver ($< 1.3\%$ diff) |
| **Max Condition Number** | $6.43$ | Singularity-free | Low $\max \mathrm{cond}(\mathbf{J})$ confirms well-conditioned motion |
| **Max Servo Torque ($\tau_{\max}$)** | $29.6 / 30.5 / 28.7\text{ N}\cdot\text{mm}$ | $22.5 / 23.0 / 22.0\text{ mN}\cdot\text{m}$ | $< 3\%$ of MG996R stall torque ($1079\text{ N}\cdot\text{mm}$); Part II assumes $0.10\text{ kg}$ lumped mass vs CAD part inertia in Part III |

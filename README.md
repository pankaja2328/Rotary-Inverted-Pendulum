# Rotary Inverted Pendulum (Furuta Pendulum)

A robust control engineering project to simulate, design, and implement an embedded controller to stabilize and balance a Rotary Inverted Pendulum (Furuta Pendulum) in both its upright (unstable equilibrium) and downright (stable equilibrium) positions.

---

## 📂 Project Structure

This repository is organized into three primary directories:

*   **`docs/`**: Design specifications, reference manuals, datasheets (e.g., AS5600 Magnetic Rotary Position Sensor), and task guidelines.
*   **`simulation/`**: MATLAB scripts and Simulink models (`.slx`) for system identification, linearization, PID controller tuning, and nonlinear dynamic simulation.
*   **`src/`**: Embedded source code, drivers, and algorithms for microcontroller programming (firmware deployment).

---

## ⚙️ 1. Project Overview & Mechanical Configuration

The objective is to design a controller that applies torque to a horizontal rotary arm to balance a free-swinging pendulum rod attached at its tip.

```
       Upright Position (Unstable Equilibrium)
                 |
                 |  \ theta (Pendulum Angle)
                 |                    |    \  Pendulum Rod
                 |                      +------o (Free Joint)
                /
               / alpha (Arm Angle)
              /
             /
         (Motor)
```

---

## 📐 2. System Dynamics (Equations of Motion)

Using the Euler-Lagrange formulation, the non-linear equations of motion for the system are defined as:

$$(J_1 + m_2 r^2 + m_2 L^2 \sin^2\theta)\ddot{\alpha} + (m_2 r L \cos\theta)\ddot{\theta} - 2 m_2 L^2 \sin\theta \cos\theta \dot{\alpha}\dot{\theta} - m_2 r L \sin\theta \dot{\theta}^2 = \tau - b_1\dot{\alpha}$$

$$(m_2 r L \cos\theta)\ddot{\alpha} + (J_2 + m_2 L^2)\ddot{\theta} + m_2 L^2 \sin\theta \cos\theta \dot{\alpha}^2 - m_2 g L \sin\theta = -b_2\dot{\theta}$$

### Parameter Definitions
*   **$\alpha$**: Rotary arm angle
*   **$\theta$**: Pendulum rod angle (measured from vertical upright)
*   **$\tau$**: Motor torque input
*   **$J_1, J_2$**: Moments of inertia (arm and pendulum respectively)
*   **$m_2$**: Mass of the pendulum rod
*   **$r$**: Length of the rotary arm
*   **$L$**: Half-length of the pendulum rod (distance to Center of Mass)
*   **$b_1, b_2$**: Viscous friction coefficients

---

## 🔒 3. Design Constraints & Rules

The implementation strictly adheres to the following constraints:
1.  **Single Actuator:** Only one motor drives the horizontal rotary arm.
2.  **Single Feedback Sensor:** A single sensor (e.g., AS5600 rotary encoder) is utilized to measure the dynamic state.
3.  **Control Techniques:** Applies established control theory and stability criteria (e.g., Classical Control Systems, Root Locus, Bode plots, State-Space LQR/Pole Placement, or PID tuning methods).

---

## 📈 4. Simulation & Control Design (`/simulation`)

The simulation folder contains MATLAB and Simulink utilities to verify control stability prior to hardware implementation:

*   **Linearization**: Near upright unstable equilibrium ($\theta \approx 0$).
*   **PID tuning**: Auto-tunes filtered PID gains using negative feedback ($u = -(K_p \alpha + K_i \int \alpha + K_d \dot{\alpha})$).
*   **Simulink Model**: Dynamic validation block diagrams (`furuta_pid_option_b_model.slx`).

For details on running the simulations, see the [Simulation README](file:///c:/Users/pankaja/Desktop/Design%20Task%2003/Rotary%20Inverted%20Pendulum/simulation/README.md).

---

## 💻 5. Embedded Implementation (`/src`)

The source folder hosts the microcontroller firmware. The controller uses the tuned PID gains to compute real-time motor control efforts using feedback from the AS5600 encoder.

# Physics-Aware Multi-Agent Reinforcement Learning for Distributed Control of Modular Floating Structures

A research-oriented simulation framework for the **distributed control of interconnected floating structures under stochastic marine disturbances**, combining dynamic system modelling, classical optimal control, constrained predictive control, and physics-aware multi-agent reinforcement learning.

The project investigates how multiple interconnected floating modules can cooperatively reduce structural motion and connector loads while limiting control effort under changing environmental conditions.

The framework follows the research pipeline:

```text
Dynamic Modelling
        ↓
State-Space Representation
        ↓
Modal Analysis
        ↓
Environmental Disturbance Simulation
        ↓
Classical / Optimal Control
     ┌──────┴──────┐
     ↓             ↓
    LQR           MPC
     \             /
      \           /
       ↓         ↓
 Physics-Aware MAPPO
          ↓
 Robustness Evaluation
          ↓
 Monte Carlo Analysis
```

> **Scope:** The public implementation uses a generic low-order floating-structure model. It is intended for control, reinforcement-learning, and simulation research and does not represent a specific vessel, offshore platform, or proprietary hydrodynamic model.

---

# 1. Motivation

Large floating structures can be constructed from multiple interconnected modules rather than a single rigid body.

Such systems introduce coupled dynamic effects:

```text
Environmental Disturbance
          ↓
     ┌─────────┐
     │Module 1 │
     └────┬────┘
          │ Connector
          ↓
     ┌─────────┐
     │Module 2 │
     └────┬────┘
          │ Connector
          ↓
     ┌─────────┐
     │Module 3 │
     └─────────┘
```

Motion of one module affects neighboring modules through the connectors.

The resulting control problem therefore involves several competing objectives:

- Reducing module displacement
- Reducing module velocity
- Limiting connector forces
- Avoiding excessive control effort
- Maintaining performance under changing disturbances
- Maintaining robustness to model uncertainty

This makes the system a suitable benchmark for **physics-aware cooperative and multi-agent control**.

---

# 2. System Architecture

The implemented architecture is:

```text
           Stochastic Marine Disturbances
                       |
                       v
          Multi-Module Dynamic Model
                       |
                       v
               System States
                       |
        +--------------+--------------+
        |              |              |
        v              v              v
       LQR            MPC        Physics-Aware
                                    MAPPO
        |              |              |
        +--------------+--------------+
                       |
                       v
                Control Forces
                       |
                       v
              Floating Modules
                       |
                       v
               Connector Loads
                       |
                       v
              Performance Metrics
```

The same dynamic environment can therefore be used to evaluate different control approaches.

---

# 3. Floating Modular Structure

The current implementation considers a chain of interconnected floating modules.

A typical configuration is:

```text
            Environmental Excitation

        ↓              ↓              ↓

   ┌─────────┐     ┌─────────┐     ┌─────────┐
   │Module 1 │=====│Module 2 │=====│Module 3 │
   └─────────┘     └─────────┘     └─────────┘
        ↑               ↑               ↑
       u1              u2              u3

          ===== Spring-Damper Connector
```

Each module has an independent control input while remaining dynamically coupled to its neighboring modules.

The default implementation uses three modules, although the model structure supports a configurable number of modules.

---

# 4. Dynamic Model

The current public model focuses on the **heave dynamics** of each module.

For module `i`:

```text
m_i z_ddot_i
+
c_i z_dot_i
+
k_i z_i
+
F_connector,i
=
F_environment,i
+
u_i
```

where:

```text
m_i               = module mass
c_i               = hydrodynamic-equivalent damping
k_i               = restoring stiffness
z_i               = vertical displacement
z_dot_i           = vertical velocity
F_connector,i     = connector interaction force
F_environment,i   = environmental disturbance
u_i               = control force
```

The model is intentionally low-order so that control and learning architectures can be studied independently from high-fidelity hydrodynamic modelling.

---

# 5. Connector Dynamics

Adjacent modules are connected using a linear spring-damper model.

For modules `i` and `j`:

```text
F_c,ij =
k_c (z_i - z_j)
+
c_c (z_dot_i - z_dot_j)
```

where:

```text
k_c = connector stiffness
c_c = connector damping
```

The connector therefore transfers forces between neighboring modules according to both relative displacement and relative velocity.

Conceptually:

```text
Module i
   |
   | z_i - z_j
   |
Spring + Damper
   |
   | F_c,ij
   |
Module j
```

This coupling is a central part of the control problem because minimizing the motion of individual modules does not necessarily minimize connector loads.

---

# 6. Matrix Dynamic Model

The coupled system can be represented as:

```text
M q_ddot + C q_dot + K q = B u + F_env
```

where:

```text
q     = module displacement vector
M     = mass matrix
C     = damping/coupling matrix
K     = restoring/coupling stiffness matrix
B     = control-input matrix
u     = control-force vector
F_env = environmental disturbance vector
```

The connector terms are incorporated directly into the global damping and stiffness matrices.

---

# 7. State-Space Representation

The state vector is defined as:

```text
x =
[ q
  q_dot ]
```

The system is converted into:

```text
x_dot = A x + B u + E d
```

where:

```text
A = system dynamics matrix
B = control matrix
E = disturbance matrix
d = environmental disturbance
```

This representation provides a common foundation for:

- Dynamic analysis
- LQR design
- MPC
- Reinforcement learning
- Robustness studies

---

# 8. Modal Analysis

The project includes modal-analysis functionality based on the generalized eigenvalue problem:

```text
K phi = lambda M phi
```

The natural frequencies are obtained from:

```text
omega_n = sqrt(lambda_n)
```

The corresponding eigenvectors provide the mode shapes.

The analysis therefore allows examination of:

```text
Mass / Stiffness
       ↓
Eigenvalue Problem
       ↓
Natural Frequencies
       +
Mode Shapes
```

This provides physical insight into the coupled dynamics before controller design.

---

# 9. Environmental Disturbance Model

The environment generates stochastic wave-like forces acting independently on each module.

The generic disturbance consists of:

```text
F_env,i(t) =
A_i sin(omega_i t + phi_i)
+
n_i(t)
```

where:

```text
A_i      = disturbance amplitude
omega_i  = excitation frequency
phi_i    = random phase
n_i(t)   = stochastic disturbance component
```

Different modules receive different excitation frequencies and phases.

This produces non-identical disturbances across the floating structure.

---

# 10. Sea-State Severity

The disturbance magnitude can be scaled using a configurable severity parameter.

Conceptually:

```text
Low Severity
     ↓
Smaller Environmental Forces

Moderate Severity
     ↓
Nominal Environmental Forces

High Severity
     ↓
Larger Environmental Forces
```

This allows the same controllers to be evaluated under different environmental conditions.

---

# 11. Uncontrolled Baseline

Before applying active control, the system can be simulated with:

```text
u = 0
```

This provides the uncontrolled reference case.

The controlled methods can then be evaluated relative to this baseline.

The comparison structure is:

```text
Uncontrolled
     ↓
    LQR
     ↓
    MPC
     ↓
Physics-Aware MAPPO
```

---

# 12. LQR Baseline

The first classical controller is a continuous-time **Linear Quadratic Regulator**.

The control law is:

```text
u = -Kx
```

The gain matrix is obtained by minimizing:

```text
J =
∫ (x^T Q x + u^T R u) dt
```

where:

```text
Q = state penalty matrix
R = control penalty matrix
```

The corresponding algebraic Riccati equation is solved to obtain the feedback gain.

LQR provides a strong model-based baseline for evaluating the learning-based controller.

---

# 13. Control Saturation

The control forces are bounded according to:

```text
-u_max <= u_i <= u_max
```

This prevents the controllers from applying arbitrarily large forces.

Control saturation is considered during simulation for both classical and learning-based methods.

---

# 14. Model Predictive Control

The second model-based baseline is a finite-horizon **Model Predictive Controller**.

At each control step, a sequence of future control actions is optimized.

The objective includes:

```text
J =
Σ [
    w_z ||z||²
    +
    w_v ||z_dot||²
    +
    w_f ||F_connector||²
    +
    w_u ||u||²
]
```

subject to:

```text
-u_max <= u_i <= u_max
```

Only the first optimized control input is applied.

The optimization is then repeated at the next time step:

```text
Current State
     ↓
Predict Future Dynamics
     ↓
Optimize Control Sequence
     ↓
Apply First Input
     ↓
New State
     ↓
Repeat
```

This produces a receding-horizon controller capable of explicitly considering actuator limits.

---

# 15. Why Compare LQR and MPC?

The two model-based controllers provide different reference points.

```text
LQR
 |
 +-- Infinite-horizon state feedback
 +-- Computationally lightweight
 +-- No explicit constraint optimization

MPC
 |
 +-- Finite prediction horizon
 +-- Explicit actuator bounds
 +-- Receding-horizon optimization
```

These baselines provide meaningful comparisons for the reinforcement-learning architecture.

---

# 16. Multi-Agent Reinforcement Learning

The main learning-based component uses a compact **Multi-Agent Proximal Policy Optimization (MAPPO)** architecture.

Each floating module is treated as an agent.

For a three-module system:

```text
             Global Floating Structure
                      |
          +-----------+-----------+
          |           |           |
          v           v           v
       Agent 1     Agent 2     Agent 3
          |           |           |
          v           v           v
         u1          u2          u3
```

Each agent generates a normalized control action that is converted into an actuator force.

---

# 17. Centralized Training, Decentralized Execution

The MAPPO implementation follows the **Centralized Training, Decentralized Execution (CTDE)** concept.

During training:

```text
              Global State
                   |
                   v
          Centralized Critic
                   |
            Value Estimate
```

while the control policies operate using local observations:

```text
Local Observation 1 -> Shared Actor -> Action 1
Local Observation 2 -> Shared Actor -> Action 2
Local Observation 3 -> Shared Actor -> Action 3
```

The actor parameters are shared among homogeneous modules.

This provides a compact multi-agent learning architecture.

---

# 18. Agent Observation

For agent `i`, the local observation contains:

```text
o_i =
[
 z_i,
 z_dot_i,
 z_(i-1) - z_i,
 z_(i+1) - z_i
]
```

where available.

Thus each agent receives:

- Its own displacement
- Its own vertical velocity
- Relative displacement to the left neighbor
- Relative displacement to the right neighbor

Boundary modules use zero for unavailable neighboring states.

---

# 19. Agent Action

Each agent generates one normalized action:

```text
a_i ∈ [-1, 1]
```

The action is converted to physical control force through:

```text
u_i = u_max a_i
```

Therefore:

```text
Observation
     ↓
Actor Network
     ↓
Normalized Action
     ↓
Actuator Scaling
     ↓
Control Force
```

---

# 20. Physics-Aware Reward Function

A central feature of the project is the reward formulation.

The global reward penalizes physically meaningful quantities:

```text
r =
-
[
w_1 mean(z²)
+
w_2 mean(z_dot²)
+
w_3 mean(F_connector²)
+
w_4 mean(u²)
]
```

The individual terms correspond to:

```text
Module Motion
      +
Dynamic Motion
      +
Structural Connector Loads
      +
Control Energy
```

This prevents the learning objective from focusing only on a generic numerical reward.

Instead, the reward directly reflects quantities relevant to the physical system.

---

# 21. Why Physics-Aware MARL?

A conventional RL reward could simply penalize displacement:

```text
r = -z²
```

However, such a controller could potentially reduce displacement while producing excessive connector forces or control effort.

The physics-aware objective instead considers:

```text
Motion Reduction
       +
Velocity Reduction
       +
Connector Load Reduction
       +
Control-Effort Reduction
```

The learning problem therefore represents a multi-objective physical control problem.

---

# 22. MAPPO Actor Network

The decentralized actor uses a compact neural network:

```text
Local Observation
       |
       v
   Linear Layer
       |
      Tanh
       |
   Linear Layer
       |
      Tanh
       |
   Linear Layer
       |
       v
   Action Mean
```

A Gaussian action distribution is used during training.

The final action is bounded to the valid normalized control range.

---

# 23. Centralized Critic

The critic receives the complete global system state:

```text
x =
[z_1 ... z_N
 z_dot_1 ... z_dot_N]
```

Architecture:

```text
Global State
     |
     v
Linear Layer
     |
    Tanh
     |
Linear Layer
     |
    Tanh
     |
Linear Layer
     |
     v
State Value
```

This allows the critic to use information from the complete coupled structure during training.

---

# 24. PPO Update

The actor is trained using the clipped PPO objective.

Conceptually:

```text
ratio =
pi_new(a|s) / pi_old(a|s)
```

and:

```text
L_actor =
-min(
 ratio * A,
 clip(ratio, 1-epsilon, 1+epsilon) * A
)
```

The critic minimizes the state-value prediction error:

```text
L_critic =
(V(s) - R)²
```

The implementation is intentionally compact so that the learning architecture remains easy to inspect and modify.

---

# 25. Robustness to Model Uncertainty

The project supports randomized physical parameters.

For example:

```text
m = m_0 (1 + delta_m)
```

```text
c = c_0 (1 + delta_c)
```

```text
k = k_0 (1 + delta_k)
```

and similar uncertainty can be applied to connector parameters.

The current Monte Carlo example uses uncertainty up to:

```text
±20%
```

This allows evaluation of controller sensitivity to modelling errors.

---

# 26. Monte Carlo Analysis

The repository includes a Monte Carlo evaluation example.

The default study executes:

```text
100 randomized simulations
```

with uncertain physical parameters.

The process is:

```text
Nominal Model
     ↓
Randomize Parameters
     ↓
Run Simulation
     ↓
Calculate Metrics
     ↓
Repeat
     ↓
Mean + Standard Deviation
```

This provides a statistical robustness assessment rather than relying on a single deterministic simulation.

---

# 27. Evaluation Metrics

Several quantitative performance metrics are implemented.

## Heave RMS

```text
RMS_z =
sqrt(mean(z²))
```

This measures the overall vertical motion of the modular structure.

---

## Connector Force RMS

```text
RMS_F =
sqrt(mean(F_connector²))
```

This quantifies average dynamic connector loading.

---

## Peak Connector Load

```text
F_peak =
max |F_connector|
```

This captures the maximum connector force observed during the simulation.

---

## Control Energy

A control-effort metric is calculated using:

```text
E_u =
Σ u²
```

This provides a relative measure of actuator usage.

---

## Maximum Displacement

```text
z_max =
max |z_i|
```

This captures the largest module displacement during the simulation.

---

## Constraint Violations

The simulation counts commands exceeding the configured actuator-force limit.

With proper saturation:

```text
|u_i| <= u_max
```

the expected number of actuator-limit violations should remain zero.

---

# 28. Experimental Comparison

The intended comparison is:

| Controller | Model-Based | Constraints | Learning | Multi-Agent |
|---|---:|---:|---:|---:|
| Uncontrolled | — | — | — | — |
| LQR | ✓ | Saturation | — | — |
| MPC | ✓ | ✓ | — | — |
| Physics-Aware MAPPO | Hybrid | Action bounds | ✓ | ✓ |

The repository provides the framework for obtaining quantitative results from these controllers.

No fabricated numerical performance results are included.

---

# 29. Environmental Test Scenarios

The framework can be used to investigate different disturbance levels.

For example:

```text
Scenario 1
Low environmental disturbance

Scenario 2
Nominal environmental disturbance

Scenario 3
High environmental disturbance
```

The baseline script evaluates multiple disturbance-severity values.

Further scenarios can be introduced by modifying the environment parameters.

---

# 30. Repository Structure

```text
physics_aware_marl_floating_structures/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   ├── __init__.py
│   ├── dynamics.py
│   ├── environment.py
│   ├── controllers.py
│   ├── mappo.py
│   └── evaluation.py
│
└── examples/
    ├── run_baselines.py
    ├── train_mappo.py
    └── monte_carlo.py
```

---

# 31. Module Description

| Module | Purpose |
|---|---|
| `dynamics.py` | Coupled floating-module dynamics and modal analysis |
| `environment.py` | MARL environment and stochastic disturbances |
| `controllers.py` | LQR and receding-horizon MPC |
| `mappo.py` | Physics-aware CTDE MAPPO architecture |
| `evaluation.py` | Performance metrics and controller evaluation |
| `run_baselines.py` | Uncontrolled/LQR/MPC comparison |
| `train_mappo.py` | MAPPO training |
| `monte_carlo.py` | Parameter-uncertainty robustness analysis |

---

# 32. Installation

Clone the repository:

```bash
git clone <repository-url>
cd physics_aware_marl_floating_structures
```

Install the dependencies:

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
NumPy
SciPy
PyTorch
Matplotlib
```

---

# 33. Running Classical Baselines

Run:

```bash
python examples/run_baselines.py
```

The script evaluates:

```text
Uncontrolled
LQR
MPC
```

under representative environmental conditions.

The output includes quantitative performance metrics.

---

# 34. Training Physics-Aware MAPPO

Run:

```bash
python examples/train_mappo.py
```

The training loop performs:

```text
Environment Reset
       ↓
Local Observations
       ↓
Decentralized Actor
       ↓
Control Actions
       ↓
Dynamic Simulation
       ↓
Physics-Aware Reward
       ↓
Trajectory Collection
       ↓
PPO Update
       ↓
Next Episode
```

After training, the actor network is saved as:

```text
mappo_actor.pt
```

---

# 35. Running Monte Carlo Analysis

Run:

```bash
python examples/monte_carlo.py
```

The example performs repeated simulations with parameter uncertainty.

For each metric, it reports:

```text
Mean
Standard Deviation
```

across the Monte Carlo population.

---

# 36. Recommended Result Figures

After running the simulations, the following figures would be useful additions to the repository:

```text
results/
├── module_heave_response.png
├── connector_force_comparison.png
├── lqr_vs_mpc_vs_mappo.png
├── mappo_training_curve.png
├── control_effort.png
├── modal_analysis.png
└── monte_carlo_boxplot.png
```

These should be generated from actual simulation outputs rather than manually specified example values.

---

# 37. Recommended Result Table

Once the experiments have been executed, the controllers can be compared using:

| Method | Heave RMS | Connector RMS | Peak Load | Control Energy | Violations |
|---|---:|---:|---:|---:|---:|
| Uncontrolled | measured | measured | measured | — | — |
| LQR | measured | measured | measured | measured | measured |
| MPC | measured | measured | measured | measured | measured |
| Physics-Aware MAPPO | measured | measured | measured | measured | measured |

Only measured simulation results should be inserted into this table.

---

# 38. Technologies

- Python
- NumPy
- SciPy
- PyTorch
- State-Space Modelling
- Linear Quadratic Regulation
- Model Predictive Control
- Multi-Agent Reinforcement Learning
- Proximal Policy Optimization
- Monte Carlo Simulation

---

# 39. Research Areas

This project is related to:

- Marine Control Systems
- Floating Structures
- Multi-Agent Reinforcement Learning
- Physics-Aware AI
- Distributed Control
- Optimal Control
- Model Predictive Control
- Dynamic System Modelling
- Structural Load Mitigation
- Cooperative Systems
- Robust Control
- Simulation-Based Engineering

---

# 40. Current Scope

The current implementation includes:

- Multi-module coupled heave model
- Spring-damper connector model
- State-space formulation
- Modal-analysis functionality
- Stochastic environmental forcing
- Variable disturbance severity
- LQR baseline
- Constrained finite-horizon MPC baseline
- Multi-agent environment
- CTDE-style MAPPO
- Shared decentralized actor
- Centralized critic
- Physics-aware reward
- Control-force limits
- Parameter uncertainty
- Monte Carlo evaluation
- Quantitative performance metrics

---

# 41. Current Limitations

The current public model intentionally does not include:

- Full 6-DoF hydrodynamics
- Radiation-memory effects
- Frequency-dependent added mass
- Hydrodynamic interaction coefficients
- CFD coupling
- Mooring-line dynamics
- Experimental tank-test data
- Real wave-basin measurements
- Real actuator dynamics
- Validated structural connector models
- Hardware-in-the-loop testing

These limitations are important when interpreting the simulation results.

---

# 42. Future Extensions

## Higher-Fidelity Hydrodynamics

The low-order model could be extended toward:

```text
6-DoF Module Dynamics
        +
Added Mass
        +
Radiation Damping
        +
Wave Excitation
        +
Hydrodynamic Coupling
```

---

## State Estimation

Noisy sensor measurements could be introduced:

```text
y = Cx + v
```

followed by:

```text
Kalman Filter
      ↓
Estimated State
      ↓
LQR / MPC / MAPPO
```

Possible estimators include:

- Kalman Filter
- Extended Kalman Filter
- Unscented Kalman Filter

---

## Distributed MPC

The centralized MPC architecture could be extended to:

```text
Module 1 MPC ←→ Module 2 MPC ←→ Module 3 MPC
```

with neighboring modules exchanging limited state or predicted-trajectory information.

---

## Variable-Stiffness Connectors

The connector stiffness could become a controllable variable:

```text
k_c = k_c(t)
```

allowing the system to investigate adaptive structural coupling.

---

## Communication Constraints

Multi-agent coordination could be evaluated under:

- Communication delay
- Packet loss
- Limited communication range
- Partial observations
- Neighbor-state uncertainty

---

## Actuator Faults

Robustness studies could include:

```text
u_available =
alpha u_command
```

where:

```text
0 <= alpha <= 1
```

represents partial actuator degradation.

---

## Digital-Twin Extension

A genuine digital-twin study would require connection to higher-fidelity or physical reference data.

A future architecture could be:

```text
Physical / High-Fidelity System
              ↓
          Sensor Data
              ↓
       State Estimation
              ↓
     Parameter Estimation
              ↓
       Dynamic Model
              ↓
      Control / MARL
              ↓
        Model Update
```

The current repository should therefore be described as a **simulation framework**, not as a validated digital twin.

---

# 43. Research Development Path

The project can evolve through four stages:

```text
Phase 1
Dynamic Modelling
     ↓
State-Space + Modal Analysis

Phase 2
Model-Based Control
     ↓
LQR + MPC

Phase 3
Intelligent Control
     ↓
Physics-Aware MAPPO

Phase 4
Validation
     ↓
High-Fidelity / Experimental Data
```

This provides a clear path from fundamental dynamic modelling to data-supported intelligent control.

---

# 44. Public Implementation Notice

The source code in this repository contains **generic research and educational implementations**.

The public version intentionally excludes:

- Proprietary platform parameters
- Restricted vessel information
- Operational deployment configurations
- Confidential environmental datasets
- Platform-specific actuator models
- Unpublished experimental datasets
- Sensitive system configurations

All physical parameters and environmental disturbances in the public implementation are generic simulation values.

---

# 45. Status

**Research-oriented simulation framework / active development**

The current project demonstrates:

```text
Physics-Based Dynamic Model
          ↓
Classical Optimal Control
          ↓
Constrained Predictive Control
          ↓
Multi-Agent Reinforcement Learning
          ↓
Physics-Aware Objective Design
          ↓
Robustness Evaluation
```

The next research step is to validate the control and learning architecture against higher-fidelity hydrodynamic or experimental reference data.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Marine Control Systems
- Guidance, Navigation and Control
- Multi-Agent Reinforcement Learning
- Physics-Aware AI
- Model Predictive Control
- Dynamic System Modelling
- State Estimation
- Cooperative Autonomy
- Digital-Twin-Based Control

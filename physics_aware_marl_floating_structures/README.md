# Physics-Aware Multi-Agent Reinforcement Learning for Distributed Control of Modular Floating Structures

Research-oriented simulation framework for cooperative control of interconnected floating modules under stochastic marine disturbances.

## Implemented
- Coupled multi-module heave dynamics
- Spring-damper connector loads
- State-space formulation and modal analysis
- Stochastic wave-like disturbances
- Centralized LQR baseline
- Constrained receding-horizon MPC baseline
- CTDE-style MAPPO: shared decentralized actor + centralized critic
- Physics-aware reward: motion + connector loads + control effort
- ±20% model-parameter uncertainty and Monte Carlo evaluation

## Model
For each module:

`m_i z_ddot_i + c_i z_dot_i + k_i z_i + F_connector,i = F_env,i + u_i`

Connector:

`F_c,ij = k_c(z_i-z_j) + c_c(z_dot_i-z_dot_j)`

System:

`M q_ddot + C q_dot + K q = B u + F_env`

## Control comparison
`Uncontrolled -> LQR -> MPC -> Physics-Aware MAPPO`

## Metrics
- Heave RMS
- Connector-force RMS
- Peak connector load
- Control energy
- Maximum displacement
- Constraint violations

## Run
```bash
pip install -r requirements.txt
python examples/run_baselines.py
python examples/train_mappo.py
python examples/monte_carlo.py
```

## Scope
This is a generic low-order research simulator. It does not claim experimentally validated hydrodynamic coefficients, tank-test validation, a real platform model, or a full digital twin. Those require external physical/high-fidelity reference data.

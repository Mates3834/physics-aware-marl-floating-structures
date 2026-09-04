import numpy as np
from src.environment import FloatingMARLEnv
from src.controllers import LQRController,MPCController
from src.evaluation import simulate
for severity in (.5,1.,1.5):
    e=FloatingMARLEnv(severity=severity,seed=4); print("\nSeverity",severity,"Uncontrolled",simulate(e,lambda x:np.zeros(e.n)))
    e=FloatingMARLEnv(severity=severity,seed=4); print("LQR",simulate(e,LQRController(e.model)))
e=FloatingMARLEnv(severity=1.,horizon=100,seed=4); print("\nMPC",simulate(e,MPCController(e.model,dt=e.dt)))

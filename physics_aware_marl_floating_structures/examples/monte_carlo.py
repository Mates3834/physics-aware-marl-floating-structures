import numpy as np
from src.environment import FloatingMARLEnv
from src.controllers import LQRController
from src.evaluation import simulate
R=[]
for seed in range(100):
    e=FloatingMARLEnv(severity=1.2,uncertainty=.20,seed=seed); R.append(simulate(e,LQRController(e.model)))
for k in R[0]:
    a=np.array([r[k] for r in R],float); print(k,"mean=",a.mean(),"std=",a.std())

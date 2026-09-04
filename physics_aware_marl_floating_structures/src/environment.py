import numpy as np
from .dynamics import FloatingStructureModel,ModelParameters

class FloatingMARLEnv:
    def __init__(self,n=3,dt=.1,horizon=300,severity=1.,uncertainty=0.,seed=1):
        r=np.random.default_rng(seed); s=lambda:1+r.uniform(-uncertainty,uncertainty)
        p=ModelParameters(n=n,mass=1e5*s(),damping=2e4*s(),restoring=8e4*s(),connector_k=5e4*s(),connector_c=1e4*s())
        self.model=FloatingStructureModel(p); self.n=n; self.dt=dt; self.horizon=horizon; self.severity=severity
        self.rng=r; self.phase=r.uniform(0,2*np.pi,n); self.freq=r.uniform(.35,.9,n); self.umax=5e4
        self.reset()
    def reset(self):
        self.x=np.zeros(2*self.n); self.k=0; return self.obs(),self.x.copy()
    def obs(self):
        z=self.x[:self.n]; v=self.x[self.n:]; out=[]
        for i in range(self.n):
            out.append([z[i],v[i],z[i-1]-z[i] if i else 0,z[i+1]-z[i] if i<self.n-1 else 0])
        return np.asarray(out,dtype=np.float32)
    def disturbance(self):
        t=self.k*self.dt
        return 2.5e4*self.severity*np.sin(self.freq*t+self.phase)+self.rng.normal(0,3e3*self.severity,self.n)
    def step(self,a):
        u=np.clip(np.asarray(a).reshape(self.n),-1,1)*self.umax
        self.x=self.model.step(self.x,u,self.disturbance(),self.dt); fc=self.model.connector_forces(self.x)
        z=self.x[:self.n]; v=self.x[self.n:]
        cost=8*np.mean(z*z)+1.5*np.mean(v*v)+2e-10*np.mean(fc*fc)+2e-10*np.mean(u*u)
        self.k+=1
        return self.obs(),self.x.copy(),-float(cost),self.k>=self.horizon,{"u":u,"connector":fc}

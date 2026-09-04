from dataclasses import dataclass
import numpy as np
from scipy.linalg import eig

@dataclass
class ModelParameters:
    n:int=3; mass:float=1e5; damping:float=2e4; restoring:float=8e4
    connector_k:float=5e4; connector_c:float=1e4

class FloatingStructureModel:
    def __init__(self,p=ModelParameters()):
        self.p=p; n=p.n
        self.M=np.eye(n)*p.mass; self.C=np.eye(n)*p.damping; self.K=np.eye(n)*p.restoring
        for i in range(n-1):
            for A,v in ((self.K,p.connector_k),(self.C,p.connector_c)):
                A[i,i]+=v; A[i+1,i+1]+=v; A[i,i+1]-=v; A[i+1,i]-=v
        Mi=np.linalg.inv(self.M)
        self.A=np.block([[np.zeros((n,n)),np.eye(n)],[-Mi@self.K,-Mi@self.C]])
        self.B=np.vstack([np.zeros((n,n)),Mi]); self.E=self.B.copy()
    def f(self,x,u,d): return self.A@x+self.B@u+self.E@d
    def step(self,x,u,d,dt):
        k1=self.f(x,u,d); k2=self.f(x+.5*dt*k1,u,d)
        k3=self.f(x+.5*dt*k2,u,d); k4=self.f(x+dt*k3,u,d)
        return x+dt*(k1+2*k2+2*k3+k4)/6
    def connector_forces(self,x):
        n=self.p.n; z=x[:n]; v=x[n:]
        return np.array([self.p.connector_k*(z[i]-z[i+1])+self.p.connector_c*(v[i]-v[i+1]) for i in range(n-1)])
    def modal_analysis(self):
        vals,vecs=eig(self.K,self.M); w=np.sqrt(np.maximum(np.real(vals),0)); idx=np.argsort(w)
        return w[idx],np.real(vecs[:,idx])

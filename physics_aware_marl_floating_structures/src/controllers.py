import numpy as np
from scipy.linalg import solve_continuous_are
from scipy.optimize import minimize

class LQRController:
    def __init__(self,m,umax=5e4):
        n=m.p.n; Q=np.diag([15.]*n+[3.]*n); R=np.eye(n)*2e-8
        P=solve_continuous_are(m.A,m.B,Q,R); self.K=np.linalg.solve(R,m.B.T@P); self.umax=umax
    def __call__(self,x): return np.clip(-self.K@x,-self.umax,self.umax)

class MPCController:
    def __init__(self,m,dt=.1,H=5,umax=5e4): self.m=m; self.dt=dt; self.H=H; self.umax=umax; self.n=m.p.n
    def __call__(self,x):
        def J(flat):
            s=x.copy(); total=0.; U=flat.reshape(self.H,self.n)
            for u in U:
                s=self.m.step(s,u,np.zeros(self.n),self.dt); fc=self.m.connector_forces(s)
                total+=12*np.sum(s[:self.n]**2)+2*np.sum(s[self.n:]**2)+2e-10*np.sum(fc**2)+1e-10*np.sum(u**2)
            return total
        res=minimize(J,np.zeros(self.H*self.n),bounds=[(-self.umax,self.umax)]*(self.H*self.n),method="L-BFGS-B",options={"maxiter":30})
        return res.x.reshape(self.H,self.n)[0]

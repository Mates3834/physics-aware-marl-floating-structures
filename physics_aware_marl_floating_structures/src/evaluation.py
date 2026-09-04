import numpy as np
def metrics(S,U,F,umax):
    S=np.asarray(S); U=np.asarray(U); F=np.asarray(F); n=U.shape[1]; z=S[:,:n]
    return {"heave_rms":float(np.sqrt(np.mean(z*z))),"connector_rms":float(np.sqrt(np.mean(F*F))),
            "peak_connector_load":float(np.max(np.abs(F))),"control_energy":float(np.sum(U*U)),
            "max_displacement":float(np.max(np.abs(z))),"constraint_violations":int(np.sum(np.abs(U)>umax+1e-6))}
def simulate(env,controller):
    _,s=env.reset(); S=[]; U=[]; F=[]; done=False
    while not done:
        u=np.asarray(controller(s)); _,s,_,done,info=env.step(np.clip(u/env.umax,-1,1))
        S.append(s.copy()); U.append(info["u"]); F.append(info["connector"])
    return metrics(S,U,F,env.umax)

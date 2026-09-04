import torch
from src.environment import FloatingMARLEnv
from src.mappo import MAPPO
env=FloatingMARLEnv(horizon=250,seed=3); agent=MAPPO(env.n,2*env.n)
for ep in range(80):
    o,s=env.reset(); b={"obs":[],"actions":[],"states":[],"logp":[],"rewards":[]}; done=False; total=0
    while not done:
        a,lp=agent.act(o,s); no,ns,r,done,_=env.step(a)
        b["obs"].append(o); b["actions"].append(a); b["states"].append(s); b["logp"].append(lp); b["rewards"].append(r)
        o,s=no,ns; total+=r
    b["returns"]=agent.returns(b["rewards"]); agent.update(b)
    if ep%10==0: print(ep,total)
torch.save(agent.actor.state_dict(),"mappo_actor.pt")

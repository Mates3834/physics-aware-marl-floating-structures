import numpy as np, torch
import torch.nn as nn
from torch.distributions import Normal

class Actor(nn.Module):
    def __init__(self,d=4):
        super().__init__(); self.net=nn.Sequential(nn.Linear(d,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1)); self.log_std=nn.Parameter(torch.tensor([-.5]))
    def dist(self,o):
        mean=torch.tanh(self.net(o)); return Normal(mean,self.log_std.exp().expand_as(mean))
class Critic(nn.Module):
    def __init__(self,d):
        super().__init__(); self.net=nn.Sequential(nn.Linear(d,128),nn.Tanh(),nn.Linear(128,128),nn.Tanh(),nn.Linear(128,1))
    def forward(self,x): return self.net(x)
class MAPPO:
    def __init__(self,n,state_dim,lr=3e-4,gamma=.99,clip=.2):
        self.n=n; self.gamma=gamma; self.clip=clip; self.actor=Actor(); self.critic=Critic(state_dim)
        self.oa=torch.optim.Adam(self.actor.parameters(),lr=lr); self.oc=torch.optim.Adam(self.critic.parameters(),lr=lr)
    @torch.no_grad()
    def act(self,obs,state):
        d=self.actor.dist(torch.tensor(obs,dtype=torch.float32)); raw=d.sample(); a=torch.clamp(raw,-1,1)
        return a.squeeze(-1).numpy(),float(d.log_prob(raw).sum())
    def returns(self,rewards):
        out=[]; g=0
        for r in rewards[::-1]: g=r+self.gamma*g; out.append(g)
        return out[::-1]
    def update(self,b,epochs=6):
        O=torch.tensor(np.asarray(b["obs"]),dtype=torch.float32); A=torch.tensor(np.asarray(b["actions"]),dtype=torch.float32)
        S=torch.tensor(np.asarray(b["states"]),dtype=torch.float32); old=torch.tensor(b["logp"],dtype=torch.float32); R=torch.tensor(b["returns"],dtype=torch.float32)
        for _ in range(epochs):
            d=self.actor.dist(O.reshape(-1,O.shape[-1])); lp=d.log_prob(A.reshape(-1,1)).reshape(len(O),self.n).sum(1)
            V=self.critic(S).squeeze(); adv=(R-V).detach(); ratio=torch.exp(lp-old)
            la=-torch.min(ratio*adv,torch.clamp(ratio,1-self.clip,1+self.clip)*adv).mean(); lc=((V-R)**2).mean()
            self.oa.zero_grad(); la.backward(); self.oa.step(); self.oc.zero_grad(); lc.backward(); self.oc.step()

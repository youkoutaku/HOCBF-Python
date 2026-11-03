"""Independently implemented linear-class-K high-order CBF recursion."""
import casadi as ca
import numpy as np
from .models import box
from .qp import quadratic

class HOCBF:
    def __init__(self,model,obstacles,radii,gains=(1.5,1.3)):
        self.model=model;self.obstacles=np.asarray(obstacles).reshape(-1,2);self.radii=np.asarray(radii)
        if len(self.radii)!=len(self.obstacles):raise ValueError('obstacle/radius mismatch')
        degree=2 if model.kind in ('double_integrator','dynamic_unicycle') else 1
        self.degree=degree;self.gains=np.asarray(gains[:degree])
        if len(self.gains)!=degree or np.any(self.gains<=0):raise ValueError('One positive gain per derivative is required')
        x=ca.SX.sym('x',model.nx);o=ca.SX.sym('o',2);r=ca.SX.sym('r');f,g=model.affine(x)
        h=ca.sumsqr(model.center(x)-o)-r*r;psi=h;nested=[h]
        for k in self.gains[:-1]:
            channel=ca.simplify(ca.jacobian(psi,x)@g)
            if not channel.is_zero():raise ValueError('Chosen relative degree is invalid: input appears in an earlier derivative')
            psi=ca.jacobian(psi,x)@f+k*psi;nested.append(psi)
        normal=ca.jacobian(psi,x)@g;constant=ca.jacobian(psi,x)@f+self.gains[-1]*psi
        self.rows=ca.Function('hocbf_rows',[x,o,r],[normal,constant,ca.vertcat(*nested)])

    def action(self,x,goal,dt,t=0.):
        nominal=self.model.nominal(x,goal,dt);A,b=box(*self.model.bounds(x,dt));membership=[]
        for o,r in zip(self.obstacles,self.radii):
            normal,c,nested=self.rows(x,o,r);A=np.vstack([A,-np.asarray(normal).reshape(1,2)]);b=np.r_[b,float(c)];membership.extend(np.asarray(nested).ravel().tolist())
        u,_=quadratic(np.eye(2),-nominal,A,b)
        return u,{'status':'ok' if u is not None else 'qp_infeasible','relative_degree':self.degree,'initial_nested_set':membership,'nested_set_member':bool(np.all(np.array(membership)>=-1e-9)),'minimum_constraint_residual':None if u is None else float(np.min(b-A@u)),'nominal':nominal.tolist()}

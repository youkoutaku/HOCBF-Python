"""Deterministic numerical validation; no claims of theorem certification."""
import numpy as np
from .models import Model,KINDS
from .qp import quadratic

def common_checks():
    rows=[]
    for kind in KINDS:
        m=Model(kind);x=m.initial();u=np.array([.1,.15]);dt=.04
        numerical=m.integrate(x,u,dt,2)[-1];pred=np.asarray(m.symbolic_step(x,u,dt)).ravel();err=float(np.max(abs(numerical-pred)))
        rows.append({'test':'prediction_vs_independent_DOP853','model':kind,'error':err,'passed':err<1e-8})
    # A non-axis-aligned polygon with active constraints, independent Clarabel.
    import clarabel
    from scipy import sparse
    rng=np.random.default_rng(7)
    for i in range(20):
        A=np.vstack([np.eye(2),-np.eye(2),rng.normal(size=(5,2))]);b=np.r_[np.ones(4),rng.uniform(.05,1,5)]
        H=np.diag(rng.uniform(.2,3,2));q=rng.normal(size=2)*3
        u,_=quadratic(H,q,A,b);s=clarabel.DefaultSettings();s.verbose=False;s.tol_feas=1e-10;s.tol_gap_abs=1e-10;s.tol_gap_rel=1e-10
        ref=clarabel.DefaultSolver(sparse.csc_matrix(H),q,sparse.csc_matrix(A),b,[clarabel.NonnegativeConeT(len(b))],s).solve();error=float(np.max(abs(u-np.asarray(ref.x))))
        rows.append({'test':'active_set_vs_clarabel','case':i,'error':error,'passed':error<1e-5})
    u,_=quadratic(np.eye(2),np.zeros(2),np.array([[1.,0.],[-1.,0.]]),np.array([-1.,-1.]))
    rows.append({'test':'infeasible_problem_retains_no_control','passed':u is None})
    return rows

def verify():
    rows=common_checks()+method_checks()
    return {'passed':all(r['passed'] for r in rows),'checks':len(rows),'records':rows,'scope':'Equations, model contracts and independent numerical solver checks; not universal closed-loop safety or exact author figure reproduction.'}

def method_checks():
    from .controller import HOCBF
    rows=[];rng=np.random.default_rng(13);o=np.array([3.,-1.]);r=.4;k1,k2=1.5,1.3
    for kind in KINDS:
        m=Model(kind);c=HOCBF(m,[o],[r],(k1,k2))
        for i in range(20):
            x=m.initial()+rng.normal(size=m.nx)*.1
            if kind=='double_integrator':x[2:]=[.3,.2]
            if kind=='dynamic_unicycle':x[3:]=[.4,.2]
            n,c0,_=c.rows(x,o,r);d=m.center(x)-o;h=d@d-r*r
            if kind=='single_integrator':normal=2*d;constant=k1*h
            elif kind=='double_integrator':hd=2*d@x[2:];normal=2*d;constant=2*x[2:]@x[2:]+(k1+k2)*hd+k1*k2*h
            else:
                direction=np.array([np.cos(x[2]),np.sin(x[2])]);perp=np.array([-direction[1],direction[0]])
                G=np.column_stack([direction,m.offset*perp]);normal=2*d@G
                if kind=='kinematic_unicycle':constant=k1*h
                else:
                    velocity=x[3]*direction+m.offset*x[4]*perp;acc=x[3]*x[4]*perp-m.offset*x[4]**2*direction
                    constant=2*velocity@velocity+2*d@acc+(k1+k2)*2*d@velocity+k1*k2*h
            err=max(float(np.max(abs(np.asarray(n).ravel()-normal))),abs(float(c0)-constant))
            rows.append({'test':'Lie_recursion_vs_closed_form','model':kind,'case':i,'error':err,'passed':err<1e-10})
    c=HOCBF(Model('dynamic_unicycle',offset=0.),[o],[r]);normal,_,_=c.rows([0,0,.4,.5,.3],o,r)
    rows.append({'test':'zero_offset_angular_channel','passed':float(normal[1])==0.})
    return rows

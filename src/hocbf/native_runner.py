"""Run the local-only strict Python port against saved original numeric outputs."""
from pathlib import Path
import gzip,importlib.util,json
import numpy as np

def original(output,suite='all'):
    private=Path.cwd()/'.local-reference/hocbf_source_port.py'
    if not private.exists():
        raise RuntimeError('Strict author-source translation is not distributed publicly because upstream provides no license. This local checkout includes it in ignored .local-reference. Public equation demos and verify work independently. See THIRD_PARTY_NOTICES.md.')
    spec=importlib.util.spec_from_file_location('_private_source_port',private);port=importlib.util.module_from_spec(spec);spec.loader.exec_module(port)
    data=Path(__file__).parent/'data'
    with gzip.open(data/'original_qps.json.gz','rt') as f:records=json.load(f)['records']
    reference=np.load(data/'original_trajectories.npz',allow_pickle=False);out=Path(output)
    if out.exists() and any(out.iterdir()):raise FileExistsError(out)
    out.mkdir(parents=True,exist_ok=True);summaries=[]
    for name in (['acc','robot2d'] if suite=='all' else [suite]):
        matrix_error=0.;control_error=0.;cost_error=0.
        for q in records[name]:
            if name=='acc':H,F,A,b=port.acc_qp(q['state']);nu=1;normalizer=1650.
            else:
                s=q['sense'];sense=np.column_stack([np.atleast_1d(s['x']),np.atleast_1d(s['y'])]);H,F,A,b,_,_=port.robot_qp(q['state'],q['dst'],sense);nu=2;normalizer=1.
            for key,value in [('H',H),('F',F),('A',A),('b',b)]:matrix_error=max(matrix_error,float(np.max(abs(value-np.asarray(q[key]).reshape(value.shape)))))
            u,info=port.solve_qp(H,F,A,b);ref=np.asarray(q['u']);control_error=max(control_error,float(np.max(abs(u[:nu]-ref[:nu]))/normalizer));obj=.5*ref@H@ref+F@ref;cost_error=max(cost_error,abs(info['objective']-obj)/max(1.,abs(obj)))
        result=port.simulate_acc() if name=='acc' else port.simulate_robot(reference['robot_noise'])
        if name=='acc':
            ref=reference['acc_result1'];state_error=float(np.max(abs(result['result1'][:,[0,1,3]]-ref[:,[0,1,3]])));loop_control_error=float(np.max(abs(result['result1'][:,2]-ref[:,2]))/1650.)
        else:
            state_error=float(np.max(abs(result['states'][1:,:2]-reference['robot_position'])));loop_control_error=float(np.max(abs(result['controls'][:,:2]-reference['robot_controls'])))
        np.savez_compressed(out/(name+'.npz'),**{k:v for k,v in result.items() if isinstance(v,np.ndarray)})
        summaries.append({'example':name,'qps':len(records[name]),'matrix_error':matrix_error,'normalized_physical_control_error':control_error,'relative_objective_error':cost_error,'closed_loop_state_error':state_error,'closed_loop_control_error':loop_control_error,'passed':matrix_error<1e-10 and control_error<1e-5 and cost_error<1e-6 and state_error<1e-3 and loop_control_error<1e-3})
    result={'passed':all(s['passed'] for s in summaries),'examples':summaries,'local_only_source_translation':True,'noise':'Exact recorded 2500 x 14 Octave noise realization, not a cross-language random-seed equivalence claim.'};(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');return result

import os
# One numerical thread per process, set before importing numerical packages.
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS','VECLIB_MAXIMUM_THREADS','NUMEXPR_NUM_THREADS']:os.environ[key]='1'
import argparse,json
PACKAGE='hocbf'
SUITES=['all', 'acc', 'robot2d']
from pathlib import Path

def main(argv=None):
    parser=argparse.ArgumentParser(description='HOCBF-Python: source reproduction and declared model extensions')
    sub=parser.add_subparsers(dest='command',required=True)
    run=sub.add_parser('run');run.add_argument('--model',choices=['single_integrator','double_integrator','kinematic_unicycle','dynamic_unicycle'],required=True);run.add_argument('--output',required=True);run.add_argument('--steps',type=int,default=400);run.add_argument('--dt',type=float,default=.05);run.add_argument('--offset',type=float,default=.15)
    demo=sub.add_parser('demo',help='Straight-ahead goal, one obstacle, all four models')
    demo.add_argument('--model',choices=['single_integrator','double_integrator','kinematic_unicycle','dynamic_unicycle'])
    demo.add_argument('--output',required=True);demo.add_argument('--steps',type=int,default=800)
    demo.add_argument('--dt',type=float,default=.05)
    demo.add_argument('--guidance',choices=['auto','direct'],default='auto')
    demo.add_argument('--no-animation',action='store_true')
    verify=sub.add_parser('verify');verify.add_argument('--output')
    native=sub.add_parser('original');native.add_argument('--suite',choices=SUITES,default='all');native.add_argument('--output',required=True)
    if PACKAGE!='vocbf':sub.add_parser('fetch-original')
    args=parser.parse_args(argv)
    if args.command=='run':
        if args.steps<=0 or args.dt<=0:parser.error('steps and dt must be positive')
        from .simulation import run as simulate
        value=simulate(args.model,args.output,args.steps,args.dt,args.offset)
    elif args.command=='demo':
        from .single_obstacle import run as demonstrate
        value=demonstrate(args.output,args.model,args.steps,args.dt,args.guidance,not args.no_animation)
    elif args.command=='verify':
        from .validation import verify as check
        from .simulation import jsonable
        value=jsonable(check())
        if args.output:
            p=Path(args.output)
            if p.exists():raise FileExistsError(p)
            p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in value.items() if k!='records'},indent=2));return 0 if value['passed'] else 1
    elif args.command=='original':
        from .native_runner import original
        value=original(args.output,args.suite)
    else:
        from .upstream import fetch
        value=fetch()
    print(json.dumps(value,indent=2,allow_nan=False));return 0 if value.get('passed',True) else 1

if __name__=='__main__':raise SystemExit(main())

# Model and evidence scope

All systems are written as `xdot = f(x) + g(x) u`. “First-order” and “second-order” below describe the physical position/input relation. State dimension, dynamical order and barrier relative degree are different concepts. These are four concrete planar examples, not a promise that every arbitrary nonlinear control-affine system is supported.

| Example directory | CLI model | State | Input and equations |
|---|---|---|---|
| first_order_affine | single_integrator | `[px, py]` | `[vx, vy]`, `pdot=u` |
| second_order_affine | double_integrator | `[px, py, vx, vy]` | `[ax, ay]`, `pdot=v`, `vdot=u` |
| kinematic_unicycle | kinematic_unicycle | `[qx, qy, theta]` | `[v, omega]`, `qdot=v[cos(theta),sin(theta)]`, `thetadot=omega` |
| dynamic_unicycle | dynamic_unicycle | `[qx, qy, theta, v, omega]` | `[a, alpha]`, additionally `vdot=a`, `omegadot=alpha` |

For both unicycles the obstacle geometry is measured at `p=q+ell[cos(theta),sin(theta)]`, with `ell=0.15 m` by default. `--offset 0` retains the axle-centered model and exposes its lost instantaneous angular channel where applicable. It is not silently replaced with a nonzero certificate offset.

The `Model.affine` interface exposes drift and control matrices. A custom model also needs a defined position output, dimensions, physical bounds, nominal controller and integration. Verify the appropriate barrier relative degree and assumptions before reusing the example controller. Changing f/g is not sufficient to inherit the paper theorem.

The demo observer uses independent DOP853 integration at 21 points per hold, saves every actual input and reports sampled collisions. This is not a validated continuous-time enclosure. MPC prediction uses exact integrators and RK4 for unicycles; ZOCBF unicycle prediction uses a numerical flow, not a proved integration-error bound. Continuous-time HOCBF/VOCBF constraints do not automatically certify held-input intersample safety. Failed optimizations and unsuccessful tasks remain visible.

Each `run` output contains `trajectory.npz`, `events.json`, `summary.json` and `trajectory.png`. Position is in metres, time in seconds, angles in radians. Solver verification, script completion, safe execution and arrival are separate statements. These examples do not reproduce or update the prior five-method ranking.

## Single-obstacle demo completion and guidance

The separate `demo` command requires body-center goal error below `0.15 m` plus the following stopping predicate for a continuous `0.5 s` dwell:

| Model | Additional stopping predicate |
|---|---|
| single_integrator | Euclidean input norm below `0.08 m/s` |
| double_integrator | Euclidean velocity norm below `0.08 m/s` |
| kinematic_unicycle | The existing numeric predicate `norm([v, omega]) < 0.08`; this combines components with different units and is a demo tolerance, not a physical speed norm |
| dynamic_unicycle | `abs(v) < 0.05 m/s` and `abs(omega) < 0.05 rad/s` |

All four straight-ahead demos use `vmax=0.7 m/s` (a componentwise bound for integrators), `wmax=1.8 rad/s`, `amax=0.9 m/s²`, `alphamax=1.5 rad/s²`, `offset=0.15 m`, and the unchanged HOCBF recursion. First- and second-order integrator nominal tracking stays unchanged. The two unicycles use offset-point velocity tracking and an explicit waypoint as documented in the [README](../README.md#single-obstacle-demo). `--guidance direct` disables that waypoint and retains the stopping-at-obstacle limitation. This scenario and its nominal guidance are separate from the original paper problems and the earlier diagonal-goal fixture.

## This method

Original author ACC and Robot2D MATLAB problems have a strict local Python port, checked against 2,800 recorded native QPs and full closed loops. Because no upstream license is present, this direct source translation stays in ignored .local-reference; it is not included in public archives or wheels.

Public code independently implements the HOCBF recursion with linear class-K functions and a nominal-tracking QP. Original Robot2D uses [x,y,theta,v] with [yaw-rate,acceleration], fractional powers, its own CLFs and source quirks; it is not the new five-state demo.

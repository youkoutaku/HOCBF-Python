"""Reproducible straight-ahead obstacle demo, separate from author experiments.

The unicycles use disclosed waypoint guidance because this local HOCBF filter
can stop at the obstacle under direct-goal tracking. The filter is unchanged.
"""
from pathlib import Path
import hashlib
import json
import platform
import sys

import numpy as np

from .controller import HOCBF
from .models import KINDS, Model
from .simulation import jsonable

GOAL = np.array([4.0, 0.0])
OBSTACLES = np.array([[2.0, 0.15]])
RADII = np.array([0.45])
WAYPOINT = np.array([2.5, -0.7])
TITLES = dict(zip(KINDS, ["Single integrator", "Double integrator",
                         "Kinematic unicycle", "Dynamic unicycle"]))


class DemoModel(Model):
    """Offset-point velocity tracking for the demo's two unicycles only."""

    def initial(self):
        x = np.zeros(self.nx)
        if "unicycle" in self.kind:
            x[0] = -self.offset
        return x

    def nominal(self, x, goal, dt=0.05):
        if "unicycle" not in self.kind:
            return super().nominal(x, goal, dt)
        desired = 0.7 * (np.asarray(goal) - self.center(x))
        heading = np.array([np.cos(x[2]), np.sin(x[2])])
        normal = np.array([-np.sin(x[2]), np.cos(x[2])])
        vref = np.clip(desired @ heading, 0.0, self.vmax)
        wref = np.clip(desired @ normal / self.offset, -self.wmax, self.wmax)
        u = np.array([vref, wref]) if self.kind == "kinematic_unicycle" else np.array([
            3.0 * (vref - x[3]), 4.0 * (wref - x[4])])
        return np.clip(u, *self.bounds(x, dt))


def source_hashes():
    root = Path(__file__).parent
    return {f"src/hocbf/{name}": hashlib.sha256((root / name).read_bytes()).hexdigest()
            for name in ("single_obstacle.py", "controller.py", "models.py", "qp.py",
                         "simulation.py", "__main__.py")}


def run_model(kind, output, steps=800, dt=0.05, guidance="auto"):
    """Execute one model and retain controls, dense samples and solver events."""
    if steps <= 0 or not np.isfinite(dt) or dt <= 0:
        raise ValueError("steps and dt must be positive and finite")
    if guidance not in ("auto", "direct"):
        raise ValueError("guidance must be auto or direct")
    out = Path(output)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Refusing to overwrite {out}")
    model = DemoModel(kind, vmax=0.7)
    controller = HOCBF(model, OBSTACLES, RADII)
    guided = guidance == "auto" and "unicycle" in kind
    x = model.initial()
    states, controls, times, events, dense_positions = [x.copy()], [], [0.0], [], []
    phase, dwell, outcome = "approach", 0.0, "timeout"
    minimum = float(np.min(np.linalg.norm(model.center(x) - OBSTACLES, axis=1) - RADII))
    for k in range(steps):
        position = model.center(x)
        if guided and phase == "approach" and position[0] >= 0.8:
            phase = "bypass"
        if guided and phase == "bypass" and np.linalg.norm(position - WAYPOINT) < 0.2:
            phase = "finish"
        target = WAYPOINT if phase == "bypass" else GOAL
        u, info = controller.action(x, target, dt, k * dt)
        info.update(time=k * dt, guidance_phase=phase if guided else "direct",
                    tracking_target=target.tolist())
        events.append(jsonable(info))
        if u is None:
            outcome = info.get("status", "no_control")
            break
        if not np.isfinite(u).all():
            outcome = "nonfinite_control"
            break
        samples = model.integrate(x, u, dt)
        centers = np.array([model.center(z) for z in samples])
        dense_positions.append(centers)
        clearance = np.linalg.norm(centers[:, None, :] - OBSTACLES[None, :, :], axis=2) - RADII
        minimum = min(minimum, float(clearance.min()))
        x = samples[-1]
        states.append(x.copy())
        controls.append(u.copy())
        times.append((k + 1) * dt)
        if minimum < -1e-6:
            outcome = "sampled_collision"
            break
        dwell = dwell + dt if model.settled(x, u, GOAL) else 0.0
        if dwell >= 0.5 - 1e-9:
            outcome = "goal_complete"
            break
    center = np.array([model.center(z) for z in states])
    controls_array = np.asarray(controls).reshape(-1, 2)
    summary = dict(
        scenario="single_obstacle", method="hocbf", model=kind, model_extension=True,
        guidance="one_waypoint" if guided else "direct_goal", initial=states[0].tolist(),
        goal=GOAL.tolist(), obstacles=OBSTACLES.tolist(), inflated_radii=RADII.tolist(),
        offset=model.offset, dt=dt, requested_steps=steps, executed_steps=len(controls),
        duration=len(controls) * dt, outcome=outcome,
        final_goal_error=float(np.linalg.norm(center[-1] - GOAL)),
        minimum_sampled_clearance=minimum, collision_sampled=minimum < -1e-6,
        settled_dwell=dwell, required_settled_dwell=0.5,
        maximum_lateral_excursion=float(np.max(np.abs(center[:, 1]))),
        filter_intervention_steps=sum(np.linalg.norm(u - event["nominal"]) > 1e-6
                                      for u, event in zip(controls_array, events)),
        solver_fallbacks=0, input_semantics="held physical control",
        model_parameters=dict(vmax=model.vmax, wmax=model.wmax, amax=model.amax,
                              alphamax=model.alphamax, offset=model.offset),
        controller_parameters=dict(gains=controller.gains.tolist()),
        nominal_tracking=("q=0.7*(target-center); inverse offset-point velocity; "
                          "dynamic velocity tracking gains (3,4)" if "unicycle" in kind
                          else "unchanged Model.nominal"),
        waypoint_guidance=(dict(waypoint=WAYPOINT.tolist(), activate_at_x=0.8,
                               switch_to_goal_distance=0.2) if guided else None),
        verification="21 independent DOP853 samples per hold; no interval enclosure or continuous-time safety proof",
        python=sys.version.split()[0], platform=platform.platform(), source_sha256=source_hashes())
    out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out / "trajectory.npz", time=np.asarray(times), state=np.asarray(states),
                        control=controls_array, center=center,
                        dense_center=np.asarray(dense_positions).reshape(-1, 21, 2))
    (out / "summary.json").write_text(json.dumps(jsonable(summary), indent=2, allow_nan=False) + "\n")
    (out / "events.json").write_text(json.dumps(events, indent=2, allow_nan=False) + "\n")
    return jsonable(summary)


def render(output, animate=True):
    """Render saved trajectories without rerunning controllers."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    out = Path(output)
    records = []
    for kind in KINDS:
        folder = out / kind
        if (folder / "summary.json").exists():
            s = json.loads((folder / "summary.json").read_text())
            with np.load(folder / "trajectory.npz", allow_pickle=False) as data:
                records.append((kind, s, data["time"].copy(), data["center"].copy()))
    if not records:
        raise ValueError(f"No model records in {out}")
    guidance_label = ("Unicycles: one explicit waypoint" if any(
        s["guidance"] == "one_waypoint" for _, s, _, _ in records)
        else "Direct goal: no waypoints")

    def setup(ax, kind, summary):
        ax.plot([0, 4], [0, 0], "--", color="#8c97a3", lw=1, label="Straight reference")
        ax.add_patch(plt.Circle(OBSTACLES[0], RADII[0], color="#b8bec7", alpha=0.7,
                               label="Inflated obstacle"))
        ax.scatter(0, 0, color="#253549", s=25, zorder=5, label="Start")
        ax.scatter(*GOAL, marker="*", color="#253549", s=100, zorder=5, label="Goal")
        if summary["guidance"] == "one_waypoint":
            ax.scatter(*WAYPOINT, marker="+", color="#99651b", s=50, label="Guidance waypoint")
        ax.set(xlim=(-0.25, 4.3), ylim=(-1.15, 0.95), xlabel="x [m]", ylabel="y [m]", aspect="equal")
        ax.set_title(TITLES[kind] + " | " + summary["guidance"].replace("_", " "), fontsize=11)
        ax.grid(alpha=0.15)

    fig, axes = plt.subplots(2, 2, figsize=(11, 6.7), layout="constrained")
    for ax in axes.flat:
        ax.set_visible(False)
    for ax, (kind, s, _, center) in zip(axes.flat, records):
        ax.set_visible(True)
        setup(ax, kind, s)
        ax.plot(center[:, 0], center[:, 1], color="#167b67", lw=2, label="Body center")
        ax.scatter(*center[-1], color="#167b67", marker="s", s=20, zorder=5)
        ax.text(0.02, 0.04, f"{s['outcome']} | {s['duration']:.2f} s | clearance {s['minimum_sampled_clearance']:.3f} m",
                transform=ax.transAxes, fontsize=9)
        single, single_ax = plt.subplots(figsize=(6.2, 3.7), layout="constrained")
        setup(single_ax, kind, s)
        single_ax.plot(center[:, 0], center[:, 1], color="#167b67", lw=2)
        single_ax.legend(fontsize=7, loc="lower left")
        single.savefig(out / kind / "trajectory.png", dpi=160)
        plt.close(single)
    axes.flat[0].legend(loc="upper left", fontsize=7, ncol=2)
    fig.suptitle("HOCBF | Straight-ahead goal with one obstacle\n"
                 + guidance_label + "; sampled numerical demonstration", fontsize=13)
    fig.savefig(out / "trajectories.png", dpi=160)
    plt.close(fig)
    if not animate:
        return
    fig, axes = plt.subplots(2, 2, figsize=(9, 5.6), layout="constrained")
    artists = []
    for ax in axes.flat:
        ax.set_visible(False)
    for ax, (kind, s, time, center) in zip(axes.flat, records):
        ax.set_visible(True)
        setup(ax, kind, s)
        line, = ax.plot([], [], color="#167b67", lw=2)
        dot, = ax.plot([], [], "o", color="#167b67", ms=5)
        status = ax.text(0.02, 0.04, "", transform=ax.transAxes, fontsize=8)
        artists.append((line, dot, status, time, center, s))
    fig.suptitle("HOCBF | " + guidance_label + "\nShared simulation clock", fontsize=13)
    duration = max(s["duration"] for _, s, _, _ in records)
    frames = np.linspace(0, duration, min(141, int(duration * 10) + 1))

    def update(t):
        result = []
        for line, dot, status, time, center, s in artists:
            index = min(int(np.searchsorted(time, t, side="right")), len(time))
            line.set_data(center[:index, 0], center[:index, 1])
            dot.set_data([center[index - 1, 0]], [center[index - 1, 1]])
            status.set_text(f"t = {t:.1f} s" + (" | " + s["outcome"] if t >= time[-1] else ""))
            result.extend([line, dot, status])
        return result

    animation = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)
    animation.save(out / "animation.gif", writer=PillowWriter(fps=10), dpi=90)
    plt.close(fig)


def run(output, model=None, steps=800, dt=0.05, guidance="auto", animate=True):
    out = Path(output)
    if out.exists() and any(out.iterdir()):
        raise FileExistsError(f"Refusing to overwrite {out}")
    kinds = [model] if model else KINDS
    results = [run_model(kind, out / kind, steps, dt, guidance) for kind in kinds]
    summary = dict(scenario="single_obstacle", method="hocbf", results=results,
                   all_goal_complete=all(s["outcome"] == "goal_complete" for s in results),
                   all_sampled_collision_free=all(not s["collision_sampled"] for s in results),
                   source_sha256=source_hashes(),
                   scope="Illustrative model extensions, not author experiments or a comparative benchmark")
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    render(out, animate=animate)
    return summary

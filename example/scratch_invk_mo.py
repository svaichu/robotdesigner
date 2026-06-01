import marimo

__generated_with = "0.22.0"
app = marimo.App()


@app.cell
def _():
    from robotdesigner.mujoco.composer import build_arm
    import gymnasium as gym

    import mujoco as mj
    import mjinx


    return build_arm, gym


@app.cell
def _(gym):
    gym.register(
            id="AbstractArm-v0",
            entry_point="robotdesigner.envs.abstract_mujoco_env:make_env",
        )
    return


@app.cell
def _(build_arm):
    spec = build_arm(ndof=3, control="pos")

    return (spec,)


@app.cell
def _(gym, spec):
    env = gym.make(
        "AbstractArm-v0",
        spec=spec,
        render_mode="rgb_array",
    )
    return


@app.cell
def _():
    import jax
    import numpy as np
    from mujoco import mjx
    from mjinx.problem import Problem
    from mjinx.components.tasks import FrameTask
    from mjinx.components.barriers import JointBarrier
    from mjinx.solvers import LocalIKSolver
    from mjinx.configuration import integrate

    return (
        FrameTask,
        JointBarrier,
        LocalIKSolver,
        Problem,
        integrate,
        jax,
        mjx,
        np,
    )


@app.cell
def _():
    import mujoco_warp as mjw

    return


@app.cell
def _():
    import mujoco_playground

    return


@app.cell
def _():
    # jax kernel choose between jax and warp

    return


@app.cell
def _(spec):
    mj_model = spec.compile()
    # mjx_model = mjx.put_model(mj_model)
    return (mj_model,)


@app.cell
def _(mj_model, mjx):
    mjx_model = mjx.put_model(mj_model)
    return (mjx_model,)


@app.cell
def _(Problem, mjx_model):
    problem = Problem(mjx_model)
    return (problem,)


@app.cell
def _(FrameTask, problem):
    frame_task = FrameTask("ee_task", cost=1, gain=20, obj_name="link3_link")
    problem.add_component(frame_task)
    return (frame_task,)


@app.cell
def _(JointBarrier, problem):
    joints_barrier = JointBarrier("jnt_range", gain=10)
    problem.add_component(joints_barrier)
    return


@app.cell
def _(LocalIKSolver, mjx_model):
    solver = LocalIKSolver(mjx_model)

    # Initial configuration
    dt = 1e-2
    return dt, solver


@app.cell
def _(integrate, jax, solver):
    solve_jit = jax.jit(solver.solve)
    integrate_jit = jax.jit(integrate, static_argnames=["dt"])
    return integrate_jit, solve_jit


@app.cell
def _(
    dt,
    frame_task,
    integrate_jit,
    mjx_model,
    np,
    problem,
    solve_jit,
    solver,
):
    q = np.zeros(mjx_model.nq)
    solver_data = solver.init(q)
    for t in np.arange(0, 5, dt):
        # Update target and compile problem
        frame_task.target_frame = np.array([0.1 * np.sin(t), 0.1 * np.cos(t), 0.1, 1, 0, 0, 0])
        problem_data = problem.compile()

        # Solve the IK problem
        opt_solution, solver_data = solve_jit(q, solver_data, problem_data)

        # Integrate to get new configuration
        q = integrate_jit(
            mjx_model,
            q,
            opt_solution.v_opt,
            dt,
        )
        print(f"Time: {t:.2f}, q: {q}")
    return


@app.cell
def _():
    return


@app.cell
def _():
    from types import SimpleNamespace
    class Experiment(SimpleNamespace):
        def __init__(self):
            super().__init__()
            q: int = 0

    e = Experiment()
    return (e,)


@app.cell
def _():
    return


@app.cell
def _(e):
    e.q = 4
    return


@app.cell
def _(e):
    e.q = 2
    return


if __name__ == "__main__":
    app.run()

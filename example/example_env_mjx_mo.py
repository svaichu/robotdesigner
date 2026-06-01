import marimo

__generated_with = "0.22.0"
app = marimo.App()


@app.cell
def _():
    import marimo as mo

    return (mo,)


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    # Robot Arm MJX Environment

    Build a robot arm spec and load the built-in MJX env by name.

    This uses the mujoco_playground functional API (no Gymnasium required):

    ```python
    state = env.reset(rng)
    state = env.step(state, act)
    ```
    """)
    return


@app.cell
def _(mo):
    ndof_slider = mo.ui.slider(1, 6, value=3, label="ndof (number of joints)")
    control_radio = mo.ui.radio(["pos", "torque"], value="pos", label="control")
    n_steps_slider = mo.ui.slider(10, 200, value=50, step=10, label="rollout steps")
    mo.hstack([ndof_slider, control_radio, n_steps_slider])
    return control_radio, n_steps_slider, ndof_slider


@app.cell
def _(control_radio, ndof_slider):
    from robotdesigner.mujoco.composer import build_arm

    spec = build_arm(ndof=ndof_slider.value, control=control_radio.value)
    print(f"model  : {spec.modelname}")
    print(f"ndof   : {ndof_slider.value}")
    print(f"control: {control_radio.value}")
    return (spec,)


@app.cell
def _(spec):
    from robotdesigner.envs import load_mjx_environment, list_mjx_environments

    ENV_NAME = "AbstractArmMjx"
    env = load_mjx_environment(ENV_NAME, spec=spec)
    print(f"available envs   : {list_mjx_environments()}")
    print(f"action_size      : {env.action_size}")
    print(f"observation_size : {env.observation_size}")
    print(f"sim_dt           : {env.sim_dt}")
    print(f"ctrl_dt          : {env.dt}")
    print(f"n_substeps       : {env.n_substeps}")
    return (env,)


@app.cell
def _(env):
    import jax
    import jax.numpy as jp

    rng = jax.random.PRNGKey(0)
    state = env.reset(rng)

    print("obs shape :", state.obs.shape)
    print("obs       :", state.obs)
    print("reward    :", state.reward)
    return jax, state


@app.cell
def _(state):
    state.info
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Single rollout

    Step the environment for the selected number of steps, collecting
    ee_pos, target_pos, reward, and done at each timestep.
    """)
    return


@app.cell
def _(env, jax):
    import pandas as pd

    _rng = jax.random.PRNGKey(42)
    _state = env.reset(_rng)

    import numpy as np
    env.set_target(np.array([0.2, 0.1, 0.3], dtype=np.float32))
    _state = env.reset(_rng)

    rows = []
    for _step in range(5):
        _action = jax.random.uniform(
            jax.random.fold_in(_rng, _step),
            (env.action_size,),
            minval=-1.0,
            maxval=1.0,
        )
        _state = env.step(_state, _action)
        _obs = _state.obs
        _done = bool(_state.done)
        rows.append({
            "step"    : int(_state.info["timestep"]),
            "reward"  : round(float(_state.reward), 4),
            "ee_x"    : round(float(_obs[0]), 3),
            "ee_y"    : round(float(_obs[1]), 3),
            "ee_z"    : round(float(_obs[2]), 3),
            "target_x": round(float(_obs[3]), 3),
            "target_y": round(float(_obs[4]), 3),
            "target_z": round(float(_obs[5]), 3),
            "done"    : _done,
        })
        if _done:
            break

    df = pd.DataFrame(rows)
    df
    return df, np


@app.cell
def _(df, mo):
    mo.ui.table(df)
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## JIT-compiled rollout

    jax.jit traces env.step once and compiles it to XLA.
    """)
    return


@app.cell
def _(env, jax, n_steps_slider, np):
    import time

    _jit_step = jax.jit(env.step)

    _rng = jax.random.PRNGKey(7)
    env.set_target(np.array([0.2, 0.1, 0.3], dtype=np.float32))
    _state = env.reset(_rng)

    _action0 = jax.random.uniform(_rng, (env.action_size,), minval=-1.0, maxval=1.0)
    _ = _jit_step(_state, _action0).obs.block_until_ready()

    _t0 = time.perf_counter()
    _jit_rows = []
    for _step in range(n_steps_slider.value):
        _action = jax.random.uniform(
            jax.random.fold_in(_rng, _step),
            (env.action_size,),
            minval=-1.0,
            maxval=1.0,
        )
        _state = _jit_step(_state, _action)
        _jit_rows.append(float(_state.reward))
        if bool(_state.done):
            break

    _state.obs.block_until_ready()
    _elapsed = time.perf_counter() - _t0
    print(f"JIT rollout ({n_steps_slider.value} steps): {_elapsed*1000:.2f} ms")
    print(f"mean reward : {sum(_jit_rows)/len(_jit_rows):.4f}")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Batched rollout with vmap

    jax.vmap vectorizes reset and step over a batch of environments.
    """)
    return


@app.cell
def _(env, jax, n_steps_slider, np):
    _batch_size = 8

    _vreset = jax.vmap(env.reset)
    _vstep  = jax.vmap(env.step)

    env.set_target(np.array([0.2, 0.1, 0.3], dtype=np.float32))

    _rngs   = jax.random.split(jax.random.PRNGKey(0), _batch_size)
    _states = _vreset(_rngs)

    print(f"batch obs shape  : {_states.obs.shape}")
    print(f"batch reward shape: {_states.reward.shape}")

    for _step in range(n_steps_slider.value):
        _actions = jax.random.uniform(
            jax.random.fold_in(jax.random.PRNGKey(1), _step),
            (_batch_size, env.action_size),
            minval=-1.0,
            maxval=1.0,
        )
        _states = _vstep(_states, _actions)
        if bool(jax.numpy.all(_states.done)):
            break

    print(f"final mean reward: {float(_states.reward.mean()):.4f}")
    return


@app.cell(hide_code=True)
def _(mo):
    mo.md(r"""
    ## Render a trajectory

    Replay the single rollout through env.render and stop when done.
    """)
    return


@app.cell
def _(env, jax, mo, n_steps_slider, np):
    env.set_target(np.array([0.2, 0.1, 0.3], dtype=np.float32))
    _rng = jax.random.PRNGKey(99)
    _state = env.reset(_rng)

    _trajectory = [_state]
    for _step in range(n_steps_slider.value):
        _action = jax.random.uniform(
            jax.random.fold_in(_rng, _step),
            (env.action_size,),
            minval=-1.0,
            maxval=1.0,
        )
        _state = env.step(_state, _action)
        _trajectory.append(_state)
        if bool(_state.done):
            break

    frames = env.render(_trajectory, height=240, width=320)
    print(f"rendered {len(frames)} frames, shape {frames[0].shape}")

    mo.image(frames[0], width=320)
    return


if __name__ == "__main__":
    app.run()

import marimo

__generated_with = "0.20.4"
app = marimo.App(width="medium")


@app.cell
def _():
    import marimo as mo

    return (mo,)


@app.cell
def _(mo):
    mo.md(r"""
    # Robot Arm Gymnasium Environment

    ## Part 1 — Register & use with `gym.make()`

    Build a robot arm spec, register it as a Gymnasium environment, and run a
    rollout using the standard `gym.make()` API.
    """)
    return


@app.cell
def _(mo):
    ndof_slider = mo.ui.slider(1, 6, value=3, label="ndof (number of joints)")
    control_radio = mo.ui.radio(["pos", "torque"], value="pos", label="control")
    mo.hstack([ndof_slider, control_radio])
    return control_radio, ndof_slider


@app.cell
def _(control_radio, ndof_slider):
    from robotdesigner.mujoco.composer import build_arm

    spec = build_arm(ndof=ndof_slider.value, control=control_radio.value)
    print(f"model : {spec.modelname}")
    print(f"ndof  : {ndof_slider.value}")
    print(f"control: {control_radio.value}")
    return (spec,)


@app.cell
def _(spec):
    spec.compile()
    return


@app.cell
def _():
    import gymnasium as gym

    # Register once — guard prevents errors on cell re-run.
    # gym passes kwargs from gym.make() into AbstractMujocoEnv.__init__ as **overrides.
    # if "AbstractArm-v0" not in gym.envs.registry:
    gym.register(
            id="AbstractArm-v0",
            entry_point="robotdesigner.envs.abstract_mujoco_env:make_env",
        )
    return (gym,)


@app.cell
def _(gym, spec):
    # Convert MjSpec → XML string so the config is serializable.
    # gym.make forwards extra kwargs directly to AbstractMujocoEnv.__init__.
    env = gym.make(
        "AbstractArm-v0",
        spec=spec,
        render_mode="rgb_array",
    )
    print(f"Action space     : {env.action_space}")
    print(f"Observation space: {env.observation_space}")
    return (env,)


@app.cell
def _(env):
    obs, _ = env.reset()
    print("obs shape :", obs.shape)
    print("obs       :", obs)
    return


@app.cell
def _(env):
    import pandas as pd

    rows = []
    _obs, _ = env.reset()
    for _step in range(50):
        _action = env.action_space.sample()
        _obs, _reward, _terminated, _truncated, _ = env.step(_action)
        rows.append({
            "step": _step,
            "reward": round(float(_reward), 4),
            "ee_x": round(float(_obs[0]), 3),
            "ee_y": round(float(_obs[1]), 3),
            "ee_z": round(float(_obs[2]), 3),
        })
        if _terminated or _truncated:
            break

    df = pd.DataFrame(rows)
    df
    return (df,)


@app.cell
def _(df, mo):
    mo.ui.table(df)
    return


@app.cell
def _(mo):
    mo.md(r"""
    ---

    ## Part 2 — Train with Ray RLlib (new API)

    The same `"AbstractArm-v0"` registration works directly with RLlib.
    Pass `xml` as a string in `env_config` — this keeps the config fully
    picklable so RLlib can ship it to remote workers.
    """)
    return


@app.cell
def _(spec):
    try:
        import ray
        from ray.rllib.algorithms.ppo import PPOConfig

        ray.init(ignore_reinit_error=True)

        algo = (
            PPOConfig()
            .environment(
                env="AbstractArm-v0",
                env_config={
                    "xml": spec.to_xml(),
                    "render_mode": "rgb_array",
                },
            )
            .env_runners(num_env_runners=1)
            .build()
        )

        result = algo.train()
        print(f"episode_reward_mean : {result['env_runners']['episode_reward_mean']:.3f}")
        print(f"episode_len_mean    : {result['env_runners']['episode_len_mean']:.1f}")

        algo.stop()
        ray.shutdown()

    except ImportError:
        print("ray[rllib] not installed — skipping RLlib example.")
        print("Install with: pip install 'ray[rllib]'")
    return


if __name__ == "__main__":
    app.run()

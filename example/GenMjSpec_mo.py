import marimo

__generated_with = "0.20.4"
app = marimo.App()


@app.cell
def _():
    from robotdesigner.mujoco import composer
    import mujoco
    import mujoco.viewer

    return composer, mujoco


@app.cell
def _(composer):
    mujoco_spec = composer.build_arm(ndof=3, control="pos", joint_orientation=["z", "y", "y"])
    return (mujoco_spec,)


@app.cell
def _(mujoco, mujoco_spec):
    model = mujoco_spec.compile()
    data = mujoco.MjData(model)

    return data, model


@app.cell
def _(data, model, mujoco):
    mujoco.viewer.launch(model, data)
    return


if __name__ == "__main__":
    app.run()

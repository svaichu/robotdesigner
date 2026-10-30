Future Ideas, DONT CONSIDER THIS AS A ROADMAP, JUST IDEAS

1. Connection to FreeCAD.
2. Mjx-Jax and differentiability.
3. Standardize interface with Mujoco-Warp using Newton.
4. TorchRL's own render stack (`torchrl.render` / `rlrender` CLI) as an optional adapter, mainly to render trained policies from checkpoints (formats mp4, gif, ipynb, frames, npz; backends env / pixels / mujoco). It would now find `env.render()`, and `MujocoStateReader` could read the MuJoCo state. Its MP4 writer needs `torchcodec>=0.10` plus FFmpeg libraries, which are not in the robotdesigner venv. Only the source was read, nothing was run. Not adopted for now: the in-repo `robotdesigner.viz` covers recording and saving without a checkpoint or policy factory.
5. Add molvospaces env as another backend for another library.

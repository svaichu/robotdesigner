# pizza_robot: agent memory

Standalone MuJoCo scene: an ElRobot arm on a table making a pizza. It is not wired into the `robotdesigner` package (no MjSpec composer, no gymnasium env).

## Files
- `elrobot_table.xml`: the whole scene. Meshes load from `assets/elrobot/`, so the XML must stay next to `assets/`.
- `run_elrobot_table.py`: opens the MuJoCo viewer (`python pizza_robot/run_elrobot_table.py` from the repo root, using `.venv`).
- `assets/elrobot/`: STL meshes copied from `norma-core/hardware/elrobot/simulation/assets/`.

## History
1. Started with the SO101 follower (URDF from `norma-core/software/station/clients/station-viewer/public/devices/so101/`).
2. Replaced it with ElRobot (URDF `norma-core/hardware/elrobot/simulation/elrobot_follower.urdf`). All SO101 files were deleted.

## Scene layout (table top at z=0.75, table 1.2 x 0.8 m)
- Robot base on the front table edge at (0, -0.36). At zero pose the arm stands up, then reaches forward (+y).
- `pizza_plate`: disc, r=0.11, near (0, -0.15).
- `bin_salami` (right, x=+0.22): 18 flat ellipsoid discs (`bin_salami_disc*`, 4 cm wide, 8 mm thick).
- `bin_cheese` (left, x=-0.22): shredded cheese, modelled in three layers (see below).
- `camera_stand` at the back edge with camera `overhead` (targetbody `pizza_plate`).
- Camera `wrist` on `Gripper_Base_v1_1`, looking along the gripper.

## ElRobot model
- Joints are named `joint1`..`joint7` and `gripper`. Each has a position actuator (kp=10, kv=0.5) with the URDF ctrlrange.
- The jaws `jaw_left` and `jaw_right` are slide joints tied to `gripper` by `<equality><joint>` with polycoef ±0.0115. The URDF mimic tag is ignored by MuJoCo, so this replaces it.
- The URDF 2.94 Nm effort limit was dropped so the arm can hold itself up in simulation.
- Only the gripper bodies collide. All other arm geoms have contype/conaffinity 0.
- Zero-inertia links needed `1e-7` inertia to compile.
- Timestep is 1 ms. Default joint armature is 0.01 and damping 0.3.

## Cheese bin (compute vs realism)
Measured single-threaded, no rendering: no cheese ~10x real time, 120 real shreds ~1.7x, current setup ~4x.
1. A solid collision box "bed" (13.2 cm square, 1.2 cm thick) fills the bottom of the bin.
2. 150 decorative capsules (contype=0, conaffinity=0) sit on the bed. They only render.
3. 50 real free-body capsule shreds are poured on top. They were settled in a temporary closed copy of the scene, and the settled poses were written into the XML.
Limit: the gripper passes through the decoration and can only pick up the 50 real shreds.

## Lessons learned (do not repeat)
- `<composite type="particle">` is deprecated in MuJoCo 3.6. Use `<replicate>`, but it only makes regular grids with a fixed rotation step, so I generated the bodies by hand.
- Flat cylinders (r=2 cm, h=6 mm) fell through thin box floors. Flat ellipsoids work, and so do thick floors.
- Thin light capsules blew up at dt=2 ms. Use dt=1 ms with armature 1e-4 and damping 1e-3 on their free joints.
- Joint defaults apply to free joints too. The default `frictionloss=0.02` froze the tiny shreds in mid-air, so set `frictionloss="0"` on them.
- `qpos[:6]` is not the arm. Free-body objects declared first take the first qpos slots, so look joints up by name (`model.joint(name).qposadr`).
- `pkill -f run_elrobot_table` also kills the shell that runs it. Use `pgrep` then `kill`, in a separate command from the relaunch.
- Offscreen `mujoco.Renderer` is limited to 640x480 unless `<visual><global offwidth offheight/>` is set.

## Open items
- Arm starts at its zero pose, with the gripper over the plate. A folded home pose has not been set.
- No joint-limit enforcement beyond actuator ctrlrange (URDF limits were imported as ranges, and `autolimits` is on, so check that before relying on them).
- Nothing is registered as a gymnasium env, and there are no tests.

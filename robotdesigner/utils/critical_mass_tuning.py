import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("assets/one_arm_pos.xml")
data = mujoco.MjData(model)

# Set to nominal/home pose
mujoco.mj_resetData(model, data)
mujoco.mj_forward(model, data)

# Read full mass matrix
M = np.zeros((model.nv, model.nv))
mujoco.mj_fullM(model, M, data.qM)

# Diagonal = effective inertia per joint
M_diag = np.diag(M)

# Choose desired bandwidth (rad/s) — tune this
omega_n = 20.0  # e.g. 20 rad/s ≈ 3 Hz bandwidth

kp = omega_n**2 * M_diag
kv = 2 * np.sqrt(kp * M_diag)  # critically damped, ignores joint damping

# Subtract any existing joint damping to avoid double-counting
joint_damping = model.dof_damping[:]
kv_net = np.maximum(kv - joint_damping, 0)

print("kp per joint:", kp)
print("kv per joint:", kv_net)
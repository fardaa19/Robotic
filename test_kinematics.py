import numpy as np
from kr210_kinematics import forward_kinematics, inverse_kinematics, rpy_to_rotation

np.set_printoptions(precision=4, suppress=True)


def rotation_to_rpy(R):
    """Inverse of rpy_to_rotation (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    return roll, pitch, yaw


# KR210 joint limits (radians), from kr210.urdf.xacro
LIMITS = [
    (-np.deg2rad(185), np.deg2rad(185)),
    (-np.deg2rad(45),  np.deg2rad(85)),
    (-np.deg2rad(210), np.deg2rad(65-90)),  # approx, joint3 offset in urdf
    (-np.deg2rad(350), np.deg2rad(350)),
    (-np.deg2rad(125), np.deg2rad(125)),
    (-np.deg2rad(350), np.deg2rad(350)),
]

rng = np.random.default_rng(42)

n_tests = 200
pos_errs = []
rot_errs = []
worst = None

for trial in range(n_tests):
    q_true = np.array([rng.uniform(lo, hi) for (lo, hi) in LIMITS])

    pos, R_ee, _ = forward_kinematics(q_true)
    roll, pitch, yaw = rotation_to_rpy(R_ee)

    q_ik = inverse_kinematics(pos[0], pos[1], pos[2], roll, pitch, yaw)

    pos_check, R_check, _ = forward_kinematics(q_ik)

    pos_err = np.linalg.norm(pos_check - pos)
    rot_err = np.linalg.norm(R_check - R_ee)

    pos_errs.append(pos_err)
    rot_errs.append(rot_err)

    if worst is None or pos_err > worst[0]:
        worst = (pos_err, rot_err, q_true, q_ik)

pos_errs = np.array(pos_errs)
rot_errs = np.array(rot_errs)

print(f"Ran {n_tests} random FK -> IK -> FK round-trip tests")
print(f"Position error   : mean={pos_errs.mean():.3e} m   max={pos_errs.max():.3e} m")
print(f"Orientation error: mean={rot_errs.mean():.3e}     max={rot_errs.max():.3e}")
print()
print("Worst case:")
print(f"  pos_err={worst[0]:.3e} m, rot_err={worst[1]:.3e}")
print(f"  q_true = {worst[2]}")
print(f"  q_ik   = {worst[3]}")

assert pos_errs.max() < 1e-6, "Position round-trip error too large!"
assert rot_errs.max() < 1e-6, "Orientation round-trip error too large!"
print("\nAll round-trip tests PASSED (sub-micron position, sub-1e-6 rotation error).")

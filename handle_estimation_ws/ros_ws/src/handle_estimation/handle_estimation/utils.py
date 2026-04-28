import numpy as np


MIN_EIGENVALUE_RATIO = 3.0


def compute_stable_orientation(points, prev_axes=None, min_ratio=3.0):
    if len(points) < 20:
        return np.eye(3), np.array([0.0, 0.0, 0.0])

    centroid = np.mean(points, axis=0)
    centered = points - centroid

    U, S, Vt = np.linalg.svd(centered, full_matrices=False)

    axes = Vt

    if len(S) >= 3 and S[2] > 1e-10:
        ratio = S[0] / S[2] if S[2] > 1e-10 else 0
    else:
        ratio = 0

    if ratio < min_ratio:
        return np.eye(3), np.array([0.0, 0.0, 0.0])

    det = np.linalg.det(axes)
    if det < 0:
        axes = -axes

    if prev_axes is not None:
        for i in range(3):
            dot = np.dot(axes[i], prev_axes[i])
            if dot < 0:
                axes[i] = -axes[i]

    return axes, centroid


def smooth_orientation(current_axes, prev_axes, alpha=0.15):
    if prev_axes is None:
        return current_axes

    smoothed = np.zeros_like(current_axes)
    for i in range(3):
        smoothed[i] = alpha * current_axes[i] + (1 - alpha) * prev_axes[i]

    x = smoothed[0]
    y = smoothed[1] - np.dot(smoothed[1], x) / (np.dot(x, x) + 1e-10) * x
    y = y / (np.linalg.norm(y) + 1e-10)
    z = np.cross(x, y)
    z = z / (np.linalg.norm(z) + 1e-10)

    return np.array([x, y, z])


def axes_to_euler(axes):
    rx = axes[0]
    ry = axes[1]
    rz = axes[2]

    roll = np.arctan2(rx[1], rx[0])
    pitch = np.arctan2(-rx[2], np.sqrt(rx[0]**2 + rx[1]**2))
    yaw = np.arctan2(ry[0], rz[0])

    return roll, pitch, yaw


def apply_ema_smoothing(pose_history, new_pose, alpha=0.3):
    if not pose_history:
        return new_pose

    smoothed = []
    for old, new in zip(pose_history[-1], new_pose):
        smoothed.append(alpha * new + (1 - alpha) * old)

    return smoothed
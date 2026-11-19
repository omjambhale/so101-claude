"""SO-101 forward/inverse kinematics from the official URDF (TheRobotStudio
SO-ARM100, Simulation/SO101/so101_new_calib.urdf), numpy only.

Joint angles are LeRobot degrees (use_degrees=True, "new calibration": zero at
the middle of each joint's range), the same convention LeRobot's own
RobotKinematics uses with this URDF. Positions are metres in the arm's base
frame: +x forward, +y left, +z up.
"""
import xml.etree.ElementTree as ET

import numpy as np

CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper_frame_joint"]
DOWN = np.array([0.0, 0.0, -1.0])


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, t):
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(t) * k + (1 - np.cos(t)) * k @ k


class SO101Kinematics:
    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        for j in root.findall("joint"):
            o, a = j.find("origin"), j.find("axis")
            fixed = np.eye(4)
            fixed[:3, :3] = _rpy(*[float(v) for v in o.get("rpy").split()])
            fixed[:3, 3] = [float(v) for v in o.get("xyz").split()]
            axis = np.array([float(v) for v in a.get("xyz").split()]) if a is not None else None
            self.joints[j.get("name")] = (fixed, axis if axis is not None and np.linalg.norm(axis) > 0 else None)
        missing = [n for n in CHAIN if n not in self.joints]
        if missing:
            raise ValueError(f"URDF is missing joints {missing}")

    def frames(self, q_deg):
        """q_deg = [pan, lift, elbow, wrist_flex, wrist_roll]. Returns {joint: 4x4 world transform after it}."""
        out, t, i = {}, np.eye(4), 0
        for name in CHAIN:
            fixed, axis = self.joints[name]
            t = t @ fixed
            if axis is not None:
                rot = np.eye(4)
                rot[:3, :3] = _axis_angle(axis, np.deg2rad(q_deg[i]))
                t = t @ rot
                i += 1
            out[name] = t
        return out

    def tcp(self, q_deg):
        """Tool point between the gripper jaws, metres."""
        return self.frames(q_deg)["gripper_frame_joint"][:3, 3]

    def approach_dir(self, q_deg):
        """Unit vector from the wrist-roll axis to the tool point (where the gripper points)."""
        f = self.frames(q_deg)
        v = f["gripper_frame_joint"][:3, 3] - f["wrist_roll"][:3, 3]
        return v / np.linalg.norm(v)

    def jaw_axis_xy(self, q_deg):
        """Direction the jaws close along (gripper_link x-axis), projected on the table, unit 2-vector."""
        x = self.frames(q_deg)["wrist_roll"][:3, 0]
        v = x[:2]
        n = np.linalg.norm(v)
        return v / n if n > 1e-6 else np.array([1.0, 0.0])

    def ik(self, target_xyz, q_seed, roll_deg, limits, point_down=True, iters=200, warm_only=False):
        """Solve [pan, lift, elbow, wrist_flex] for a tool position (and gripper pointing down),
        with wrist_roll fixed. `limits` = list of 5 (lo, hi) in degrees. Returns (q, pos_err_m, angle_err_deg).
        warm_only=True uses just q_seed (fast; for small steps along a path)."""
        target = np.asarray(target_xyz, dtype=float)
        best = None
        seeds = [np.array(q_seed[:4], dtype=float)]
        if not warm_only:
            pan0 = -np.degrees(np.arctan2(target[1], target[0]))  # +pan turns toward -y
            for lift, elbow, flex in [(0, 0, 90), (20, -20, 80), (-20, 30, 70), (40, -50, 90), (10, 20, 60)]:
                seeds.append(np.array([pan0, lift, elbow, flex]))
        w_ori = 0.05 if point_down else 0.0
        for seed in seeds:
            q = np.clip(seed, [l[0] for l in limits[:4]], [l[1] for l in limits[:4]])
            for _ in range(iters):
                r = self._residual(q, roll_deg, target, w_ori)
                if np.linalg.norm(r) < 2e-4:
                    break
                jac = np.zeros((len(r), 4))
                for k in range(4):
                    dq = np.zeros(4)
                    dq[k] = 0.05
                    jac[:, k] = (self._residual(q + dq, roll_deg, target, w_ori) - r) / np.radians(0.05)
                lam = 0.01
                step = jac.T @ np.linalg.solve(jac @ jac.T + lam ** 2 * np.eye(len(r)), -r)
                step = np.clip(np.degrees(step), -10, 10)
                q = np.clip(q + step, [l[0] for l in limits[:4]], [l[1] for l in limits[:4]])
            full = np.append(q, roll_deg)
            pos_err = float(np.linalg.norm(self.tcp(full) - target))
            ang_err = float(np.degrees(np.arccos(np.clip(self.approach_dir(full) @ DOWN, -1, 1)))) if point_down else 0.0
            score = pos_err + (0.002 * ang_err if point_down else 0)
            if best is None or score < best[3]:
                best = (full, pos_err, ang_err, score)
            if pos_err < 0.002 and ang_err < 3:
                break  # good enough, skip remaining seeds
        return best[0], best[1], best[2]

    def _residual(self, q4, roll_deg, target, w_ori):
        full = np.append(q4, roll_deg)
        res = self.tcp(full) - target
        if w_ori > 0:
            res = np.concatenate([res, w_ori * (self.approach_dir(full) - DOWN)])
        return res

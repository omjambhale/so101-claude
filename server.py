"""MCP server that lets Claude see through two cameras and move a LeRobot SO-101 arm.

Claude looks at the front (overhead) camera, reads an object's pixel, and calls pick(x, y) / place(x, y).
This file turns pixels into table positions (via a 4-point table calibration) and table positions into
safe, smooth joint moves (via the official SO-101 URDF).

Run by Claude Desktop (see README). Frames come from cameras.py, which must be running.
Never print() here: stdout is the MCP channel. Logs go to stderr.
"""
import json
import logging
import os
import sys
import threading
import time

import cv2
import numpy as np
from mcp.types import ToolAnnotations

try:                                   # mcp >= 2
    from mcp.server.mcpserver import Image, MCPServer
except ImportError:                    # mcp 1.x
    from mcp.server.fastmcp import FastMCP as MCPServer, Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config as _config  # noqa: E402
from kinematics import SO101Kinematics  # noqa: E402

logging.basicConfig(stream=sys.stderr, level=logging.INFO)
log = logging.getLogger("so101-claude")

CFG = _config.load()
FRAMES_DIR = CFG["frames_dir"]
CALIB_FILE = CFG["table_calibration_file"]
ROLL_OFFSET = float(CFG.get("wrist_roll_offset_deg") or 0.0)
URDF = os.path.join(HERE, "assets", "so101_new_calib.urdf")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM = JOINTS[:5]
LIMIT_MARGIN_DEG = 3.0
JOINT_SPEED_DEG_S = 45.0
CART_SPEED_M_S = 0.06
MIN_MOVE_S = 0.8
CONTROL_HZ = 30
FRAME_MAX_AGE_S = 3.0
FLOOR_MARGIN_M = 0.001       # tool point never goes below the table + this
HOVER_M = 0.06               # approach height above the table
PLACE_HOVER_M = 0.035        # lower carry height when placing (reaches farther)
OPEN_PCT = 60.0
GRASP_THRESHOLD = 2.0        # gripper % after closing above which something is held
HOLD_SQUEEZE_PCT = 4.0       # after a grab, hold this little past the contact point (no full-force squeeze)
MAX_TILT_DEG = 30.0          # gripper may lean this much off vertical to reach farther
MARKER_COLORS = (("blue", (90, 120, 150), (112, 255, 255)), ("yellow", (20, 100, 150), (38, 255, 255)))
JAW_OFFSET_REL_CM = (0.05, -2.4)   # where a centred object sits relative to the tool point (gripper frame)
TEMPLATE_HALF = 45

mcp = MCPServer(
    "so101",
    instructions=(
        "Controls a real LeRobot SO-101 arm with a front (overhead) camera and a wrist camera.\n"
        "FIRST: call get_state. If mat_calibrated is false, guide the user through table calibration: they put "
        "4 small blue or yellow tape marks on the table, spread out within the arm's reach; call "
        "calibration_start (ALL motors go limp: tell the user to hold the arm BEFORE calling it); for each mark "
        "the user touches the gripper tip to it, pointing straight down, and you call calibration_add_point "
        "with that mark's pixel in look('front', grid=True); then calibration_finish.\n"
        "TO MOVE AN OBJECT: look('front', grid=True) (zoom_at to refine) and read the object's centre pixel and "
        "its long-axis angle; pick(x, y, angle_deg); check 'held' (and look('wrist')); place(x, y); go_home().\n"
        "Work one tool call at a time and check each result before the next move. If picks land off, the "
        "front camera probably moved: redo the table calibration. Keep the user informed before big moves."
    ),
)


def _ann(read_only, destructive):
    """ToolAnnotations across mcp versions (snake_case fields in mcp 2, camelCase in mcp 1)."""
    if "read_only_hint" in getattr(ToolAnnotations, "model_fields", {}):
        return ToolAnnotations(read_only_hint=read_only, destructive_hint=destructive)
    return ToolAnnotations(readOnlyHint=read_only, destructiveHint=destructive)


MOVE = _ann(False, True)
READ = _ann(True, False)
SETTING = _ann(False, False)


class _Busy:
    """One move at a time; never hang forever if something got stuck."""
    _l = threading.Lock()

    def __enter__(self):
        if not self._l.acquire(timeout=30):
            raise RuntimeError("Arm is busy with another command.")
        return self

    def __exit__(self, *exc):
        self._l.release()


_lock = _Busy()
_robot = None
_home = None
_limits = None
_table = None
_calib_points = None
_H_now = None
_kin = SO101Kinematics(URDF)


# ---------------------------------------------------------------- connection
def _load_table():
    if not os.path.exists(CALIB_FILE):
        return None
    with open(CALIB_FILE) as f:
        t = json.load(f)
    t["H"] = np.array(t["H"])
    return t


def _goals_to_present(port):
    """Set every motor's goal to where it is now BEFORE LeRobot connects. LeRobot switches torque on during
    connect without doing this, so an arm that was moved by hand while limp would otherwise snap back."""
    try:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        bus = FeetechMotorsBus(port=port, motors={j: Motor(i + 1, "sts3215", MotorNormMode.DEGREES)
                                                  for i, j in enumerate(JOINTS)})
        bus.connect()
        try:
            pos = bus.sync_read("Present_Position", normalize=False)
            bus.sync_write("Goal_Position", pos, normalize=False)
        finally:
            bus.port_handler.closePort()
    except Exception as e:                       # never block startup on this; LeRobot's connect reports real errors
        log.warning("could not pre-sync goals: %s", e)


def _get_robot():
    global _robot, _home, _limits, _table
    if _robot is not None:
        return _robot
    if not CFG.get("follower_port"):
        raise RuntimeError("No follower_port in config.json. Run setup_claude.py first.")
    from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
    from lerobot.robots.utils import make_robot_from_config
    _goals_to_present(CFG["follower_port"])
    robot = make_robot_from_config(SOFollowerRobotConfig(port=CFG["follower_port"], id=CFG["robot_id"],
                                                         cameras={}, disable_torque_on_disconnect=False))
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        robot.disconnect()
        raise RuntimeError(f"Arm is not calibrated for id '{CFG['robot_id']}'. Run lerobot-calibrate first.")
    _robot = robot
    _limits = {}
    for n in ARM:
        c = _robot.calibration[n]
        half = (c.range_max - c.range_min) / 2 * 360 / 4095
        _limits[n] = (-half + LIMIT_MARGIN_DEG, half - LIMIT_MARGIN_DEG)
    lo, hi = _limits["wrist_roll"]
    _limits["wrist_roll"] = (lo + ROLL_OFFSET, hi + ROLL_OFFSET)
    _limits["gripper"] = (0.0, 100.0)
    _table = _load_table()
    _home = dict(CFG["home"]) if CFG.get("home") else _read_pose()
    return _robot


def _read_pose():
    """Joint angles in URDF degrees (LeRobot degrees, wrist_roll shifted by the configured offset)."""
    obs = _robot.get_observation()
    pose = {j: float(obs[f"{j}.pos"]) for j in JOINTS}
    pose["wrist_roll"] += ROLL_OFFSET
    return pose


def _send(pose):
    act = {f"{j}.pos": float(pose[j]) for j in JOINTS}
    act["wrist_roll.pos"] -= ROLL_OFFSET
    _robot.send_action(act)


def _arm_vec(pose):
    return np.array([pose[j] for j in ARM])


def _ik_limits():
    return [_limits[n] for n in ARM]


def _clamp(j, v):
    lo, hi = _limits[j]
    return min(max(v, lo), hi)


def _hold_pct(g):
    """Gripper command that keeps a light grip on what it closed on, instead of driving to 0 at full force."""
    return max(0.0, float(g) - HOLD_SQUEEZE_PCT) if g > GRASP_THRESHOLD else 0.0


# ---------------------------------------------------------------- table geometry
def _mat_z(xy):
    """Table height under (x, y): a plane through the calibration touches (tables are rarely level to the arm)."""
    pts = (_table or {}).get("points") or []
    if len(pts) >= 3:
        P = np.array([p["xyz"] for p in pts], float)
        A = np.column_stack([np.ones(len(P)), P[:, 0], P[:, 1]])
        a, b, c = np.linalg.lstsq(A, P[:, 2], rcond=None)[0]
        return float(a + b * xy[0] + c * xy[1])
    return float(_table["table_z"]) if _table else -1.0


def _detect_markers(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    found = []
    for color, lo, hi in MARKER_COLORS:
        mask = cv2.morphologyEx(cv2.inRange(hsv, lo, hi), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            if 150 < cv2.contourArea(c) < 30000:
                m = cv2.moments(c)
                found.append((color, m["m10"] / m["m00"], m["m01"] / m["m00"]))
    return found


def _refresh_mapping():
    """If the front camera shifted a little, re-anchor on the 4 tape marks (found near their saved spots)."""
    global _H_now
    _H_now = None
    ref = (_table or {}).get("markers_px")
    img = cv2.imread(os.path.join(FRAMES_DIR, "front.jpg"))
    if not ref or img is None:
        return {"mode": "fixed calibration"}
    cur = _detect_markers(img)
    pairs = []
    for color, rx, ry in ref:
        near = [(np.hypot(cx - rx, cy - ry), cx, cy) for c, cx, cy in cur if c == color]
        near = [n for n in near if n[0] < 40]
        if near:
            _, cx, cy = min(near)
            pairs.append(((rx, ry), (cx, cy)))
    if len(pairs) < 4:
        return {"mode": "fixed calibration", "marks_seen": len(pairs)}
    src = np.float32([p[1] for p in pairs]).reshape(-1, 1, 2)
    dst = np.float32([p[0] for p in pairs]).reshape(-1, 1, 2)
    T, _ = cv2.findHomography(src, dst, 0)
    shift = float(np.max(np.linalg.norm(src.reshape(-1, 2) - dst.reshape(-1, 2), axis=1)))
    if T is not None and shift > 1.5:
        _H_now = _table["H"] @ T
    return {"mode": "checked against marks", "view_shift_px": round(shift, 1)}


def _pixel_to_xy(u, v):
    p = (_H_now if _H_now is not None else _table["H"]) @ np.array([u, v, 1.0])
    return p[:2] / p[2]


# ---------------------------------------------------------------- motion
def _below_floor(arm_deg, start):
    if _table is None:
        return False
    f = _kin.frames(arm_deg)
    tip, wrist = f["gripper_frame_joint"][:3, 3], f["wrist_flex"][2, 3]
    floor = _mat_z(tip[:2]) + FLOOR_MARGIN_M
    return (tip[2] < floor and tip[2] < start[0] - 0.002) or (wrist < floor + 0.01 and wrist < start[1] - 0.002)


def _heights(arm_deg):
    f = _kin.frames(arm_deg)
    return f["gripper_frame_joint"][2, 3], f["wrist_flex"][2, 3]


def _send_path(waypoints, dt):
    if _calib_points is not None:
        raise RuntimeError("Table calibration is in progress (motors limp, user holding the arm). "
                           "Finish it with calibration_finish or abort with calibration_cancel first.")
    start = _heights(_arm_vec(_read_pose()))
    for wp in waypoints:
        if _below_floor(_arm_vec(wp), start):
            raise RuntimeError("Refused: this move would put the gripper below the table.")
    for wp in waypoints:
        _send(wp)
        time.sleep(dt)


def _joint_move(goal, duration_s=None, settle=True):
    """Smooth joint move, then nudge the command until the real joints sit on the goal
    (the servos stop 2-4 deg short under gravity otherwise)."""
    goal = dict(goal)
    for j in ARM + ["gripper"]:
        goal[j] = _clamp(j, goal[j])
    start = _read_pose()
    biggest = max(abs(goal[j] - start[j]) for j in ARM)
    duration_s = max(MIN_MOVE_S, duration_s or biggest / JOINT_SPEED_DEG_S)
    steps = max(1, int(duration_s * CONTROL_HZ))
    _send_path([{j: start[j] + (k / steps) * (goal[j] - start[j]) for j in JOINTS}
                for k in range(1, steps + 1)], 1.0 / CONTROL_HZ)
    time.sleep(0.3)
    actual = _read_pose()
    if settle:
        cmd = dict(goal)
        for _ in range(3):
            err = {j: goal[j] - actual[j] for j in ARM}
            if max(abs(v) for v in err.values()) < 0.5:
                break
            for j in ARM:
                cmd[j] = _clamp(j, goal[j] + max(-6.0, min(6.0, cmd[j] - goal[j] + err[j])))
            _send_path([cmd], 0.3)
            time.sleep(0.2)
            actual = _read_pose()
    return actual


def _solve(xyz, roll, seed, point_down=True, max_tilt_deg=MAX_TILT_DEG):
    q, pos_err, ang_err = _kin.ik(xyz, seed, roll, _ik_limits(), point_down=point_down)
    if pos_err > 0.006 or (point_down and ang_err > max_tilt_deg):
        raise RuntimeError(f"Can't reach ({xyz[0]*100:.1f}, {xyz[1]*100:.1f}, {xyz[2]*100:.1f}) cm pointing down "
                           f"(off {pos_err*100:.1f} cm, tilt {ang_err:.0f} deg). Too far: bring the object closer.")
    return q


def _cartesian_move(target, roll, gripper, point_down=True, max_tilt_deg=MAX_TILT_DEG):
    """Straight-line tool move (falls back to a joint move if the line passes through bad poses)."""
    pose = _read_pose()
    roll = _clamp("wrist_roll", roll)
    q = _arm_vec(pose)
    q[4] = roll
    start = _kin.tcp(q)
    target = np.asarray(target, float)
    final = _solve(target, roll, q, point_down, max_tilt_deg)
    n = max(2, int(np.linalg.norm(target - start) / 0.01) + 1)
    sols, seed = [], q
    try:
        for k in range(1, n + 1):
            p = start + (k / n) * (target - start)
            if k == n:
                sol = _solve(p, roll, seed, point_down, max_tilt_deg)
            else:
                sol, _, _ = _kin.ik(p, seed, roll, _ik_limits(), point_down=point_down, iters=40, warm_only=True)
            sols.append(sol)
            seed = sol
    except RuntimeError:
        sols = None
    if sols is None:
        goal = {j: float(final[i]) for i, j in enumerate(ARM)}
        goal["gripper"] = gripper
        return _joint_move(goal)
    duration = max(MIN_MOVE_S, float(np.linalg.norm(target - start)) / CART_SPEED_M_S)
    steps = max(len(sols), int(duration * CONTROL_HZ))
    g0, path = pose["gripper"], []
    for k in range(1, steps + 1):
        s = k / steps * len(sols)
        i = min(len(sols) - 1, int(np.ceil(s)) - 1)
        prev = q if i == 0 else sols[i - 1]
        arm = prev + min(1.0, s - i) * (sols[i] - prev)
        wp = {j: float(arm[ix]) for ix, j in enumerate(ARM)}
        wp["gripper"] = g0 + (k / steps) * (gripper - g0)
        path.append(wp)
    _send_path(path, duration / steps)
    return _joint_move(dict(path[-1]), 0.3)          # settle exactly on the end pose


def _roll_for_angle(xy, angle_deg, u, v):
    """wrist_roll so the jaws close ACROSS the object's long axis (angle in the front image)."""
    if angle_deg is None:
        want = xy / max(1e-6, np.linalg.norm(xy))
    else:
        a = np.radians(angle_deg)
        p1, p2 = _pixel_to_xy(u, v), _pixel_to_xy(u + 20 * np.cos(a), v + 20 * np.sin(a))
        axis = (p2 - p1) / max(1e-9, np.linalg.norm(p2 - p1))
        want = np.array([-axis[1], axis[0]])
    seed = _arm_vec(_read_pose())
    lo, hi = _limits["wrist_roll"]
    hover = [xy[0], xy[1], _mat_z(xy) + HOVER_M]
    best, best_err = None, 9.0
    for r in np.arange(np.ceil(lo / 15) * 15, hi, 15.0):
        q, pe, ae = _kin.ik(hover, seed, r, _ik_limits(), point_down=True, iters=60)
        if pe > 0.006 or ae > MAX_TILT_DEG:
            continue
        err = 1 - abs(float(_kin.jaw_axis_xy(q) @ want)) + 0.02 * abs(r - seed[4]) / 180
        if err < best_err:
            best, best_err = float(r), err
    return best if best is not None else _clamp("wrist_roll", 0.0)


def _report(pose=None):
    pose = pose or _read_pose()
    return {"pose": {j: round(v, 1) for j, v in pose.items()},
            "tool_point_cm": [round(float(v) * 100, 1) for v in _kin.tcp(_arm_vec(pose))]}


def _need_table():
    if _table is None:
        raise RuntimeError("Table not calibrated: run calibration_start first (see the server instructions).")


# ---------------------------------------------------------------- basic tools
@mcp.tool(annotations=READ)
def get_state() -> dict:
    """Joint angles, tool point (cm: +x forward, +y left, +z up), and whether the table is calibrated."""
    with _lock:
        _get_robot()
        out = _report()
        out["mat_calibrated"] = _table is not None
        out["camera"] = _refresh_mapping() if _table else None
        out["precision_mode_ready"] = bool(CFG.get("jaw_target_px"))
        c = _robot.calibration["wrist_roll"]
        if c.range_min <= 0 and c.range_max >= 4095:
            out["wrist_warning"] = ("wrist_roll has no position limits (0..4095). If the wrist has stoppers, ask the "
                                    "user to run tools/check_wrist_roll.py once (Claude Desktop must be quit).")
        return out


def _frame_path(camera):
    path = os.path.join(FRAMES_DIR, f"{camera}.jpg")
    if camera not in ("front", "wrist"):
        raise RuntimeError("camera must be 'front' or 'wrist'.")
    if not os.path.exists(path):
        raise RuntimeError("No camera frames. Start cameras.py in a Terminal (see README).")
    if time.time() - os.path.getmtime(path) > FRAME_MAX_AGE_S:
        raise RuntimeError("Camera frames are stale. Is cameras.py still running?")
    return path


@mcp.tool(annotations=READ)
def look(camera: str = "front", grid: bool = False, zoom_at: list[float] | None = None) -> Image:
    """Latest photo: "front" (overhead) or "wrist" (gripper). grid=True labels pixels; zoom_at=[x, y] = 3x close-up."""
    img = cv2.imread(_frame_path(camera))
    if img is None:
        raise RuntimeError("Could not read the frame; try again.")
    if zoom_at is not None:
        img = _zoom(img, float(zoom_at[0]), float(zoom_at[1]))
    elif grid:
        img = _grid(img, 0, 0, 1.0, 50)
    return Image(data=cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes(), format="jpeg")


def _grid(img, x0, y0, scale, every):
    out = img.copy()
    h, w = out.shape[:2]
    for gx in range(int(np.ceil(x0 / every) * every), int(x0 + w / scale) + 1, every):
        px = int((gx - x0) * scale)
        cv2.line(out, (px, 0), (px, h), (0, 255, 255), 1)
        cv2.putText(out, str(gx), (px + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    for gy in range(int(np.ceil(y0 / every) * every), int(y0 + h / scale) + 1, every):
        py = int((gy - y0) * scale)
        cv2.line(out, (0, py), (w, py), (0, 255, 255), 1)
        cv2.putText(out, str(gy), (2, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    return out


def _zoom(img, x, y, half=80, scale=3):
    h, w = img.shape[:2]
    x0, y0 = int(max(0, min(w - 2 * half, x - half))), int(max(0, min(h - 2 * half, y - half)))
    crop = cv2.resize(img[y0:y0 + 2 * half, x0:x0 + 2 * half], None, fx=scale, fy=scale,
                      interpolation=cv2.INTER_CUBIC)
    return _grid(crop, x0, y0, scale, 10)


@mcp.tool(annotations=MOVE)
def pick(x: float, y: float, angle_deg: float | None = None, grasp_height_cm: float = 0.8) -> dict:
    """Pick the object centred at pixel (x, y) of the FRONT image. angle_deg = its long axis in the image
    (0 = left-right, 90 = up-down; omit if round). grasp_height_cm = where the jaws close above the table."""
    with _lock:
        _get_robot()
        _need_table()
        view = _refresh_mapping()
        xy = _pixel_to_xy(x, y)
        z0 = _mat_z(xy)
        roll = _roll_for_angle(xy, angle_deg, x, y)
        hover = [xy[0], xy[1], z0 + HOVER_M]
        grasp = [xy[0], xy[1], z0 + max(0.3, grasp_height_cm) / 100]
        down = True
        try:
            q_hover = _solve(hover, roll, _arm_vec(_read_pose()))
            _solve(grasp, roll, q_hover)
        except RuntimeError:                                # far object: lean in instead of pointing down
            down = False
            q_hover = _solve(hover, roll, _arm_vec(_read_pose()), point_down=False)
            _solve(grasp, roll, q_hover, point_down=False)
        tcp = _kin.tcp(_arm_vec(_read_pose()))
        if tcp[2] < z0 + HOVER_M - 0.01:                     # lift first if we're low
            try:
                _cartesian_move([tcp[0], tcp[1], z0 + HOVER_M], _read_pose()["wrist_roll"], OPEN_PCT)
            except RuntimeError:
                pass
        goal = {j: float(q_hover[i]) for i, j in enumerate(ARM)}
        goal["gripper"] = OPEN_PCT
        _joint_move(goal)
        _cartesian_move(grasp, roll, OPEN_PCT, point_down=down)
        pose = _read_pose()
        pose["gripper"] = 0.0
        _joint_move(pose, 1.0, settle=False)
        time.sleep(0.4)
        g = _read_pose()["gripper"]
        _cartesian_move(hover, roll, _hold_pct(g), point_down=down)
        return {"target_cm": [round(float(xy[0]) * 100, 1), round(float(xy[1]) * 100, 1)],
                "held": bool(g > GRASP_THRESHOLD), "gripper_after_close": round(float(g), 1), "view": view}


@mcp.tool(annotations=MOVE)
def place(x: float, y: float, release_height_cm: float = 1.5, turn_deg: float = 0.0) -> dict:
    """Carry the held object over pixel (x, y) of the FRONT image, turn the wrist by turn_deg (to straighten
    it), lower to release_height_cm above the table, open, lift."""
    with _lock:
        _get_robot()
        _need_table()
        view = _refresh_mapping()
        xy = _pixel_to_xy(x, y)
        z0 = _mat_z(xy)
        pose = _read_pose()
        roll = _clamp("wrist_roll", pose["wrist_roll"] + turn_deg)
        g = _hold_pct(pose["gripper"])
        hover = [xy[0], xy[1], z0 + HOVER_M]
        q = _solve(hover, roll, _arm_vec(pose))
        goal = {j: float(q[i]) for i, j in enumerate(ARM)}
        goal["gripper"] = g
        _joint_move(goal)
        _cartesian_move([xy[0], xy[1], z0 + release_height_cm / 100], roll, g)
        p = _read_pose()
        p["gripper"] = OPEN_PCT
        _joint_move(p, 0.8, settle=False)
        _cartesian_move(hover, roll, OPEN_PCT)
        return {"placed_cm": [round(float(xy[0]) * 100, 1), round(float(xy[1]) * 100, 1)], "view": view}


@mcp.tool(annotations=MOVE)
def move_to(x_cm: float, y_cm: float, z_cm: float, point_down: bool = True) -> dict:
    """Move the tool point to (x, y, z) cm in the arm frame (+x forward, +y left, +z up), for small corrections."""
    with _lock:
        _get_robot()
        pose = _read_pose()
        _cartesian_move([x_cm / 100, y_cm / 100, z_cm / 100], pose["wrist_roll"], pose["gripper"],
                        point_down=point_down, max_tilt_deg=25)
        return _report()


@mcp.tool(annotations=MOVE)
def set_gripper(percent_open: float) -> dict:
    """Open or close the gripper: 0 = closed, 100 = fully open."""
    with _lock:
        _get_robot()
        goal = _read_pose()
        goal["gripper"] = float(percent_open)
        _joint_move(goal, 0.8, settle=False)
        return _report()


@mcp.tool(annotations=MOVE)
def turn_wrist(delta_deg: float) -> dict:
    """Rotate the gripper about its own axis by delta_deg (stays inside the wrist's safe range)."""
    with _lock:
        _get_robot()
        goal = _read_pose()
        goal["wrist_roll"] = goal["wrist_roll"] + float(delta_deg)
        _joint_move(goal, 1.2)
        return _report()


@mcp.tool(annotations=MOVE)
def go_home() -> dict:
    """Lift clear of the table, then return to the rest pose."""
    with _lock:
        _get_robot()
        pose = _read_pose()
        tcp = _kin.tcp(_arm_vec(pose))
        if _table is not None and tcp[2] < _mat_z(tcp[:2]) + HOVER_M:
            try:
                _cartesian_move([tcp[0], tcp[1], _mat_z(tcp[:2]) + HOVER_M], pose["wrist_roll"], pose["gripper"],
                                point_down=False)
            except RuntimeError:
                pass
        _joint_move(dict(_home), settle=False)
        return _report()


@mcp.tool(annotations=SETTING)
def set_home() -> dict:
    """Save the arm's CURRENT pose as the rest pose used by go_home (also after restarts)."""
    global _home
    with _lock:
        _get_robot()
        pose = _read_pose()
        _config.save_keys(home={j: round(v, 2) for j, v in pose.items()})   # URDF degrees
        _home = pose
        return {"home_saved": {j: round(v, 1) for j, v in pose.items()}}


# ---------------------------------------------------------------- table calibration
def _hold_where_it_is():
    pose = _read_pose()
    _send(pose)
    _robot.bus.enable_torque()


@mcp.tool(annotations=MOVE)
def calibration_start() -> dict:
    """ALL motors go limp: the user must be holding the arm before you call this. Then for each of the 4 tape
    marks the user touches the gripper tip to it, pointing straight down, and you call
    calibration_add_point(pixel of that mark in the FRONT image). Then calibration_finish."""
    global _calib_points
    with _lock:
        _get_robot()
        _calib_points = []
        _robot.bus.disable_torque()
        return {"status": "Motors OFF. The user is holding the arm."}


@mcp.tool(annotations=MOVE)
def calibration_add_point(x: float, y: float) -> dict:
    """Record one point: the gripper tip is touching the mark at FRONT-image pixel (x, y)."""
    with _lock:
        _get_robot()
        if _calib_points is None:
            raise RuntimeError("Call calibration_start first.")
        q = _arm_vec(_read_pose())
        p = _kin.tcp(q)
        tilt = float(np.degrees(np.arccos(np.clip(_kin.approach_dir(q) @ np.array([0, 0, -1.0]), -1, 1))))
        _calib_points.append({"px": [float(x), float(y)], "xyz": p.tolist(), "tilt_deg": tilt})
        return {"points": len(_calib_points), "tool_point_cm": [round(float(v) * 100, 1) for v in p],
                "warning": None if tilt < 20 else f"Tilted {tilt:.0f} deg: hold it more vertical and redo."}


@mcp.tool(annotations=MOVE)
def calibration_finish() -> dict:
    """Fit and save the camera-pixel -> table mapping, then motors back ON holding where the arm is."""
    global _calib_points, _table
    with _lock:
        _get_robot()
        try:
            good = [p for p in (_calib_points or []) if p["tilt_deg"] < 20]
            if len(good) < 4:
                raise RuntimeError("Need at least 4 points with the gripper pointing down.")
            src = np.array([p["px"] for p in good], np.float64)
            dst = np.array([p["xyz"][:2] for p in good], np.float64)
            H, _ = cv2.findHomography(src, dst, 0)
            if H is None:
                raise RuntimeError("Points are in a line; spread them out.")
            img = cv2.imread(os.path.join(FRAMES_DIR, "front.jpg"))
            table = {"H": H.tolist(), "table_z": float(np.mean([p["xyz"][2] for p in good])),
                     "points": good, "created": time.strftime("%Y-%m-%d %H:%M"),
                     "markers_px": [[c, float(u), float(v)] for c, u, v in _detect_markers(img)] if img is not None else []}
            with open(CALIB_FILE, "w") as f:
                json.dump(table, f, indent=1)
            _table = _load_table()
            return {"saved": os.path.basename(CALIB_FILE), "points_used": len(good),
                    "marks_saved": len(table["markers_px"])}
        finally:
            _hold_where_it_is()
            _calib_points = None


@mcp.tool(annotations=MOVE)
def calibration_cancel() -> dict:
    """Abort calibration; motors back ON holding where the arm is."""
    global _calib_points
    with _lock:
        _get_robot()
        _hold_where_it_is()
        _calib_points = None
        return {"status": "Motors ON."}


# ---------------------------------------------------------------- precision mode (wrist camera)
def _wrist_gray():
    path = _frame_path("wrist")
    t0 = time.time() + 0.5
    while os.path.getmtime(path) < t0:
        time.sleep(0.05)
        if time.time() > t0 + 4:
            raise RuntimeError("Wrist camera frames are stale.")
    time.sleep(0.05)
    img = cv2.imread(path)
    if img is None:
        time.sleep(0.2)
        img = cv2.imread(path)
    if img is None:
        raise RuntimeError("Could not read the wrist frame.")
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def _template(gray, px):
    """Patch around the object, kept off the gripper's own fixed jaw (which never moves in this camera and
    would otherwise fool the tracker) if wrist_jaw_region is configured."""
    h, w = gray.shape
    x, y = int(round(px[0])), int(round(px[1]))
    cx, cy = x, y
    jaw = CFG.get("wrist_jaw_region")
    if jaw:
        xm, ym = jaw["x_max"], jaw["y_min"]
        if cx - TEMPLATE_HALF < xm and cy + TEMPLATE_HALF > ym:
            cy = min(cy, ym - TEMPLATE_HALF)
            if cy < y - 60:                      # would lose the object: slide right instead
                cy, cx = y, max(x, xm + TEMPLATE_HALF)
    x0, y0 = max(0, cx - TEMPLATE_HALF), max(0, cy - TEMPLATE_HALF)
    x1, y1 = min(w, cx + TEMPLATE_HALF), min(h, cy + TEMPLATE_HALF)
    return gray[y0:y1, x0:x1].copy(), (x - x0, y - y0)


def _track(gray, tmpl, off):
    best = (-1.0, None)
    th, tw = tmpl.shape
    for ang in (0, -10, 10, -20, 20):
        t = tmpl if ang == 0 else cv2.warpAffine(tmpl, cv2.getRotationMatrix2D((off[0], off[1]), ang, 1.0), (tw, th),
                                                  borderMode=cv2.BORDER_REPLICATE)
        _, sc, _, loc = cv2.minMaxLoc(cv2.matchTemplate(gray, t, cv2.TM_CCOEFF_NORMED))
        if sc > best[0]:
            best = (float(sc), np.array([loc[0] + off[0], loc[1] + off[1]], float))
        if ang == 0 and sc > 0.85:
            break
    return best[1], best[0]


@mcp.tool(annotations=MOVE)
def calibrate_jaw_target() -> dict:
    """Precision-mode setup, once. Moves the gripper (open, pointing down) to hover over the middle of the
    calibrated table. Then the user slides a small flat object on the table until it sits exactly between
    the jaw tips (as seen from the side), you look('wrist') and call set_jaw_target(x, y) with its centre."""
    with _lock:
        _get_robot()
        _need_table()
        pts = np.array([p["xyz"][:2] for p in _table["points"]], float)
        xy = pts.mean(axis=0)
        roll = _clamp("wrist_roll", 0.0)
        q = _solve([xy[0], xy[1], _mat_z(xy) + HOVER_M], roll, _arm_vec(_read_pose()))
        goal = {j: float(q[i]) for i, j in enumerate(ARM)}
        goal["gripper"] = OPEN_PCT
        _joint_move(goal)
        return {"status": "Hovering. Ask the user to put an object right between the jaw tips, then "
                          "look('wrist') and call set_jaw_target(x, y).", "hover_cm": [round(float(v) * 100, 1) for v in xy]}


@mcp.tool(annotations=SETTING)
def set_jaw_target(x: float, y: float) -> dict:
    """Save the WRIST-image pixel where an object sits when it is exactly between the jaws at hover."""
    global CFG
    _config.save_keys(jaw_target_px=[float(x), float(y)])
    CFG = _config.load()
    return {"jaw_target_px": CFG["jaw_target_px"]}


@mcp.tool(annotations=MOVE)
def center_on(obj_px: float, obj_py: float) -> dict:
    """Precision mode. Call when hovering (jaws open, pointing down, ~6 cm above the table) with the object
    visible in look('wrist') at (obj_px, obj_py). Slides the arm until the object sits right between the
    jaws. Then call grab_here()."""
    target_cfg = CFG.get("jaw_target_px")
    if not target_cfg:
        raise RuntimeError("Precision mode not set up: run calibrate_jaw_target first.")
    with _lock:
        _get_robot()
        _need_table()
        pose = _read_pose()
        roll, g = pose["wrist_roll"], pose["gripper"]
        tcp = _kin.tcp(_arm_vec(pose))
        z = float(tcp[2])
        g0 = _wrist_gray()
        tmpl, off = _template(g0, (obj_px, obj_py))
        p0, _ = _track(g0, tmpl, off)
        # learn how 1 cm in x and in y moves the image here (it changes with arm angle)
        J = np.zeros((2, 2))
        for i in range(2):
            d = np.zeros(2)
            d[i] = 0.01
            _cartesian_move([tcp[0] + d[0], tcp[1] + d[1], z], roll, g)
            p, sc = _track(_wrist_gray(), tmpl, off)
            if sc < 0.5:
                _cartesian_move([tcp[0], tcp[1], z], roll, g)
                raise RuntimeError(f"Lost the object while learning (match {sc:.2f}).")
            J[:, i] = (p - p0)          # px per cm
            _cartesian_move([tcp[0], tcp[1], z], roll, g)
        if abs(np.linalg.det(J)) < 50:
            raise RuntimeError(f"Camera map unclear: {J.round(1).tolist()}")
        xy = tcp[:2].copy()
        p, it = p0, 0
        target = np.array(target_cfg, float)
        for it in range(1, 7):
            err = target - p
            if np.linalg.norm(err) < 8:
                break
            step = np.linalg.solve(J, err) / 100          # metres
            n = np.linalg.norm(step)
            if n > 0.03:
                step *= 0.03 / n
            xy = xy + step
            _cartesian_move([xy[0], xy[1], z], roll, g)
            p, sc = _track(_wrist_gray(), tmpl, off)
            if sc < 0.45:
                raise RuntimeError(f"Lost the object (match {sc:.2f}).")
        return {"object_px": [round(float(v)) for v in p], "target_px": [round(float(v)) for v in target],
                "error_px": round(float(np.linalg.norm(target - p)), 1), "moves": it,
                "tool_point_cm": [round(float(xy[0]) * 100, 2), round(float(xy[1]) * 100, 2)]}


@mcp.tool(annotations=MOVE)
def grab_here(grasp_height_cm: float = 0.7) -> dict:
    """Go straight down from the current hover, close the jaws, lift to hover. Reports whether it holds."""
    with _lock:
        _get_robot()
        _need_table()
        pose = _read_pose()
        tcp = _kin.tcp(_arm_vec(pose))
        roll = pose["wrist_roll"]
        z0 = _mat_z(tcp[:2])
        _cartesian_move([tcp[0], tcp[1], z0 + grasp_height_cm / 100], roll, pose["gripper"])
        p = _read_pose()
        p["gripper"] = 0.0
        _joint_move(p, 1.0, settle=False)
        time.sleep(0.4)
        gval = _read_pose()["gripper"]
        _cartesian_move([tcp[0], tcp[1], z0 + HOVER_M], roll, _hold_pct(gval))
        return {"held": bool(gval > GRASP_THRESHOLD), "gripper_after_close": round(float(gval), 1),
                "grabbed_at_cm": [round(float(tcp[0]) * 100, 2), round(float(tcp[1]) * 100, 2)]}


@mcp.tool(annotations=MOVE)
def put_down(slot_x_cm: float, slot_y_cm: float, turn_ccw_deg: float = 0.0) -> dict:
    """Precision mode. Place the held object (grabbed with center_on + grab_here) so its CENTRE lands on the
    slot (cm, arm frame), turned turn_ccw_deg counter-clockwise as seen from above."""
    with _lock:
        _get_robot()
        _need_table()
        pose = _read_pose()
        yaw_grab = -pose["shoulder_pan"] + pose["wrist_roll"]
        yaw_place = np.radians(yaw_grab + turn_ccw_deg)
        R = np.array([[np.cos(yaw_place), -np.sin(yaw_place)], [np.sin(yaw_place), np.cos(yaw_place)]])
        T = np.array([slot_x_cm, slot_y_cm]) / 100 - R @ (np.array(JAW_OFFSET_REL_CM) / 100)
        z0 = _mat_z(T)
        g = _hold_pct(pose["gripper"])
        q_guess = _kin.ik([T[0], T[1], z0 + PLACE_HOVER_M], _arm_vec(pose), pose["wrist_roll"], _ik_limits(),
                          point_down=True)[0]
        roll = float(np.degrees(yaw_place)) + q_guess[0]
        lo, hi = _limits["wrist_roll"]
        while roll > hi:
            roll -= 360
        while roll < lo:
            roll += 360
        if not lo <= roll <= hi:
            raise RuntimeError(f"Wrist can't reach roll {roll:.0f} deg; try a different turn_ccw_deg (e.g. +/-180).")
        tcp = _kin.tcp(_arm_vec(pose))
        _cartesian_move([tcp[0], tcp[1], max(tcp[2], z0 + PLACE_HOVER_M)], pose["wrist_roll"], g)
        goal = _read_pose()
        goal["wrist_roll"] = roll
        goal["gripper"] = g
        _joint_move(goal, 1.2)
        _cartesian_move([T[0], T[1], z0 + PLACE_HOVER_M], roll, g)
        _cartesian_move([T[0], T[1], z0 + 0.009], roll, g)
        p = _read_pose()
        p["gripper"] = 40.0
        _joint_move(p, 0.8, settle=False)
        _cartesian_move([T[0], T[1], z0 + PLACE_HOVER_M], roll, 40.0)
        pz = _read_pose()
        return {"jaws_at_cm": [round(float(T[0]) * 100, 2), round(float(T[1]) * 100, 2)],
                "yaw_check_deg": round(-pz["shoulder_pan"] + pz["wrist_roll"] - (yaw_grab + turn_ccw_deg), 1)}


# ---------------------------------------------------------------- health
@mcp.tool(annotations=READ)
def diagnose() -> dict:
    """READ-ONLY health check, moves nothing: per motor position, goal, load, temperature, voltage, torque,
    error status and limits. Use it when a motor makes noise, stalls or won't move."""
    with _lock:
        _get_robot()
        bus = _robot.bus
        out = {}
        for j in JOINTS:
            r = {}
            for reg in ("Present_Position", "Goal_Position", "Present_Load", "Present_Temperature",
                        "Present_Voltage", "Torque_Enable", "Status", "Min_Position_Limit", "Max_Position_Limit"):
                try:
                    r[reg] = int(bus.read(reg, j, normalize=False))
                except Exception as e:
                    r[reg] = f"n/a ({type(e).__name__})"
            out[j] = r
        return {"motors": out, "hint": "Temperature in C (stop above ~60), Voltage in 0.1 V, Status != 0 means "
                                       "a motor error (e.g. overload). See README troubleshooting."}


if __name__ == "__main__":
    mcp.run("stdio")

"""Settings for this machine, stored in config.json next to this file (created by setup_claude.py).

config.json is personal (USB port, camera numbers, calibration tweaks) and is ignored by git.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.environ.get("SO101_CLAUDE_CONFIG", os.path.join(HERE, "config.json"))

DEFAULTS = {
    "follower_port": None,          # e.g. /dev/tty.usbmodemXXXX (macOS) or /dev/ttyACM0 (Linux)
    "robot_id": "my_follower",      # the --robot.id you used for lerobot-calibrate
    "frames_dir": os.path.join(HERE, "frames"),
    "table_calibration_file": os.path.join(HERE, "table_calibration.json"),
    "cameras": {
        # name: OpenCV index, capture size, and the width frames are saved at
        "front": {"index": 1, "width": 1920, "height": 1080, "save_width": 960},
        "wrist": {"index": 0, "width": 640, "height": 480, "save_width": 640},
    },
    # Added to LeRobot's wrist_roll angle before using it with the URDF. 0 for a normal LeRobot
    # calibration (wrist centred when you calibrated). tools/check_wrist_roll.py sets it if it re-zeros the wrist.
    "wrist_roll_offset_deg": 0.0,
    # Pixel in the WRIST image where an object sits when it is exactly between the open jaws at hover.
    # Set with the calibrate_jaw_target / set_jaw_target tools. Needed only for center_on (precision mode).
    "jaw_target_px": None,
    # Part of the wrist image covered by the gripper's own fixed jaw, {"x_max": .., "y_min": ..}; the
    # tracker keeps its template out of it. Optional.
    "wrist_jaw_region": None,
    # Saved rest pose (URDF degrees, written by the set_home tool). If null, the pose at server start is used.
    "home": None,
}


def load():
    cfg = json.loads(json.dumps(DEFAULTS))
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            user = json.load(f)
        for k, v in user.items():
            if k == "cameras" and isinstance(v, dict):
                for name, cam in v.items():
                    cfg["cameras"].setdefault(name, {}).update(cam)
            else:
                cfg[k] = v
    return cfg


def save_keys(**updates):
    """Update only the given keys in config.json (keeps everything else as is)."""
    data = {}
    if os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE) as f:
            data = json.load(f)
    data.update(updates)
    tmp = CONFIG_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, CONFIG_FILE)

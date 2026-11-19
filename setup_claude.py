"""One-time setup: finds your arm's USB port and cameras, writes config.json, and adds the robot to
Claude Desktop. Safe to run again (it keeps your other settings and backs up the Claude config first).

    python setup_claude.py
"""
import glob
import json
import os
import platform
import shutil
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import config  # noqa: E402


def ask(prompt, default=None):
    tail = f" [{default}]" if default not in (None, "") else ""
    ans = input(f"{prompt}{tail}: ").strip()
    return ans or (default if default is not None else "")


def yes(prompt, default=True):
    d = "Y/n" if default else "y/N"
    ans = input(f"{prompt} [{d}]: ").strip().lower()
    return default if not ans else ans.startswith("y")


def check_imports():
    missing = []
    for mod, pkg in (("lerobot", "lerobot[feetech]"), ("mcp", "mcp"), ("cv2", "opencv-python"), ("numpy", "numpy")):
        try:
            __import__(mod)
        except ImportError:
            missing.append(pkg)
    if missing:
        print(f"\nMissing packages: {', '.join(missing)}")
        print("Activate the Python environment where LeRobot is installed, then run:")
        print("    pip install -r requirements.txt")
        sys.exit(1)


def serial_ports():
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    ports += (glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/tty.usbserial*") + glob.glob("/dev/tty.wchusbserial*")
              + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return sorted({p for p in ports if "Bluetooth" not in p and not p.startswith("/dev/cu.")})


def find_port(current):
    print("\n== 1. Follower arm USB port ==")
    if current and yes(f"Keep the saved port {current}?"):
        return current
    ports = serial_ports()
    print("Ports found:", ", ".join(ports) or "none")
    if len(ports) == 1 and yes(f"Use {ports[0]} (the follower arm)?"):
        return ports[0]
    input("Unplug the FOLLOWER arm's USB cable (leave power on), then press Enter...")
    time.sleep(0.5)
    gone = [p for p in ports if p not in serial_ports()]
    input("Plug it back in, then press Enter...")
    time.sleep(1.0)
    if len(gone) == 1:
        print(f"Follower arm is on {gone[0]}")
        return gone[0]
    print("Couldn't tell which port it is.")
    return ask("Type the follower port")


def check_calibration(robot_id):
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
    path = os.path.join(HF_LEROBOT_CALIBRATION, "robots", "so_follower", f"{robot_id}.json")
    if os.path.exists(path):
        print(f"Found LeRobot calibration: {path}")
        with open(path) as f:
            w = json.load(f).get("wrist_roll", {})
        if w.get("range_min", 0) <= 0 and w.get("range_max", 4095) >= 4095:
            print("Tip: the wrist has no position limits yet. Run `python tools/check_wrist_roll.py` once.")
    else:
        print(f"WARNING: no LeRobot calibration file for id '{robot_id}'. Calibrate the arm first:")
        print("    lerobot-calibrate --robot.type=so101_follower --robot.port=<port> --robot.id=" + robot_id)


def pick_cameras(cfg):
    print("\n== 3. Cameras ==")
    import cv2
    import numpy as np
    cams = cfg["cameras"]
    if yes(f"Keep saved cameras (front = {cams['front']['index']}, wrist = {cams['wrist']['index']})?",
           default=os.path.exists(config.CONFIG_FILE)):
        return cams
    print("Looking for cameras (macOS may ask Terminal for camera permission: allow it, then re-run)...")
    tiles = []
    for i in range(6):
        cap = cv2.VideoCapture(i)
        ok, frame = cap.read() if cap.isOpened() else (False, None)
        for _ in range(5):                       # first frames are often dark
            if ok:
                ok, frame = cap.read()
        cap.release()
        if ok and frame is not None:
            t = cv2.resize(frame, (480, int(frame.shape[0] * 480 / frame.shape[1])))
            t = cv2.copyMakeBorder(t[:360] if t.shape[0] >= 360 else t, 0, max(0, 360 - t.shape[0]), 0, 0,
                                   cv2.BORDER_CONSTANT)
            cv2.putText(t, f"camera {i}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            tiles.append(t)
            print(f"  camera {i}: OK")
    if not tiles:
        print("No cameras found. Check the cables / Continuity Camera, then run setup again.")
        return cams
    os.makedirs(cfg["frames_dir"], exist_ok=True)
    sheet = os.path.join(cfg["frames_dir"], "camera_check.jpg")
    cv2.imwrite(sheet, np.hstack(tiles))
    print(f"Saved a picture of every camera: {sheet}")
    if platform.system() == "Darwin":
        os.system(f'open "{sheet}"')
    front = int(ask("Number of the FRONT camera (looks down at the table and the arm)", cams["front"]["index"]))
    wrist = int(ask("Number of the WRIST camera (on the gripper)", cams["wrist"]["index"]))
    cams["front"]["index"], cams["wrist"]["index"] = front, wrist
    return cams


def claude_config_path():
    system = platform.system()
    if system == "Darwin":
        return os.path.expanduser("~/Library/Application Support/Claude/claude_desktop_config.json")
    if system == "Windows":
        return os.path.join(os.environ.get("APPDATA", ""), "Claude", "claude_desktop_config.json")
    return None


def add_to_claude():
    print("\n== 4. Claude Desktop ==")
    entry = {"command": sys.executable, "args": [os.path.join(HERE, "server.py")]}
    path = claude_config_path()
    if path is None:
        print("Claude Desktop config location unknown on this OS. Add this under \"mcpServers\" yourself:")
        print(json.dumps({"so101": entry}, indent=2))
        return
    data = {}
    if os.path.exists(path):
        try:
            with open(path) as f:
                data = json.load(f)
        except json.JSONDecodeError:
            print(f"{path} isn't valid JSON, so I won't touch it. Add this under \"mcpServers\" yourself:")
            print(json.dumps({"so101": entry}, indent=2))
            return
    servers = data.setdefault("mcpServers", {})
    if servers.get("so101") == entry:
        print("Claude Desktop already has the robot ('so101').")
        return
    if "so101" in servers and not yes("Claude Desktop already has an 'so101' entry. Replace it?", default=False):
        print("Left it unchanged.")
        return
    if not yes(f"Add the robot to Claude Desktop ({path})?"):
        print("Skipped. Entry to add under \"mcpServers\":")
        print(json.dumps({"so101": entry}, indent=2))
        return
    if os.path.exists(path):
        backup = f"{path}.backup-{time.strftime('%Y%m%d-%H%M%S')}"
        shutil.copy2(path, backup)
        print(f"Backed up your Claude config to {backup}")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    servers["so101"] = entry
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, path)
    print("Added. Quit Claude Desktop completely (Cmd+Q) and open it again to load the robot.")


def main():
    print("SO-101 x Claude setup\n")
    check_imports()
    cfg = config.load()
    port = find_port(cfg.get("follower_port"))
    print("\n== 2. Arm id ==")
    robot_id = ask("The --robot.id you used with lerobot-calibrate", cfg.get("robot_id") or "my_follower")
    check_calibration(robot_id)
    cams = pick_cameras(cfg)
    config.save_keys(follower_port=port, robot_id=robot_id, cameras=cams)
    print(f"\nSaved {config.CONFIG_FILE}")
    add_to_claude()
    print("\nNext:")
    print("  1. Keep `python cameras.py` running in its own Terminal window.")
    print("  2. Put the arm in its folded rest pose and power it on.")
    print("  3. Open Claude Desktop and say: \"Check the robot and help me calibrate the table.\"")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled. Nothing else was changed.")

"""Check (and optionally fix) the follower's wrist_roll range.

Symptom this fixes: the gripper only follows part of its twist, or near one end the wrist motor buzzes,
pushes into its stopper or swings "the other way round". Cause: the wrist's stopper-to-stopper travel
crosses the motor's internal 0/4095 rollover, which happens when the wrist was not centred at
calibration. Fix: move the motor's zero (Homing_Offset) so the rollover sits in the dead zone behind
the stoppers, and give the wrist real position limits.

Run with Claude Desktop QUIT (it holds the USB port) and the arm powered:
    python tools/check_wrist_roll.py
Only the wrist_roll motor is touched. Nothing is written unless you type "yes yes".
"""
import argparse
import json
import os
import shutil
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import config  # noqa: E402

DEG = 360 / 4095


def calib_file(robot_id):
    """The exact file LeRobot (>= 0.6) reads for the SO-101 follower."""
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION
    path = os.path.join(HF_LEROBOT_CALIBRATION, "robots", "so_follower", f"{robot_id}.json")
    if not os.path.exists(path):
        sys.exit(f"No LeRobot calibration file {path}. Calibrate the arm with LeRobot >= 0.6 first.")
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30)
    ap.add_argument("--margin-deg", type=float, default=10.0, help="limit distance from each stopper")
    a = ap.parse_args()
    cfg = config.load()
    if not cfg.get("follower_port"):
        sys.exit("No follower_port in config.json: run setup_claude.py first.")
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    bus = FeetechMotorsBus(port=cfg["follower_port"],
                           motors={"wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES)})
    bus.connect()
    try:
        old_offset = int(bus.read("Homing_Offset", "wrist_roll", normalize=False))
        print(f"Current wrist_roll Homing_Offset = {old_offset}")
        input(f"\nThe wrist will go LIMP for {a.seconds:.0f} s. Then turn the gripper by hand slowly to one "
              "stopper, then all the way to the other. Press Enter to start...")
        bus.disable_torque(["wrist_roll"])
        vals, prev, unwrapped = [], None, 0
        t_end = time.time() + a.seconds
        while time.time() < t_end:
            v = int(bus.read("Present_Position", "wrist_roll", normalize=False))
            if prev is not None:
                d = v - prev
                d -= 4096 if d > 2048 else 0
                d += 4096 if d < -2048 else 0
                unwrapped += d
            else:
                unwrapped = v
            prev = v
            vals.append(unwrapped)
            time.sleep(0.03)
        lo, hi = min(vals), max(vals)
        span = hi - lo
        print(f"\nTravel: {lo} .. {hi} raw = {span * DEG:.0f} deg")
        if span < 1500:
            sys.exit("That's under ~130 deg: it looks like the wrist wasn't turned stopper to stopper. Run again.")
        m = int(round(a.margin_deg / DEG))
        if span > 4096 - 2 * m - 20:
            sys.exit("The wrist turned (almost) a full circle, so its stoppers weren't found. Turn it only between "
                     "the two stoppers and run again.")
        crosses = lo < 0 or hi > 4095
        print("Crosses the motor's rollover: " + ("YES (this is the problem)" if crosses else "no"))
        if crosses:
            gap = 4096 - span
            delta = lo - gap // 2                          # Present_new = Present_old - delta
            new_offset = (old_offset + delta + 2047) % 4096 - 2047   # motor accepts -2047..2047
            if new_offset > 2047:                                      # 2048 can't be stored: shift by one count
                delta -= 1
                new_offset = (old_offset + delta + 2047) % 4096 - 2047
        else:
            print("The zero is fine; only position limits will be added.")
            delta, new_offset = 0, old_offset
        new_lo, new_hi = lo - delta, hi - delta
        lim_lo, lim_hi = max(1, new_lo + m), min(4094, new_hi - m)

        cal_path = calib_file(cfg["robot_id"])
        cal = json.load(open(cal_path))
        w = cal["wrist_roll"]
        old_mid = (w["range_min"] + w["range_max"]) / 2
        new_mid = (lim_lo + lim_hi) / 2
        extra = ((new_mid + delta - old_mid) * DEG + 180) % 360 - 180
        roll_offset = round(float(cfg.get("wrist_roll_offset_deg") or 0) + extra, 1)

        print("\nProposed change (wrist_roll only):")
        print(f"  Homing_Offset:      {old_offset} -> {new_offset}")
        print(f"  Position limits:    {w['range_min']}..{w['range_max']} -> {lim_lo}..{lim_hi} "
              f"({a.margin_deg:.0f} deg from each stopper)")
        print(f"  config.json wrist_roll_offset_deg -> {roll_offset} (keeps Claude's picks aimed the same way)")
        if abs(extra) > 0.5:
            print(f"\nNOTE: LeRobot's wrist_roll zero moves by {extra:.0f} deg. Leader-arm teleop, recorded datasets and "
                  "trained policies will see the follower's wrist rotated by that much.")
        if input('\nType "yes yes" to write it: ').strip().lower() != "yes yes":
            print("Nothing written.")
            return
        backup = f"{cal_path}.backup-{time.strftime('%Y%m%d-%H%M%S')}"
        shutil.copy2(cal_path, backup)
        bus.disable_torque(["wrist_roll"])
        bus.write("Homing_Offset", "wrist_roll", new_offset, normalize=False)
        bus.write("Min_Position_Limit", "wrist_roll", lim_lo, normalize=False)
        bus.write("Max_Position_Limit", "wrist_roll", lim_hi, normalize=False)
        time.sleep(0.1)
        got = [int(bus.read(r, "wrist_roll", normalize=False))
               for r in ("Homing_Offset", "Min_Position_Limit", "Max_Position_Limit")]
        if got != [new_offset, lim_lo, lim_hi]:
            for reg, val in (("Homing_Offset", old_offset), ("Min_Position_Limit", w["range_min"]),
                             ("Max_Position_Limit", w["range_max"])):
                bus.write(reg, "wrist_roll", val, normalize=False)
            sys.exit(f"Read-back mismatch {got}. Restored the motor's old values; calibration file NOT changed.")
        p = int(bus.read("Present_Position", "wrist_roll", normalize=False))
        bus.write("Goal_Position", "wrist_roll", p, normalize=False)   # no jump if torque comes on
        bus.disable_torque(["wrist_roll"])
        w.update(homing_offset=new_offset, range_min=lim_lo, range_max=lim_hi)
        with open(cal_path, "w") as f:
            json.dump(cal, f, indent=4)
        config.save_keys(wrist_roll_offset_deg=roll_offset)
        print(f"\nDone. Calibration updated (backup: {backup}); config.json updated.")
        print("The wrist is limp now; it holds again the next time the arm connects.")
        if not (lim_lo < p < lim_hi):
            print("NOTE: the wrist is resting outside its new limits. Turn it by hand towards the middle "
                  "before using the arm.")
    finally:
        bus.port_handler.closePort()


if __name__ == "__main__":
    main()

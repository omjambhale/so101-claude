# Troubleshooting

## Claude doesn't show the robot tools
- Quit Claude Desktop completely (Cmd+Q on macOS) and reopen it after running `setup_claude.py`.
- Check the `so101` entry in Claude Desktop's config (`setup_claude.py` prints its path). `command` must be
  the Python that has LeRobot installed, and `args` the full path to `server.py`.

## "No camera frames" / "Camera frames are stale"
- `python cameras.py` must be running in its own Terminal window.
- macOS: the first run asks Terminal for camera permission. Allow it and start `cameras.py` again.
- Camera numbers can change when you plug things in or out (iPhone Continuity Camera especially).
  Re-run `python setup_claude.py` and pick the cameras again.

## "Arm is not calibrated for id ..."
- The `robot_id` in `config.json` must match the `--robot.id` you used with `lerobot-calibrate`.

## Can't connect / port busy
- Only one program can talk to the arm at a time. Quit Claude Desktop before running LeRobot teleop,
  `lerobot-calibrate` or anything in `tools/`, and stop those before opening Claude Desktop.
- USB ports can change after replugging: re-run `setup_claude.py`.

## Picks or places land a few cm off
- The front camera moved: ask Claude to redo the table calibration.
- During calibration, hold the gripper pointing straight down (Claude warns if it's tilted more than 20 deg).
- Objects outside the 4 tape marks are less accurate: put the marks around the area you use.

## "Can't reach ... Too far"
- The object is outside the arm's reach pointing down. Move it closer to the arm.

## The gripper twists to the wrong angle when picking
- The wrist's zero doesn't match the URDF. This happens if the wrist was not centred when you ran
  `lerobot-calibrate`, or after changing the wrist calibration. Recalibrate with the wrist centred, or set
  `wrist_roll_offset_deg` in `config.json` (degrees added to the wrist angle; try +/-90 first).

## The wrist only follows part of its turn, buzzes, or swings "the other way round"
The wrist's travel between its two stoppers crosses the motor's internal 0/4095 rollover. Near that point the
motor thinks the far end is a few degrees away and drives into the stopper. With power on it can buzz, trip
its overload protection, and the twisted cable can then spin it back.

Fix (touches only the wrist motor; nothing is written without typing "yes yes"):
```bash
# Quit Claude Desktop first, arm powered on
python tools/check_wrist_roll.py
```
It records the wrist's stopper-to-stopper travel while you turn it by hand, then moves the motor's zero so the
rollover sits behind the stoppers and sets real limits 10 deg from each stopper. It backs up your LeRobot
calibration file first and updates `wrist_roll_offset_deg` so picks stay aimed correctly.

Note: if the zero moves, LeRobot's wrist_roll angle shifts by the amount the tool prints. Leader-arm teleop,
recorded datasets and trained policies will see the wrist rotated by that much (re-record or recalibrate
the leader to match if you use them).

## A motor buzzes, gets hot, or stops responding
1. Unplug the arm's power (safe stop).
2. Wait a few minutes, plug back in, and ask Claude to run `diagnose`.
   - `Status` not 0: the motor tripped a protection (often overload). Power-cycling clears it.
   - `Present_Temperature` above ~60: let it cool.
3. If one joint keeps stalling against something, check the cable routing and that printed parts don't rub.

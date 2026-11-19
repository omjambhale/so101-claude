# so101-claude

**Control a robot arm with just Claude. No data collection. No training.**

<!-- DEMO_VIDEO -->

*Claude spelling FABLE52 with foam letters, on its own, using an SO-101 arm and two cameras.*

## Why this exists

The usual way to teach a robot arm a new task:

1. Record dozens of demonstrations by moving the arm yourself.
2. Train a model on a GPU for hours.
3. Get a robot that does that **one** task. New task? Start again.

We did exactly that to teach our arm to pick up an eraser. It took hours of recording and training for one task.

This project skips all of it. Claude looks at the table through a camera, decides what to do, and moves the arm
with a few simple tools. You just tell it what you want, in plain English, and it does it. Any task, right away.

All you need is the arm and a Claude subscription.

## What you need

- An **SO-101 robot arm**, set up with [LeRobot](https://huggingface.co/docs/lerobot/so101)
- A **camera looking down at the table** (a webcam, or your iPhone) and a **camera on the gripper**
- **Claude Desktop** (Mac or Windows) with a Claude subscription
- 4 small pieces of **blue or yellow tape**

## Get started (about 15 minutes)

```bash
git clone https://github.com/omjambhale/so101-claude.git
cd so101-claude
pip install -r requirements.txt   # in the Python environment where LeRobot is installed
python setup_claude.py            # finds your arm and cameras, connects them to Claude Desktop
```

Then:

1. Run `python cameras.py` and leave it running.
2. Restart Claude Desktop.
3. Say: **"Check the robot and help me calibrate the table."** Claude walks you through the rest.

## Try saying

- "Spell my name with the foam letters."
- "Put everything red into the bowl."
- "Sort the blocks by colour, left and right."
- "Stack the cubes, biggest at the bottom."
- "Pass me the pen." (it puts it at the edge of the table, near you)
- "Clear the table into the box."

More in [examples/PROMPTS.md](examples/PROMPTS.md).

## Safety

It's a real machine. Keep a hand near the arm's power plug: **unplugging it is always a safe stop.**
When Claude calibrates the table, all motors go limp, so hold the arm.

---

## How it works (technical details)

```
 front camera ─┐                       ┌─> pick(x, y) / place(x, y) / move_to / ...
 wrist camera ─┴─> cameras.py ─> frames/ ─> server.py (MCP) <──> Claude Desktop
                                               │
                                               └─> LeRobot ─> SO-101 follower arm
```

- `server.py` is an [MCP](https://modelcontextprotocol.io) server. It gives Claude tools to **look** (camera
  images with a pixel grid) and to **act** (pick, place, move, open/close the gripper, go home).
- Claude reads an object's pixel in the overhead image. A one-time **table calibration** (touch the gripper tip to
  4 tape marks) maps pixels to table positions, and inverse kinematics on the official SO-101 URDF turns
  positions into joint moves.
- Safety lives in the server, not in the model: the gripper never goes below the table, moves are slow and
  smooth, joints stay inside their calibrated limits, nothing moves while you hold the arm during calibration,
  and the gripper holds objects lightly instead of squeezing at full force.

**Requirements:** Python 3.12+, LeRobot 0.6+ (the arm calibrated with `lerobot-calibrate`), `mcp` (see
`requirements.txt`). Recommended once: `python tools/check_wrist_roll.py` gives the wrist safe limits (quit
Claude Desktop first). On macOS, allow Terminal to use the camera the first time. `python preview.py` shows what
Claude sees (needs `pip install opencv-python`).

### Tools Claude gets

| Tool | What it does |
|---|---|
| `get_state` | Joint angles, gripper position, whether the table is calibrated |
| `look` | Latest front or wrist image, with an optional pixel grid or 3x zoom |
| `pick` / `place` | Pick the object at a pixel of the front image; place it at another pixel |
| `move_to`, `turn_wrist`, `set_gripper` | Small corrections |
| `go_home`, `set_home` | Return to / save the rest pose |
| `calibration_start` / `_add_point` / `_finish` / `_cancel` | Table calibration |
| `calibrate_jaw_target`, `set_jaw_target`, `center_on`, `grab_here`, `put_down` | Precision mode: lines objects up with the wrist camera (for tight placing like spelling) |
| `diagnose` | Read-only motor health check (temperature, load, errors) |

### Troubleshooting

See [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md). Most common:

- **"Camera frames are stale"**: `cameras.py` isn't running, or a camera number changed (re-run `setup_claude.py`).
- **Picks land a bit off**: the camera moved. Ask Claude to redo the table calibration.
- **The wrist only follows part of its turn, or buzzes near one end**: run `python tools/check_wrist_roll.py`.
- **"Port busy"**: only one program can use the arm. Quit Claude Desktop before running LeRobot or `tools/`.

### Files

```
server.py                  MCP server (the tools above)
kinematics.py              forward / inverse kinematics from the SO-101 URDF (numpy only)
cameras.py                 camera daemon -> frames/
preview.py                 live view of the cameras
setup_claude.py            one-time setup
config.py                  settings (your config.json is never committed)
tools/check_wrist_roll.py  fixes the wrist-roll rollover problem
assets/                    SO-101 URDF (Apache-2.0, TheRobotStudio)
```

## Credits

[LeRobot](https://github.com/huggingface/lerobot) by Hugging Face, and the SO-101 by
[TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100).

## License

MIT. The SO-101 URDF in `assets/` is Apache-2.0 (see `assets/NOTICE.md`).

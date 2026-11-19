# Example prompts

Start every session with: **"Check the robot."** (Claude calls `get_state` and tells you if anything is missing.)

## First time
- "Help me calibrate the table." (needs the 4 tape marks)
- "Set up precision mode." (optional, for tight placing; uses the wrist camera)

## Simple
- "What do you see on the table?"
- "Pick up the yellow block and put it in the top-left corner."
- "Open the gripper halfway." / "Go home."

## Multi-step
- "Spell FABLE52 with the foam letters in a straight line near the bottom edge of the mat. Turn each letter
  so it reads correctly from the camera."
- "Sort the letters: vowels on the left, consonants on the right."
- "Stack the three cubes, biggest at the bottom."
- "Put every pen into the cup."

## Tips
- Objects should be within reach and inside the area of the 4 tape marks.
- Flat, light objects at least ~1.5 cm wide work best with the stock gripper.
- If something slips, ask Claude to look with the wrist camera and try again; it adjusts.

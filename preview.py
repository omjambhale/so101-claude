"""Live preview of what Claude sees (reads frames/*.jpg; changes nothing). cameras.py must be running.
    python preview.py            # front + wrist side by side
    python preview.py front      # one camera
Press q in the window to close."""
import os
import sys
import time

import cv2
import numpy as np

import config

FRAMES = config.load()["frames_dir"]
H = 480                                           # display height of each view


def view(name):
    path = os.path.join(FRAMES, f"{name}.jpg")
    img = cv2.imread(path)
    if img is None:
        img = np.zeros((H, 640, 3), np.uint8)
        cv2.putText(img, f"no {name} frame", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return img
    h, w = img.shape[:2]
    img = cv2.resize(img, (int(w * H / h), H))
    age = time.time() - os.path.getmtime(path)
    color = (0, 255, 0) if age < 2 else (0, 0, 255)
    cv2.putText(img, f"{name}  age {age:.1f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return img


if __name__ == "__main__":
    names = sys.argv[1:] or ["front", "wrist"]
    try:
        while True:
            cv2.imshow("SO-101 cameras (q to close)", np.hstack([view(n) for n in names]))
            if cv2.waitKey(150) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()
    except cv2.error:
        sys.exit("This OpenCV build has no windows (LeRobot installs the 'headless' one). For the live preview run\n"
                 "    pip install opencv-python\nor just open frames/front.jpg and frames/wrist.jpg.")

"""Camera daemon: keeps the newest frame of each camera in frames/<name>.jpg a few times per second.

Run it in its own Terminal and leave it running while you use Claude:
    python cameras.py

The MCP server only reads these files, so Claude Desktop never needs camera permission itself.
Cameras are set in config.json (setup_claude.py picks them for you).
"""
import os
import threading
import time

import cv2

import config

SAVE_EVERY_S = 0.15


class LatestFrame:
    """Reads a camera non-stop in its own thread so we always save the NEWEST frame
    (reading only a few times a second makes OpenCV hand back old, buffered frames)."""

    def __init__(self, index, width, height):
        self.cap = cv2.VideoCapture(index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.ok, self.frame = False, None
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            ok, frame = self.cap.read()
            with self.lock:
                self.ok, self.frame = ok, frame
            if not ok:
                time.sleep(0.05)

    def read(self):
        with self.lock:
            return self.ok, None if self.frame is None else self.frame.copy()


def main():
    cfg = config.load()
    frames_dir = cfg["frames_dir"]
    os.makedirs(frames_dir, exist_ok=True)
    cams = {name: (LatestFrame(c["index"], c["width"], c["height"]), c["save_width"])
            for name, c in cfg["cameras"].items()}
    print(f"Saving frames to {frames_dir}  (Ctrl+C to stop)")
    time.sleep(1.0)
    last_report = 0.0
    while True:
        status = []
        for name, (cam, save_w) in cams.items():
            ok, frame = cam.read()
            if not ok or frame is None:
                status.append(f"{name}: NO FRAME")
                continue
            h, w = frame.shape[:2]
            if w != save_w:
                frame = cv2.resize(frame, (save_w, int(h * save_w / w)), interpolation=cv2.INTER_AREA)
            tmp = os.path.join(frames_dir, f".{name}.tmp.jpg")
            cv2.imwrite(tmp, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            os.replace(tmp, os.path.join(frames_dir, f"{name}.jpg"))  # atomic swap
            status.append(f"{name}: {frame.shape[1]}x{frame.shape[0]}")
        if time.time() - last_report > 5:
            print(" | ".join(status), flush=True)
            last_report = time.time()
        time.sleep(SAVE_EVERY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

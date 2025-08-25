"""
Person detection (ESP32-CAM stream) + LED/Buzzer alarm over HTTP.
- Stream:   http://192.168.4.1:81/stream  (default ESP32-CAM)
- LED API:  http://192.168.4.1/led?state=on/off

pip install opencv-python numpy requests
Place these files beside this script:
  - MobileNetSSD_deploy.prototxt
  - MobileNetSSD_deploy.caffemodel
"""

import cv2
import time
import threading
import requests
import numpy as np
from collections import deque
from pathlib import Path

# -------------------------
# Configuration
# -------------------------
STREAM_URL   = "http://192.168.4.1:81/stream"
LED_API_URL  = "http://192.168.4.1/led"
CONF_THRESH  = 0.5          # detection confidence threshold
DEBOUNCE_ON  = 3            # frames with person before starting alarm
DEBOUNCE_OFF = 30           # frames with no person before stopping alarm
SHOW_WINDOW  = True         # set False if running headless
DRAW_FPS     = True

PROTOTXT     = "MobileNetSSD_deploy.prototxt"
CAFFEMODEL   = "MobileNetSSD_deploy.caffemodel"

# MobileNet-SSD class labels (Caffe, 20 classes + background)
CLASSES = [
    "background","aeroplane","bicycle","bird","boat","bottle","bus","car",
    "cat","chair","cow","diningtable","dog","horse","motorbike","person",
    "pottedplant","sheep","sofa","train","tvmonitor"
]
PERSON_IDX = CLASSES.index("person")

# -------------------------
# LED/Buzzer control
# -------------------------
def set_led(state: bool, timeout=0.8):
    """Send LED on/off, mapped to your buzzer as well (GPIO4)."""
    try:
        r = requests.get(LED_API_URL, params={"state": "on" if state else "off"}, timeout=timeout)
        # Optional: check r.status_code == 200
    except requests.RequestException:
        pass  # network hiccup: ignore and keep going

def pulse(on_ms: int, off_ms: int):
    """One on/off pulse measured in milliseconds."""
    set_led(True)
    time.sleep(on_ms / 1000.0)
    set_led(False)
    time.sleep(off_ms / 1000.0)

class AlarmPlayer:
    """
    Plays a non-blocking alarm 'tune' (rhythm) by toggling LED/buzzer in a thread.
    Call start() to begin, stop() to end. Re-entrant safe.
    """
    def __init__(self):
        self._t = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        # A simple pattern: short-short-long | short-short-long (loop)
        # (on_ms, off_ms) pairs; tweak to taste.
        self.pattern = [
            (120, 120), (120, 200), (400, 250),
            (120, 120), (120, 200), (400, 350),
        ]

    def _run(self):
        while not self._stop.is_set():
            for on_ms, off_ms in self.pattern:
                if self._stop.is_set():
                    break
                pulse(on_ms, off_ms)

    def start(self):
        with self._lock:
            if self._t and self._t.is_alive():
                return
            self._stop.clear()
            self._t = threading.Thread(target=self._run, daemon=True)
            self._t.start()

    def stop(self):
        with self._lock:
            if not self._t:
                return
            self._stop.set()
            self._t.join(timeout=1.0)
            self._t = None
        set_led(False)  # ensure off at the end

# -------------------------
# Frame generator (yield to keep stream fluid)
# -------------------------
def frames_from_stream(url: str):
    """
    Generator yielding frames from an MJPEG stream using OpenCV.
    Using yield prevents blocking the rest of your program.
    """
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open stream: {url}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                # brief backoff and retry read
                time.sleep(0.05)
                continue
            yield frame
    finally:
        cap.release()

# -------------------------
# Load detector
# -------------------------
def load_mobilenet_ssd(prototxt: str, model: str):
    if not (Path(prototxt).exists() and Path(model).exists()):
        raise FileNotFoundError(
            "Missing model files.\n"
            "Place 'MobileNetSSD_deploy.prototxt' and 'MobileNetSSD_deploy.caffemodel' "
            "in the same folder as this script."
        )
    net = cv2.dnn.readNetFromCaffe(prototxt, model)
    return net

def detect_person(net, frame):
    """
    Returns (has_person: bool, detections: list)
    detections: list of (conf, (x1,y1,x2,y2), class_id)
    """
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()
    found = False
    out = []
    # detections shape: [1, 1, N, 7]: [image_id, label, conf, x1, y1, x2, y2]
    for i in range(detections.shape[2]):
        conf = float(detections[0, 0, i, 2])
        if conf < CONF_THRESH:
            continue
        class_id = int(detections[0, 0, i, 1])
        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
        (x1, y1, x2, y2) = box.astype("int")
        out.append((conf, (x1, y1, x2, y2), class_id))
        if class_id == PERSON_IDX:
            found = True
    return found, out

# -------------------------
# Main loop
# -------------------------
def main():
    # Load detector
    net = load_mobilenet_ssd(PROTOTXT, CAFFEMODEL)

    # Alarm player (non-blocking)
    alarm = AlarmPlayer()
    alarm_on = False

    # Debounce windows
    seen_person = deque(maxlen=DEBOUNCE_ON)
    seen_nobody = deque(maxlen=DEBOUNCE_OFF)

    # FPS calc
    last = time.time()
    fps = 0.0

    for frame in frames_from_stream(STREAM_URL):  # <- generator with yield inside
        has_person, dets = detect_person(net, frame)

        # Debounce logic
        if has_person:
            seen_person.append(1)
            seen_nobody.clear()
        else:
            seen_nobody.append(1)
            seen_person.clear()

        # Turn alarm on/off based on debounce
        if (not alarm_on) and (len(seen_person) == DEBOUNCE_ON):
            alarm.start()
            set_led(True)   # ensure on immediately
            alarm_on = True

        if alarm_on and (len(seen_nobody) == DEBOUNCE_OFF):
            alarm.stop()
            alarm_on = False

        # Visualization (optional)
        if SHOW_WINDOW:
            # draw detections
            for conf, (x1, y1, x2, y2), cid in dets:
                label = CLASSES[cid] if 0 <= cid < len(CLASSES) else str(cid)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label}:{conf:.2f}", (x1, max(20, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                            cv2.LINE_AA)

            # FPS
            if DRAW_FPS:
                now = time.time()
                df = now - last
                last = now
                fps = 0.9 * fps + 0.1 * (1.0 / df if df > 0 else 0.0)
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                            cv2.LINE_AA)

            cv2.imshow("ESP32-CAM (Person Detector)", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
                break

    # Cleanup
    alarm.stop()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # ensure LED/buzzer off if you Ctrl+C
        set_led(False)

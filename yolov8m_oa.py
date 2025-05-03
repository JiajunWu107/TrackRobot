"""
Enhanced YOLOv8 navigation demo
------------------------------
Tracks a single *target* class **and** performs reactive obstacle avoidance
by computing linear (v) and angular (w) velocities that are sent over UDP to
an embedded controller.

Usage
~~~~~
    python yolov8m_oa.py                # default target = "chair"
    python yolov8m_oa.py class=person   # set a different target class

The script keeps the original behaviour of approaching the most confidently
seen target, but augments it with a simple repulsive‐field obstacle
avoidance strategy:

* **Attractive term** – points the robot towards the horizontal centre of
  the highest–confidence *target* bounding‑box and decides a base forward
  speed based on the target's image area (proxy for distance).
* **Repulsive term** – for every *non‑target* detection whose centre lies in
  the lower half of the image (i.e. likely on the ground plane), a repulsive
  angular velocity is added that pushes the robot away from the obstacle.
  If an obstacle sits directly in front and is close (large image area), the
  linear speed is capped to a slow crawl or zero to prevent collision.

Only the computation of **(v, w)** has changed; everything else (model
loading, display, UDP protocol) remains drop‑in compatible with the original
script.
"""

from __future__ import annotations

import sys
import socket
from typing import List, Tuple

import cv2
import torch
from ultralytics import YOLO

# ──────────────────────────────────────────────────────────────────────
# Runtime configuration
# ──────────────────────────────────────────────────────────────────────
DEFAULT_CLASS = "chair"                 # default target class to follow
CAM_INDEX = 0                           # OpenCV camera index / video path
JETSON_IP = "10.5.144.120"             # UDP peer
JETSON_PORT = 8888

# Velocity limits (m s⁻¹ and rad s⁻¹, change to fit your robot)
MAX_V = 0.30        # forward speed upper bound
MAX_W = 0.16        # yaw rate upper bound (|w| ≤ MAX_W)

# Obstacle‑avoidance heuristics
GROUND_Y_FRAC = 0.5   # consider detections whose centre‑y is below H×0.5
AREA_STOP_FRAC = 1/3  # stop if obstacle area ≥ (W×H)×AREA_STOP_FRAC
AREA_SLOW_FRAC = 1/7  # slow if obstacle area ≥ (W×H)×AREA_SLOW_FRAC
CENTRE_BAND_FRAC = 0.25  # |offset| < W×CENTRE_BAND_FRAC ⇒ "ahead"

# ──────────────────────────────────────────────────────────────────────
# Helper functions
# ──────────────────────────────────────────────────────────────────────
BBox = Tuple[float, float, float, float, float, str]  # (x1,y1,x2,y2,conf,label)


def attractive_term(target_box: BBox | None, img_w: int, img_h: int) -> Tuple[float, float]:
    """Convert the *target* bounding‑box into base (v, w).

    If *target_box* is *None* no attractive motion is produced.
    """
    if target_box is None:
        return 0.0, 0.0

    x1, y1, x2, y2, conf, _ = target_box
    centre_x = (x1 + x2) / 2
    offset_x = centre_x - img_w / 2  # +ve ⇒ target on right

    # Angular velocity towards target (proportional control)
    w_tgt = (offset_x / (img_w / 2)) * MAX_W
    w_tgt = max(-MAX_W, min(w_tgt, MAX_W))

    # Linear velocity – faster when the box is small (far)
    area = (x2 - x1) * (y2 - y1)
    far_thresh = (img_w * img_h) / 10
    mid_thresh = (img_w * img_h) / 3
    if area < far_thresh:
        v_tgt = MAX_V
    elif area < mid_thresh:
        v_tgt = 0.10
    else:
        v_tgt = 0.0  # already very close

    return v_tgt, w_tgt


def repulsive_term(obstacles: List[BBox], img_w: int, img_h: int) -> Tuple[float, float]:
    """Compute repulsive (Δv, Δw) from *obstacle* detections.

    A simple potential‑field: every obstacle exerts a torque inversely
    proportional to its horizontal distance from the image centre and
    proportional to its relative area.  Obstacles directly ahead and close
    may also reduce the forward speed.
    """
    if not obstacles:
        return 0.0, 0.0

    delta_w = 0.0
    slow_factor = 1.0  # multiplier for the attractive linear velocity

    img_area = img_w * img_h
    centre_band = img_w * CENTRE_BAND_FRAC

    for (x1, y1, x2, y2, conf, label) in obstacles:
        centre_x = (x1 + x2) / 2
        centre_y = (y1 + y2) / 2
        area = (x2 - x1) * (y2 - y1)

        # Ignore obstacles that are likely not on the ground plane (upper half)
        if centre_y < img_h * GROUND_Y_FRAC:
            continue

        # Angular repulsion (steer away)
        offset_x = centre_x - img_w / 2
        # direction: obstacle left ⇒ turn right (−offset_x)
        normalized_offset = -(offset_x / (img_w / 2))
        strength = area / img_area  # 0‒1 proxy for distance
        delta_w += normalized_offset * strength * MAX_W

        # Linear speed modulation if obstacle is right in front
        if abs(offset_x) < centre_band:
            if area >= img_area * AREA_STOP_FRAC:
                slow_factor = 0.0  # immediate stop
            elif area >= img_area * AREA_SLOW_FRAC:
                slow_factor = min(slow_factor, 0.3)  # crawl

    # Clamp angular component
    delta_w = max(-MAX_W, min(delta_w, MAX_W))
    delta_v = -1.0 * (1 - slow_factor) * MAX_V  # negative because we *reduce* v

    return delta_v, delta_w


def compute_control_signals(target_boxes: List[BBox], obstacles: List[BBox], img_w: int, img_h: int) -> Tuple[float, float]:
    """Fuse attractive and repulsive terms into final (v, w)."""
    # Attractive – pick highest‑confidence target (list already sorted)
    tgt_box = target_boxes[0] if target_boxes else None
    v_attr, w_attr = attractive_term(tgt_box, img_w, img_h)

    # Repulsive – accumulate effects of all obstacles
    v_rep, w_rep = repulsive_term(obstacles, img_w, img_h)

    v = max(0.0, min(v_attr + v_rep, MAX_V))
    w = max(-MAX_W, min(w_attr + w_rep, MAX_W))

    return v, w


# ──────────────────────────────────────────────────────────────────────
# Main script
# ──────────────────────────────────────────────────────────────────────

def main() -> None:
    # Parse command‑line arguments (syntax:  class=person)
    target_class = DEFAULT_CLASS
    for arg in sys.argv[1:]:
        if arg.lower().startswith("class="):
            target_class = arg.split("=", 1)[1].strip().lower()

    # Initialise YOLOv8 model
    model = YOLO("yolov8m.pt")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model.to(device)

    coco_names = {name.lower() for name in model.names.values()}
    if target_class not in coco_names:
        print(f"[WARN] Unknown class '{target_class}'. Falling back to '{DEFAULT_CLASS}'.")
        target_class = DEFAULT_CLASS
    print(f"[INFO] Target class   : {target_class}")
    print(f"[INFO] Compute device : {device}")

    # UDP socket to embedded controller
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Failed to open camera.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            img_h, img_w = frame.shape[:2]

            # ----------------------------------------- inference
            results = model.predict(frame, conf=0.25, verbose=False, device=device)[0]

            # Separate detections into target vs obstacle lists
            target_boxes: List[BBox] = []
            obstacle_boxes: List[BBox] = []

            for box in sorted(results.boxes, key=lambda b: float(b.conf[0]), reverse=True):
                cls_id = int(box.cls[0])
                label = results.names[cls_id].lower()
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0]
                record: BBox = (float(x1), float(y1), float(x2), float(y2), conf, label)

                if label == target_class:
                    target_boxes.append(record)
                else:
                    obstacle_boxes.append(record)

            # ----------------------------------------- control logic
            v, w = compute_control_signals(target_boxes, obstacle_boxes, img_w, img_h)
            print(f"v={v:.2f}  w={w:.2f}", end="\r")

            # Send command
            cmd = f"!{v:.2f}@{w:.2f}#"
            sock.sendto(cmd.encode(), (JETSON_IP, JETSON_PORT))

            # ----------------------------------------- visualisation (optional)
            for (x1, y1, x2, y2, conf, label) in target_boxes:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            for (x1, y1, x2, y2, conf, label) in obstacle_boxes:
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 1)

            cv2.putText(frame, f"v={v:.2f}  w={w:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("YOLOv8 Navigation", frame)

            if cv2.waitKey(1) & 0xFF == 27:  # ESC
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

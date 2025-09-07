import cv2
from ultralytics import YOLO
import serial
import time

model = YOLO("yolov8s.pt")  # Pretrained on COCO
serial = serial.Serial(port = "/dev/ttyTHS1", baudrate=9600)   
def compute_control_signals(bboxes, img_w, img_h):
    if not bboxes:
        return 0.0, 0.0
    x1, y1, x2, y2, conf, label = bboxes[0]
    bx_center_x = (x1 + x2) / 2
    bx_center_y = (y1 + y2) / 2
    offset_x = bx_center_x - (img_w / 2)
    
    max_w = 0.4
    w = (offset_x / (img_w / 2)) * max_w
    w = max(-max_w, min(w, max_w))

    max_v = 0.8
    box_area = (x2 - x1) * (y2 - y1)
    area_thresh = (img_w * img_h) / 10
    if box_area < area_thresh:
        v = 0.8
    else:
        v = 0.3
    v = max(0.0, min(v, max_v))

    return v, w

cap = cv2.VideoCapture(0)  # or a video file
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # YOLOv8 inference
    results = model.predict(frame, conf=0.25)
    r = results[0]

    # Filter out bboxes for a desired class, e.g. "chair"
    target_class = "chair"
    bboxes = []
    for box in r.boxes:
        x1, y1, x2, y2 = box.xyxy[0]
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        label = r.names[cls_id]  # e.g. "chair"
        if label == target_class and conf > 0.25:
            bboxes.append((x1, y1, x2, y2, conf, label))

    # Compute (v, w) for the robot
    img_h, img_w = frame.shape[:2]
    v, w = compute_control_signals(bboxes, img_w, img_h)
    print(f"v={v:.2f}, w={w:.2f}")
    
    time.sleep(1) # Wait for port to initialize

    # Turn data into a string and write
    data = '!'+ str(v) + '@' + str(w) + '#'
    serial.write(data.encode('utf-8'))

    # Draw bounding boxes
    for (x1, y1, x2, y2, conf, label) in bboxes:
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    cv2.putText(frame, f"v={v:.2f}, w={w:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    cv2.imshow("YOLOv8 Robot Demo", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break
serial.close()
cap.release()
cv2.destroyAllWindows()

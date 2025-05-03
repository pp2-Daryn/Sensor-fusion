# follow_me_yolo_kalman.py  (fixed for newer Ultralytics API)
# ---------------------------------------------------------------
import cv2
from ultralytics import YOLO
import numpy as np
import socket, time

# ── 0.  Settings ────────────────────────────────────────────────
ESP_IP, PORT = "192.168.8.35", 4210
TX_INTERVAL  = 0.10
CENTER_LEFT, CENTER_RIGHT = 0.35, 0.65
NEAR_THR, FAR_THR         = 0.8, 0.5

# ── 1.  UDP socket ----------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ADDR = (ESP_IP, PORT)
CODE = {"left":b"L","right":b"R","forward":b"F","back":b"B","stop":b"S"}

# ── 2.  Camera ---------------------------------------------------
cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
if not cam.isOpened():
    raise RuntimeError("No camera found")
W, H = map(int, (cam.get(3), cam.get(4)))   # width, height

# ── 3.  YOLOv8‑Nano ---------------------------------------------
model = YOLO("yolov8n.pt")                  # tiny model

# ── 4.  Kalman filter (4×1 state) -------------------------------
dt = 1/30
kf = cv2.KalmanFilter(4,2,0,cv2.CV_32F)
kf.transitionMatrix  = np.array([[1,dt,0,0],[0,1,0,0],[0,0,1,dt],[0,0,0,1]],np.float32)
kf.measurementMatrix = np.array([[1,0,0,0],[0,0,1,0]],np.float32)
kf.processNoiseCov      = np.eye(4, dtype=np.float32) * 1e-2   # 4×4
kf.measurementNoiseCov  = np.eye(2, dtype=np.float32) * 2.0    # 2×2
kf.errorCovPost         = np.eye(4, dtype=np.float32)          # 4×4
kf.statePost         = np.array([[W/2],[0],[H*0.25],[0]],np.float32)

# ── 5.  Helpers --------------------------------------------------
def decide_dir(cx, h):
    if cx < W*CENTER_LEFT:  return "left"
    if cx > W*CENTER_RIGHT: return "right"
    if h  > H*NEAR_THR:     return "forward"
    if h  < H*FAR_THR:      return "back"
    return "stop"

def send(label, last, t_last):
    now=time.time()
    if label!=last or now-t_last>=TX_INTERVAL:
        sock.sendto(CODE[label], ADDR); print("→", label.upper())
        return label, now
    return last, t_last

# ── 6.  Main loop -----------------------------------------------
last_cmd, last_tx = "stop", 0.0

while True:
    ok, frame = cam.read()
    if not ok: break
    frame = cv2.flip(frame,1)

    x_kal,_,h_kal,_ = kf.predict()          # predict always

    res = model(frame, verbose=False)[0]
    boxes = res.boxes
    if boxes:                               # any detection
        cls  = boxes.cls.cpu().numpy().astype(int)
        person_idx = np.where(cls==0)[0]    # class 0 = person
        if person_idx.size:
            # choose largest box by area
            xyxy = boxes.xyxy.cpu().numpy()[person_idx]
            areas = (xyxy[:,2]-xyxy[:,0])*(xyxy[:,3]-xyxy[:,1])
            i    = person_idx[areas.argmax()]
            x1,y1,x2,y2 = map(int, boxes.xyxy[i])
            cx_meas = (x1+x2)/2
            h_meas  = (y2-y1)
            kf.correct(np.array([[cx_meas],[h_meas]],np.float32))
            cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)

    cmd = decide_dir(x_kal, h_kal)
    last_cmd, last_tx = send(cmd, last_cmd, last_tx)

    cv2.line(frame,(W//2,0),(W//2,H),(200,200,200),1)
    cv2.putText(frame,cmd.upper(),(10,35),
                cv2.FONT_HERSHEY_SIMPLEX,1.2,(255,255,255),2)
    cv2.imshow("YOLO Follow‑Me Rover",frame)
    if cv2.waitKey(1)&0xFF==27: break

cam.release()
cv2.destroyAllWindows()

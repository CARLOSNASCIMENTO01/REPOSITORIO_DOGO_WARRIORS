from ultralytics import YOLO
import cv2
import numpy as np

# =================== CONFIGURAÇÕES ===================
MODEL_PATH = "modelon.onnx"
CAMERA_INDEX = 2   # confirmado: USB Camera real
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35

model = YOLO(MODEL_PATH)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Erro: não foi possível abrir a câmera no índice", CAMERA_INDEX)
    exit()

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("Erro ao ler frame, encerrando.")
        break

    h, w = frame.shape[:2]
    start_x = w // 2 - CROP_W // 2
    start_y = h // 2 - CROP_H // 2
    cropped = frame[start_y:start_y + CROP_H, start_x:start_x + CROP_W]

    results = model.predict(cropped, imgsz=(CROP_W, CROP_H), conf=CONF_THRES, verbose=False)
    annotated = results[0].plot()

    cv2.imshow("Deteccao YOLOv8 ONNX", annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
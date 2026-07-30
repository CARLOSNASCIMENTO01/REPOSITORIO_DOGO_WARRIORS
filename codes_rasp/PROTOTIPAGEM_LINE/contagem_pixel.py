import cv2
import numpy as np
# ==============================
# CÃ‚MERA
# ==============================
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
# ==============================
# LIMIAR DO PRETO (HSV)
# ==============================
BLACK_MIN = np.array([0, 0, 0], dtype=np.uint8)
BLACK_MAX = np.array([180, 255, 60], dtype=np.uint8)
kernel = np.ones((3,3), np.uint8)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    altura, largura = frame.shape[:2]
    # =====================================
    # ROI
    # Usa toda a largura
    # Apenas os 50% inferiores
    # =====================================
    y_inicio = altura // 2
    roi = frame[y_inicio:, :]
    # =====================================
    # HSV
    # =====================================
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLACK_MIN, BLACK_MAX)
    # Remove ruÃ­dos
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Fecha pequenos buracos
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # =====================================
    # CONTAGEM
    # =====================================
    pixels_pretos = cv2.countNonZero(mask)
    pixels_totais = mask.shape[0] * mask.shape[1]
    porcentagem = (pixels_pretos / pixels_totais) * 100
    # =====================================
    # DESENHA A ROI
    # =====================================
    cv2.rectangle(
        frame,
        (0, y_inicio),
        (largura, altura),
        (255, 0, 0),
        2
    )
    # =====================================
    # TEXTO
    # =====================================
    cv2.putText(
        frame,
        f"Pixels Pretos: {pixels_pretos}",
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )
    cv2.putText(
        frame,
        f"{porcentagem:.2f}% Preto",
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )
    # =====================================
    # JANELAS
    # =====================================
    cv2.imshow("Camera", frame)
    cv2.imshow("ROI", roi)
    cv2.imshow("Mascara", mask)
    tecla = cv2.waitKey(1) & 0xFF
    if tecla == 27:
        break
cap.release()
cv2.destroyAllWindows()
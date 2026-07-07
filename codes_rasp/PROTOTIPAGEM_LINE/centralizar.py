from ultralytics import YOLO
import cv2
import lgpio as GPIO
import time

# ===================== CONFIGURAÇÕES =====================
MODEL_PATH = "model.onnx"
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"  # camera de resgate (bolas)
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

# Pinos
IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13

VEL_MAX = 50
VEL_MIN = -VEL_MAX

TURN_PULSE = 0.2
STOP_TIME = 0.2
FORWARD_TIME = 0.6

# ===================== SETUP GPIO (lgpio) =====================
h = GPIO.gpiochip_open(0)
pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for p in pins:
    GPIO.gpio_claim_output(h, p)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# ===================== MOTORES (lgpio) =====================
def aplicar_motor(in1, in2, pwm_pin, vel):
    vel = max(min(vel, VEL_MAX), VEL_MIN)
    if vel > 0:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 1)
    elif vel < 0:
        GPIO.gpio_write(h, in1, 1)
        GPIO.gpio_write(h, in2, 0)
    else:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 0)
    duty = min(abs(vel), 100)
    GPIO.tx_pwm(h, pwm_pin, 50, duty)


def mover(velL, velR):
    aplicar_motor(IN2, IN1, ENA, velL)
    aplicar_motor(IN4, IN3, ENB, velR)


def parar_motores():
    mover(0, 0)


def forward(speed=30):
    mover(speed, speed)


def backward(speed=30):
    mover(-speed, -speed)


def turn_left(speed=30):
    mover(-speed, speed)


def turn_right(speed=30):
    mover(speed, -speed)


# ===================== FUNÇÕES AUXILIARES =====================
def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()


# ===================== SETUP CÂMERA E MODELO =====================
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"❌ Não consegui abrir a câmera de resgate em {CAMERA_INDEX}")
    exit(1)

model = YOLO(MODEL_PATH)

# ===================== LOOP PRINCIPAL =====================
try:
    print("🚗 Iniciando busca por bolas...")
    while True:
        turn_left(speed=55)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(STOP_TIME)

        ret, frame = cap.read()
        if not ret:
            print("❌ Erro na câmera")
            break
        crop = center_crop(frame, CROP_W, CROP_H)

        results = model.predict(crop, imgsz=[CROP_W, CROP_H], conf=CONF_THRES, verbose=False)
        r = results[0]
        boxes = getattr(r, "boxes", None)

        if boxes is not None and len(boxes) > 0:
            xyxy = boxes.xyxy[0].tolist()
            x1, y1, x2, y2 = map(int, xyxy)
            cx = (x1 + x2) // 2
            print(f"🎯 Bola detectada em cx={cx}")

            annotated = r.plot()
            cv2.imshow("Detecção", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if cx < IMG_CENTER_X - TOLERANCE:
                print("⬅️ Corrigindo esquerda")
                turn_left(speed=50)
                time.sleep(0.15)
                parar_motores()
            elif cx > IMG_CENTER_X + TOLERANCE:
                print("➡️ Corrigindo direita")
                turn_right(speed=50)
                time.sleep(0.15)
                parar_motores()
            else:
                print("⬆️ Centralizado → avançando")
                forward(speed=60)
                time.sleep(FORWARD_TIME)
                parar_motores()
        else:
            print("🔄 Nenhuma bola — novo passo de giro")

except KeyboardInterrupt:
    print("\n🛑 Interrompido pelo usuário.")
finally:
    parar_motores()
    GPIO.gpiochip_close(h)
    cap.release()
    cv2.destroyAllWindows()
    print("✔️ Finalizado com segurança.")
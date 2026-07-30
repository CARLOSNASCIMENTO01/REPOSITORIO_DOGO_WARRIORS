"""
buscar_saida.py

Versão enxuta, baseada no mesmo padrão de "buscar_bola / centralizar_bola /
aproximar_bola" do código de resgate, mas adaptada pra procurar a SAÍDA
usando um modelo YOLO próprio (treinado só pra classe "saida", ou qualquer
outra classe que você usar no dataset).

Fluxo do loop principal:
    1) buscar_saida()      -> gira no lugar, em passos curtos e devagar,
                               até o modelo detectar a saída
    2) centralizar_saida() -> corrige o giro até a saída ficar no centro do frame
    3) aproximar_saida()   -> avança em passos curtos até a bbox da saída
                               ocupar uma fração grande o suficiente do frame
                               (proxy de "já estou perto")
    4) quando chega perto, para os motores e imprime que achou a saída
       (esse é o ponto onde você pluga o que quiser fazer depois: girar
       pra dentro, chamar outro código, etc.)

Captura em resolução maior (640x480) e redimensiona pra 320x240 antes da
inferência -- esse é o tamanho que o modelo foi treinado, então mandar a
imagem em outra escala pra ele piora a detecção. O ganho de "enxergar
mais longe" vem da câmera capturar com mais nitidez antes do resize, não
de aumentar o tamanho que entra no modelo.
"""

from ultralytics import YOLO
import cv2
import lgpio as GPIO
import lgpio
import time
import threading

# ===================== BOTAO DE PARADA IMEDIATA =====================
BUTTON = 24

class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar qualquer loop na hora."""
    pass

robo_ativo = threading.Event()
robo_ativo.set()  # comeca ligado


def checar_parada():
    """Chamar a cada iteracao de todo while/for que segura o robo por mais
    de um instante (busca, centralizacao, aproximacao, etc)."""
    if not robo_ativo.is_set():
        raise RoboParado()


def botao_thread():
    """Aperta -> para tudo na hora. Aperta de novo -> retoma."""
    anterior = 0
    while True:
        estado = GPIO.gpio_read(h, BUTTON)
        if estado == 0 and anterior == 1:
            time.sleep(0.03)
            if GPIO.gpio_read(h, BUTTON) == 0:
                if robo_ativo.is_set():
                    robo_ativo.clear()
                    parar_motores()
                    print("[BOTAO] PAROU")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                while GPIO.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
        anterior = estado
        time.sleep(0.02)


# ===================== CONFIGURACOES =====================
MODEL_PATH = "onnx_saida.onnx"   # troque pro .onnx treinado pra detectar a saida
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"

# Captura em 640x480 (4:3, mesma proporção do 320x240) -- da mais nitidez
# e campo de visao do que capturar direto em 320x240, mas SEMPRE
# redimensionamos pra INFER_W x INFER_H antes de mandar pro modelo, que
# foi treinado em 320x240. Mandar imagem em outra escala pro modelo
# piora a deteccao, entao nao pule esse resize.
CAM_W, CAM_H = 640, 480
INFER_W, INFER_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = INFER_W // 2
TOLERANCE = 30

# Motores
IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13
VEL_MAX = 50
VEL_MIN = -VEL_MAX

# Busca (giro procurando a saida) -- passos mais curtos e mais lentos,
# entao gira menos por vez e processa mais vezes por volta completa
BUSCA_MAX_GIROS = 120
TURN_PULSE = 0.2
BUSCA_TURN_SPEED = 45

# Centralizacao proporcional
CENTER_TURN_SPEED_MIN = 30
CENTER_TURN_SPEED_MAX = 42
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 8

# Aproximacao por tamanho de bbox
APPROACH_SPEED = 45
APPROACH_STEP_TIME = 0.15
STOP_AREA_RATIO = 0.35     # bbox_area / crop_area -> ajustar na pratica!
MAX_APPROACH_STEPS = 60
LOST_FRAMES = 8

h = lgpio.gpiochip_open(0)

pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for p in pins:
    GPIO.gpio_claim_output(h, p)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# ===================== MOTORES =====================
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


def turn_left(speed=30):
    mover(-speed, speed)


def turn_right(speed=30):
    mover(speed, -speed)


# ===================== CAMERA (frame cheio, redimensionado pro treino) =====================
def capturar_frame():
    """
    Captura em CAM_W x CAM_H (mais nitido/mais campo de visao) e
    redimensiona pra INFER_W x INFER_H (320x240), que e o tamanho que o
    modelo foi treinado. Sem esse resize a deteccao piora, mesmo com
    imagem "maior".
    """
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    return cv2.resize(frame, (INFER_W, INFER_H), interpolation=cv2.INTER_AREA)


def escolher_indice_mais_perto(boxes):
    """Se detectar mais de uma 'saida' no frame, fica com a de maior bbox
    (a mais perto da camera)."""
    if boxes is None or len(boxes) == 0:
        return None
    maior_area = -1
    melhor_idx = 0
    for i in range(len(boxes)):
        x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
        area = (x2 - x1) * (y2 - y1)
        if area > maior_area:
            maior_area = area
            melhor_idx = i
    return melhor_idx


# ===================== DETECCAO DA SAIDA (YOLO) =====================
def detectar_saida(frame):
    """
    Roda o modelo no frame ja redimensionado pra 320x240 (tamanho de
    treino) e devolve (cx, area, bbox, cls_id) da deteccao de saida mais
    proxima (maior bbox), ou (None, None, None, None) se nao achou nada.
    """
    results = model.predict(
        frame, imgsz=(INFER_W, INFER_H), conf=CONF_THRES, verbose=False, show=False
    )
    r = results[0]
    boxes = getattr(r, "boxes", None)

    annotated = r.plot()
    cv2.imshow("Deteccao - Saida", annotated)
    cv2.waitKey(1)

    if boxes is not None and len(boxes) > 0:
        idx = escolher_indice_mais_perto(boxes)
        x1, y1, x2, y2 = map(int, boxes.xyxy[idx].tolist())
        cx = (x1 + x2) // 2
        area = (x2 - x1) * (y2 - y1)
        cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
        return cx, area, (x1, y1, x2, y2), cls_id

    return None, None, None, None


def buscar_saida():
    """
    Gira no lugar ate o modelo detectar a saida. Passos curtos e devagar
    (BUSCA_TURN_SPEED baixo + TURN_PULSE curto), entao gira pouco por vez
    e processa um frame a cada passo -- cobre a volta toda com mais
    chances de detectar algo que passa rapido no campo de visao.
    Retorna True se achou.
    """
    print("Procurando a saida...")
    for _ in range(BUSCA_MAX_GIROS):
        checar_parada()
        turn_right(speed=BUSCA_TURN_SPEED)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.25)

        frame = capturar_frame()
        if frame is None:
            continue

        cx, area, bbox, cls_id = detectar_saida(frame)
        if cx is not None:
            print(f"Saida encontrada (cx={cx}, area={area})")
            return True

    print("Estourou o limite de giros procurando a saida")
    return False


def centralizar_saida():
    """
    Gira o robo ate a saida cair dentro de TOLERANCE do centro da imagem.
    Velocidade/duracao do pulso escalam com o erro (proporcional).
    Retorna True se centralizou, False se perdeu a saida de vista.
    """
    perdida = 0
    for _ in range(CENTER_MAX_TENTATIVAS):
        checar_parada()
        frame = capturar_frame()
        if frame is None:
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_saida(frame)

        if cx is None:
            perdida += 1
            if perdida >= CENTER_LOST_FRAMES:
                print("Perdeu a saida durante centralizacao")
                parar_motores()
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"Centralizado na saida (cx={cx})")
            parar_motores()
            return True

        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN)
        pulso = 0.05 + erro_norm * 0.10

        if erro < 0:
            print(f"Corrigindo esquerda (erro={erro}, speed={speed:.0f})")
            turn_left(speed=speed)
        else:
            print(f"Corrigindo direita (erro={erro}, speed={speed:.0f})")
            turn_right(speed=speed)

        time.sleep(pulso)
        parar_motores()
        time.sleep(0.05)

    print("Estourou tentativas de centralizacao na saida")
    parar_motores()
    return False


def aproximar_saida():
    """
    Avanca em passos curtos ate a saida ocupar uma fracao grande o
    suficiente da imagem (proxy de distancia real). Se sair do centro
    no meio do caminho, recentraliza antes de continuar. Retorna True
    se chegou perto o suficiente, False se perdeu a saida ou estourou
    o limite de passos.
    """
    frame_area = INFER_W * INFER_H
    perdida = 0

    for passo in range(MAX_APPROACH_STEPS):
        checar_parada()
        frame = capturar_frame()
        if frame is None:
            print("Erro na camera durante aproximacao")
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_saida(frame)

        if cx is None:
            perdida += 1
            print(f"Saida sumiu durante aproximacao ({perdida}/{LOST_FRAMES})")
            if perdida >= LOST_FRAMES:
                print("Perdeu a saida de vez, abortando aproximacao")
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / frame_area
        print(f"passo={passo} area_ratio={ratio:.2f} cx={cx}")

        if ratio >= STOP_AREA_RATIO:
            print("Perto o suficiente da saida, parando")
            parar_motores()
            return True

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximacao, recentralizando")
            if not centralizar_saida():
                print("Perdeu a saida tentando recentralizar na aproximacao")
                return False
            continue

        forward(speed=APPROACH_SPEED)
        time.sleep(APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximacao da saida")
    parar_motores()
    return False


# ===================== SETUP CAMERA E MODELO =====================
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

if not cap.isOpened():
    print(f"Nao consegui abrir a camera em {CAMERA_INDEX}")
    exit(1)

model = YOLO(MODEL_PATH)
print("Classes do modelo:", model.names)

cv2.namedWindow("Deteccao - Saida", cv2.WINDOW_NORMAL)

threading.Thread(target=botao_thread, daemon=True).start()


# ===================== LOOP PRINCIPAL =====================
try:
    while True:
        try:
            checar_parada()

            if not buscar_saida():
                # nao achou girando -> reposiciona um pouco e tenta de novo
                mover(-25, -25)
                time.sleep(0.4)
                mover(-30, 0)
                time.sleep(0.2)
                continue

            if not centralizar_saida():
                print("Nao conseguiu centralizar na saida, voltando pra busca")
                continue

            if not aproximar_saida():
                print("Nao conseguiu se aproximar da saida, voltando pra busca")
                continue

            print("Chegou na saida!")
            parar_motores()
            # >>> aqui é onde você pluga o que quiser fazer ao chegar na
            # saida (girar pra dentro, chamar outro script, break, etc.)
            break

        except RoboParado:
            print("Robo parado pelo botao. Aguardando reinicio...")
            parar_motores()
            while not robo_ativo.is_set():
                time.sleep(0.1)
            print("Retomando busca pela saida...")

except KeyboardInterrupt:
    print("\nInterrompido pelo usuario.")
finally:
    parar_motores()
    GPIO.gpiochip_close(h)
    cap.release()
    cv2.destroyAllWindows()
    print("Finalizado com seguranca.")
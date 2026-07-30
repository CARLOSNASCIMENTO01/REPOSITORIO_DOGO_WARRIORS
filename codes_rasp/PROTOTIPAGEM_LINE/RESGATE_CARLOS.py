from ultralytics import YOLO
import cv2
import lgpio as GPIO
import time
import lgpio
import numpy as np
import sys
import os
import threading
import smbus2
import serial
import math
import serial.tools.list_ports
from threading import Thread

# ===================== BOTAO DE PARADA IMEDIATA (mesmo padrao do seguidor_debug2.py) =====================
BUTTON = 24  # mesmo pino usado no codigo de linha

class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar QUALQUER loop na hora."""
    pass

robo_ativo = threading.Event()
robo_ativo.set()  # comeca ligado

def checar_parada():
    """Chamar a cada iteracao de todo while/for que segura o robo por mais
    de um instante (buscas, centralizacao, aproximacao, entrega, etc)."""
    if not robo_ativo.is_set():
        raise RoboParado()

def botao_thread():
    """Toggle com debounce: aperta -> para tudo na hora (mesmo no meio de
    uma busca/aproximacao/entrega). Aperta de novo -> deixa o robo continuar
    (retoma a busca por bolas do zero)."""
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


# =========================================================================
# SERIAL / ULTRASSONICO - portado do seguidor_debug2.py (so o sonar, F e E)
# =========================================================================
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"
SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0  # segundos sem dado novo pra forcar reconexao da porta

dados = {"F": 9999.0, "E": 9999.0}
ultimo_recebimento = time.time()
ser = None
_serial_thread_iniciada = [False]


def detectar_porta(sair_se_falhar=True):
    try:
        s = serial.Serial(PORTA_FIXA, 115200, timeout=0.04)
        print(f"[Serial] Conectado via ID fixo: {PORTA_FIXA}")
        return s
    except serial.SerialException:
        pass

    PALAVRAS_FTDI = ["ftdi", "ft232", "usb serial converter", "ftb6spl3"]
    portas = serial.tools.list_ports.comports()

    for p in portas:
        desc = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".lower()
        if any(palavra in desc for palavra in PALAVRAS_FTDI):
            try:
                s = serial.Serial(p.device, 115200, timeout=0.04)
                print(f"[Serial] FTDI encontrado em: {p.device} - {p.description}")
                return s
            except serial.SerialException:
                continue

    for p in portas:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            try:
                s = serial.Serial(p.device, 115200, timeout=0.04)
                print(f"[Serial] Usando porta generica: {p.device} - {p.description}")
                return s
            except serial.SerialException:
                continue

    if sair_se_falhar:
        print("\n[ERRO] Nenhuma porta serial encontrada!")
        sys.exit(1)
    return None


def aguardar_porta(intervalo=1.0):
    porta = detectar_porta(sair_se_falhar=False)
    while porta is None:
        print("[Serial] Arduino ainda nao apareceu, tentando de novo...")
        time.sleep(intervalo)
        porta = detectar_porta(sair_se_falhar=False)
    return porta


def handshake_serial():
    global ser, ultimo_recebimento
    ser = aguardar_porta()
    time.sleep(2.0)
    ser.reset_input_buffer()
    ser.write(b"START\n")
    time.sleep(2.0)
    ser.reset_input_buffer()
    ultimo_recebimento = time.time()
    print("[Serial] Handshake enviado pro Arduino")


def serial_thread():
    global ultimo_recebimento, ser
    while True:
        try:
            if ser is None or not ser.is_open:
                print("[SERIAL] Porta indisponivel, tentando reconectar...")
                novo = detectar_porta(sair_se_falhar=False)
                if novo is not None:
                    ser = novo
                    try:
                        ser.reset_input_buffer()
                    except Exception:
                        pass
                    ultimo_recebimento = time.time()
                    print("[SERIAL] Reconectado!")
                else:
                    time.sleep(1.0)
                continue

            if time.time() - ultimo_recebimento > WATCHDOG_RECONEXAO:
                print(f"[SERIAL] Sem dado novo ha mais de {WATCHDOG_RECONEXAO:.0f}s, forcando reconexao...")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                continue

            linha = None
            while ser.in_waiting:
                bruta = ser.readline().decode(errors="ignore").strip()
                if bruta:
                    linha = bruta

            if linha is None:
                time.sleep(0.005)
                continue

            ultimo_recebimento = time.time()
            for parte in linha.split(","):
                if ":" not in parte:
                    continue
                chave, valor = parte.split(":", 1)
                if chave in dados:
                    try:
                        dados[chave] = float(valor)
                    except ValueError:
                        pass

        except serial.SerialException as e:
            print("[SERIAL] erro de conexao:", e)
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1.0)

        except Exception as e:
            print("[THREAD SERIAL] erro inesperado:", e)
            time.sleep(0.1)


def distancia_frente():
    return dados["F"]


def distancia_esquerda():
    return dados["E"]


def sonar_valido():
    return (time.time() - ultimo_recebimento) < SERIAL_TIMEOUT_DADO


def iniciar_serial():
    """Sobe a thread do sonar e faz o handshake, uma vez so, em background."""
    if _serial_thread_iniciada[0]:
        return
    _serial_thread_iniciada[0] = True

    def _start():
        handshake_serial()
        serial_thread()

    threading.Thread(target=_start, daemon=True).start()


# ===================== DETECÇÃO DE TRIÂNGULO (área de entrega) =====================
ALVO_COR = None  # None = aceita verde OU vermelho; setado dinamicamente em entregar_bola()
TIPO_BOLA = None

# ===================== MAPEAMENTO CLASSE DA BOLA -> COR DA ÁREA DE ENTREGA =====================
# Mapeado pelo NOME da classe (não pelo índice), então não importa a ordem
# que o YOLO carregar as classes internamente.
NOME_COR_ENTREGA = {
    "bola-preta": "vermelho",
    "bola-prata": "verde",
}
# Preenchido automaticamente depois que o modelo é carregado (ver mais
# abaixo, logo após `model = YOLO(MODEL_PATH)`): vira {cls_id: cor}.
CLASSE_COR_ENTREGA = {}

green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])

red_min1 = np.array([0, 100, 100])
red_max1 = np.array([10, 255, 255])
red_min2 = np.array([170, 100, 100])
red_max2 = np.array([180, 255, 255])

kernel_tri = np.ones((3, 3), np.uint8)

PROPORCAO_ESPERADA = 30 / 6
TOLERANCIA_PROPORCAO = 0.8
AREA_MIN_TRI = 500
AREA_MAX_TRI = 1000000

STOP_AREA_RATIO_TRIANGULO = 0.80   # ajustar na prática, igual fizemos com a bola
TRIANGULO_MAX_GIROS = 90
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_APPROACH_SPEED = 40
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 25
TRIANGULO_CENTER_LOST_FRAMES = 15
# ===================== CONFIGURAÇÕES =====================
MODEL_PATH = "model.onnx"
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13

VEL_MAX = 50
VEL_MIN = -VEL_MAX
# ===================== APROXIMAÇÃO POR TAMANHO DE BBOX =====================
APPROACH_SPEED = 45        # velocidade do avanço durante a aproximação
APPROACH_STEP_TIME = 0.15  # duração de cada "passo" de avanço/correção
STOP_AREA_RATIO = 0.35     # bbox_area / crop_area -> ajustar na prática!
MAX_APPROACH_STEPS = 60    # trava de segurança (evita loop infinito)
LOST_BALL_FRAMES = 8       # quantos frames sem detectar até desistir
TURN_PULSE = 0.2
STOP_TIME = 0.2
FORWARD_TIME = 0.6

# ===================== CENTRALIZAÇÃO PROPORCIONAL =====================
CENTER_TURN_SPEED_MIN = 30   # giro mais fraco quando o erro é pequeno
CENTER_TURN_SPEED_MAX = 42   # giro mais forte quando o erro é grande
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5


MAX_ENTREGAS = 3
BUSCA_FINAL_MAX_GIROS = 20  # ajustar na prática

# ===================== CÂMERA DE BAIXO (linha/chão) - usada só na saída =====================
# Camera separada, apontada pro chao, so pra enxergar a linha preta/contorno
# preto durante o acionar_saida(). Ajuste o path/indice pro seu setup.
CAMERA_INDEX_LINHA = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010002-video-index0"
USAR_TCRT_PARA_LINHA = False
CAMERA_LINHA_INDEX_FALLBACK = 0
LINHA_W, LINHA_H = 160, 120
LIMIAR_LINHA = 80
AREA_MIN_LINHA = 11000
kernel_linha = np.ones((3, 3), np.uint8)

# ===================== LIMIARES DA SAÍDA (acionar_saida) =====================

DIST_LATERAL_ABERTA_CM = 40   # muita distancia no ultrassonico lateral = campo aberto
DIST_PAREDE_FRENTE_CM = 10    # pouca distancia no ultrassonico da frente = parede
DEBUG_VISUAL = False

BUSCA_MAX_GIROS_SAIDA = 120
TURN_PULSE_SAIDA = 0.15
BUSCA_TURN_SPEED_SAIDA= 35
# Centralizacao proporcional
CENTER_TURN_SPEED_MIN_SAIDA = 30
CENTER_TURN_SPEED_MAX_SAIDA = 42
CENTER_MAX_TENTATIVAS_SAIDA = 15
CENTER_LOST_FRAMES_SAIDA = 8

# Aproximacao por tamanho de bbox
APPROACH_SPEED_SAIDA = 45
APPROACH_STEP_TIME_SAIDA = 0.15
STOP_AREA_RATIO_SAIDA = 0.35     # bbox_area / crop_area -> ajustar na pratica!
MAX_APPROACH_STEPS_SAIDA = 60
LOST_FRAMES_SAIDA = 8

# Pinos
PINO_SG90 = 5
PINO_PESADO = 20
TCRT = 6  # GPIO BCM 5

CAM_W = 640
CAM_H = 480

INFER_W = 320
INFER_H = 240

ANGULO_INICIAL = 90

_gyro_lock = threading.Lock()
MPU_ADDR    = 0x68
GYRO_XOUT_H = 0x43
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)
GPIO.gpio_claim_input(h, TCRT)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL,
    PINO_PESADO: ANGULO_INICIAL,
}

DELAY_BASE = 0.03


# ===================== SETUP GPIO (lgpio) =====================
pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for p in pins:
    GPIO.gpio_claim_output(h, p)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# ===================== MOTORES (lgpio) =====================
def capturar_frame_saida():
    for _ in range(6):
        cap.read()

    ret, frame = cap.read()

    if not ret:
        return None

    return cv2.resize(
        frame,
        (INFER_W, INFER_H),
        interpolation=cv2.INTER_AREA
    )


def sensor_e():
    return GPIO.gpio_read(h, TCRT) 
LEITURAS_CONFIRMACAO_TCRT = 3

def linha_detectada_tcrt():
    """
    sensor_e() le o GPIO cru (0 ou 1). Ajuste o "== 0" abaixo se no seu
    modulo a logica for invertida (a maioria dos modulos com LM393 da
    LOW quando esta em cima de superficie escura).
    Exige leituras seguidas iguais pra nao disparar por ruido/reflexo.
    """
    for _ in range(LEITURAS_CONFIRMACAO_TCRT):
        checar_parada()
        if sensor_e() != 0:
            return False
        time.sleep(0.01)
    return True
def _mpu_write(reg, val):
    bus.write_byte_data(MPU_ADDR, reg, val)

def _mpu_read_word(reg, tentativas=3):
    for tentativa in range(tentativas):
        try:
            hi = bus.read_byte_data(MPU_ADDR, reg)
            lo = bus.read_byte_data(MPU_ADDR, reg + 1)

            valor = (hi << 8) | lo

            if valor >= 0x8000:
                valor -= 65536

            return valor

        except OSError as e:
            # Remote I/O error (121), muito comum em impactos
            if tentativa == tentativas - 1:
                raise
            time.sleep(0.001)

def _normalizar(ang):
    while ang > 180: ang -= 360
    while ang < -180: ang += 360
    return ang

def _calibrar(amostras=200):
    gx = gy = gz = 0
    for _ in range(amostras):
        gx += _mpu_read_word(GYRO_XOUT_H)
        gy += _mpu_read_word(GYRO_XOUT_H + 2)
        gz += _mpu_read_word(GYRO_XOUT_H + 4)
        time.sleep(0.005)
    gyro_offset["X"] = gx / amostras
    gyro_offset["Y"] = gy / amostras
    gyro_offset["Z"] = gz / amostras

def resetar_giroscopio():
    global ang_gyro, gyro_ultimo_tempo
    with _gyro_lock:
        ang_gyro = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        gyro_ultimo_tempo = time.time()

def angulo_x(): return int(ang_gyro["X"])
def angulo_y(): return angulo_y_estavel()
def angulo_z(): return int(ang_gyro["Z"])

def _gyro_thread():
    global ang_gyro, gyro_ultimo_tempo
    while True:
        try:
            now = time.time()
            with _gyro_lock:
                dt = now - gyro_ultimo_tempo
                gyro_ultimo_tempo = now
            if dt > 0.5:
                time.sleep(0.005)
                continue
            gx = (_mpu_read_word(GYRO_XOUT_H)     - gyro_offset["X"]) / 131.0
            gy = (_mpu_read_word(GYRO_XOUT_H + 2) - gyro_offset["Y"]) / 131.0
            gz = (_mpu_read_word(GYRO_XOUT_H + 4) - gyro_offset["Z"]) / 131.0
            with _gyro_lock:
                ang_gyro["X"] = _normalizar(ang_gyro["X"] + gx * dt)
                ang_gyro["Y"] = _normalizar(ang_gyro["Y"] + gy * dt)
                ang_gyro["Z"] = _normalizar(ang_gyro["Z"] + gz * dt)
        except OSError as e:
            print(f"[MPU] Gyro I2C: {e}")
            time.sleep(0.005)

        except Exception as e:
            print(f"[MPU] Gyro: {e}")
            time.sleep(0.005)

def iniciar_mpu():
    _mpu_write(0x6B, 0x00)
    time.sleep(0.05)
    _mpu_write(0x1B, 0x00)
    time.sleep(0.05)
    print("[MPU6050] Calibrando...")
    _calibrar()
    resetar_giroscopio()
    print("[MPU6050] Pronto")
    Thread(target=_gyro_thread, daemon=True).start()

_mpu_thread_iniciada = [False]

def iniciar_mpu_thread():
    """Sobe a calibração + thread de leitura do giroscópio, uma vez só, sob demanda."""
    if _mpu_thread_iniciada[0]:
        return
    _mpu_thread_iniciada[0] = True
    Thread(target=iniciar_mpu, daemon=True).start()    

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

_angulo_y_filtrado = None

def angulo_y_estavel(alpha=0.97):
    global _angulo_y_filtrado

    try:
        ax = _mpu_read_word(ACCEL_XOUT_H) / 16384.0
        ay = _mpu_read_word(ACCEL_YOUT_H) / 16384.0
        az = _mpu_read_word(ACCEL_ZOUT_H) / 16384.0

        angulo = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        if _angulo_y_filtrado is None:
            _angulo_y_filtrado = angulo
        else:
            _angulo_y_filtrado = (
                alpha * _angulo_y_filtrado
                + (1 - alpha) * angulo
            )

    except OSError as e:
        print(f"[MPU] Falha I2C ({e}), mantendo último valor.")
        # NÃO derruba o programa

    if _angulo_y_filtrado is None:
        return 0

    return int(_angulo_y_filtrado)


def guinada_pid(LADO, GRAUS, VEL_MAX_GIRO=100, VEL_MIN_GIRO=29, ZONA_CHEGADA=10):
    parar_motores()
    Kp_g, Ki_g, Kd_g = 0.09, 0.002, 0.001
    resetar_giroscopio()
    time.sleep(0.03)

    alvo = abs(GRAUS)
    integral = 0.0
    erro_ant = alvo
    t_ant = time.time()

    while True:
        checar_parada()
        atual = abs(angulo_z())
        erro = alvo - atual
        agora = time.time()
        dt = max(agora - t_ant, 0.001)

        if erro <= ZONA_CHEGADA:
            if LADO == 'D':
                mover(-15, 15)
            else:
                mover(15, -15)
            time.sleep(0.09)
            parar_motores()
            break

        integral += erro * dt
        integral = np.clip(integral, -20, 20)
        derivada = (erro - erro_ant) / dt
        derivada = np.clip(derivada, -300, 300)

        ajuste = Kp_g * erro + Ki_g * integral - Kd_g * derivada
        ajuste = np.clip(ajuste, 0, VEL_MAX_GIRO - VEL_MIN_GIRO)
        vel = int(VEL_MIN_GIRO + ajuste)

        if LADO == 'D':
            mover(vel, -vel)
        else:
            mover(-vel, vel)

        erro_ant = erro
        t_ant = agora
        time.sleep(0.005)


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

def capturar_frame_saida():
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

def iniciar_modo_saida():
    global model, cap
    checar_parada()
    print("===== MODO SAÍDA =====")

    # fecha câmera usada para procurar bolas
    cap.release()

    # abre novamente em resolução maior
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Erro abrindo câmera para saída")
        return False

    # troca o modelo
    model = YOLO("onnx_saida.onnx")

    cv2.destroyWindow("Detecção")
    cv2.namedWindow("Deteccao - Saida", cv2.WINDOW_NORMAL)

    print("Modelo de saída carregado.")
    return True   

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
    checar_parada()
    print("Procurando a saida...")
    for _ in range(BUSCA_MAX_GIROS_SAIDA):
        checar_parada()
        turn_right(speed=BUSCA_TURN_SPEED_SAIDA)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.25)

        frame = capturar_frame_saida()
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
    checar_parada()
    perdida = 0
    for _ in range(CENTER_MAX_TENTATIVAS_SAIDA):
        checar_parada()
        frame = capturar_frame_saida()
        if frame is None:
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_saida(frame)

        if cx is None:
            perdida += 1
            if perdida >= CENTER_LOST_FRAMES_SAIDA:
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
        speed = CENTER_TURN_SPEED_MIN_SAIDA + erro_norm * (CENTER_TURN_SPEED_MAX_SAIDA - CENTER_TURN_SPEED_MIN_SAIDA)
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
    checar_parada()
    frame_area = INFER_W * INFER_H
    perdida = 0

    for passo in range(MAX_APPROACH_STEPS_SAIDA):
        checar_parada()
        frame = capturar_frame_saida()
        if frame is None:
            print("Erro na camera durante aproximacao")
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_saida(frame)

        if cx is None:
            perdida += 1
            print(f"Saida sumiu durante aproximacao ({perdida}/{LOST_FRAMES_SAIDA})")
            if perdida >= LOST_FRAMES_SAIDA:
                print("Perdeu a saida de vez, abortando aproximacao")
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME_SAIDA)
            continue

        perdida = 0
        ratio = area / frame_area
        print(f"passo={passo} area_ratio={ratio:.2f} cx={cx}")

        if ratio >= STOP_AREA_RATIO_SAIDA:
            print("Perto o suficiente da saida, parando")
            parar_motores()
            return True

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximacao, recentralizando")
            if not centralizar_saida():
                print("Perdeu a saida tentando recentralizar na aproximacao")
                return False
            continue

        forward(speed=APPROACH_SPEED_SAIDA)
        time.sleep(APPROACH_STEP_TIME_SAIDA)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximacao da saida")
    parar_motores()
    return False 

def oscilar_servo_lgpio(pino, angulo_a, angulo_b, repeticoes=5, delay=0.15):
    """
    Oscila servo rapidamente usando diretamente lgpio.tx_servo().
    Não usa mover_servo() nem parar_servo().
    """

    pulso_min = 500
    pulso_max = 2500

    def angulo_para_pulso(angulo):
        angulo = max(0, min(180, angulo))
        return int(
            pulso_min +
            (angulo / 180.0) * (pulso_max - pulso_min)
        )

    pulso_a = angulo_para_pulso(angulo_a)
    pulso_b = angulo_para_pulso(angulo_b)

    print(f"Oscilando servo GPIO {pino}: {angulo_a}({pulso_a}) <-> {angulo_b}({pulso_b})")

    try:
        for _ in range(repeticoes):
            checar_parada()

            # vai para posição A
            lgpio.tx_servo(h, pino, pulso_a)
            time.sleep(delay)

            # vai para posição B
            lgpio.tx_servo(h, pino, pulso_b)
            time.sleep(delay)

    except lgpio.error as e:
        print("ERRO OSCILA SERVO:", e)

    finally:
        # mantém o último pulso por um pouco
        # não corta PWM imediatamente
        time.sleep(0.2)

def buscar_bola():
    """
    Gira no lugar até achar qualquer bola (usado só na busca final,
    depois da 3ª entrega). Retorna True se achou.
    """
    for _ in range(BUSCA_FINAL_MAX_GIROS):
        checar_parada()
        turn_right(speed=55)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.25)

        crop = capturar_crop()
        if crop is None:
            continue

        cx, area, bbox, cls_id = detectar_bola(crop)
        if cx is not None:
            return True
    return False


def guinada2(LADO, GRAUS, VELOCIDADE):
    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    if LADO == 'D':
        while True:
            checar_parada()
            mover(VELOCIDADE, -VELOCIDADE)
            if angulo_z() <= -Graus:
                mover(-10, 10)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    else:
        while True:
            checar_parada()
            mover(-VELOCIDADE, VELOCIDADE)
            if angulo_z() >= Graus:
                mover(10, -10)
                time.sleep(0.09)
                break
            time.sleep(0.005)


def detectar_linha_embaixo(frame):
    """
    Mesma logica de deteccao de linha do seguidor (threshold + contorno
    preto), so que aplicada na camera de baixo. Retorna True se achou um
    contorno preto grande o suficiente pra ser considerado "a linha".
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    maior = max(contours, key=cv2.contourArea)
    return cv2.contourArea(maior) >= AREA_MIN_LINHA


def saida():
    checar_parada()
    print("[SAIDA] saindo")
    mover(30,30)
    time.sleep(1.0)
    print("Saída encontrada!")
    parar_motores()
    cap.release()
    if cap_linha is not None:
        cap_linha.release()
    GPIO.gpiochip_close(h)
    os.execv(sys.executable, [
        sys.executable,
        "/home/pi/ROBO_DOGO_2026/codes_rasp/PROTOTIPAGEM_LINE/seguidor_debug2.py",
        "--auto",
        f"--entregas={ENTREGAS_HERDADAS}",
    ])
         

def _mascara_cor_tri(hsv, cor):
    if cor == "verde":
        mask = cv2.inRange(hsv, green_min, green_max)
    elif cor == "vermelho":
        m1 = cv2.inRange(hsv, red_min1, red_max1)
        m2 = cv2.inRange(hsv, red_min2, red_max2)
        mask = cv2.bitwise_or(m1, m2)
    else:
        raise ValueError("cor deve ser 'verde' ou 'vermelho'")
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_tri, iterations=2)
    mask = cv2.dilate(mask, kernel_tri, iterations=2)
    return mask

def desenhar_deteccoes_tri(frame, deteccoes):
    saida = frame.copy()
    for d in deteccoes:
        cor_bgr = (0, 255, 0) if d["cor"] == "verde" else (0, 0, 255)
        if "vertices" in d:
            cv2.drawContours(saida, [d["vertices"]], -1, cor_bgr, 2)
        cv2.circle(saida, d["centro"], 4, cor_bgr, -1)
        texto = f'{d["cor"]} :1 area={d["area"]:.0f}'
        cv2.putText(saida, texto, (d["centro"][0] - 20, d["centro"][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, cor_bgr, 1)
    return saida

def _eh_triangulo(contorno, encostando_borda=False):
    print("ENTROU _eh_triangulo")
    area = cv2.contourArea(contorno)
    if area < AREA_MIN_TRI or area > AREA_MAX_TRI:
        print("REJEITOU AREA")
        return False, None

    perimetro = cv2.arcLength(contorno, True)
    aproximado = cv2.approxPolyDP(contorno, 0.10 * perimetro, True)

    print("Vertices internos:", len(aproximado))

    # Se o contorno está encostando na borda do crop, o polígono real
    # foi cortado pela imagem -- é normal ter menos vértices quando
    # estamos muito perto. Nesse caso aceitamos a partir de 2 vértices.
    min_vertices = 2 if encostando_borda else 3
    if len(aproximado) < min_vertices or len(aproximado) > 4:
        print("REJEITOU VERTICES")
        return False, None

    # Convexidade só faz sentido checar quando o polígono não foi
    # cortado pela borda (senão vira côncavo naturalmente e não é bug).
    if not encostando_borda and not cv2.isContourConvex(aproximado):
        print("REJEITOU CONVEX")
        return False, None

    print("ACEITOU")

    # Centroide real da área visível (média dos pixels do contorno),
    # em vez do centro do bounding rect -- evita que o centro fique
    # puxado pra ponta quando a câmera só enxerga parte do retângulo.
    M = cv2.moments(contorno)
    if M["m00"] == 0:
        return False, None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]

    return True, {
        "centro": (int(cx), int(cy)),
        "area": area,
        "cor": None,
        "vertices": aproximado.reshape(-1, 2),
    }

def detectar_entrega(crop):
    """
    Procura triângulos (cantos de parede) no crop. Se ALVO_COR estiver
    definido, só aceita essa cor; senão aceita verde ou vermelho e
    devolve o de MAIOR área (mais perto). Retorna (cx, area, info) ou
    (None, None, None). Sempre mostra as janelas de debug (normal +
    máscara verde + máscara vermelho).
    """
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    cores = [ALVO_COR] if ALVO_COR else ["verde", "vermelho"]

    todas_deteccoes = []
    mask_verde = None
    mask_vermelho = None

    for cor in cores:
        mask = _mascara_cor_tri(hsv, cor)
        if cor == "verde":
            mask_verde = mask
        else:
            mask_vermelho = mask

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Quantidade de contornos: {len(contornos)}")
        for c in contornos:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            print(f"Area={area:.1f}  Vertices={len(approx)}")

            x, y, w, h_bb = cv2.boundingRect(c)
            margem = 2
            encostando_borda = (
                x <= margem or y <= margem or
                x + w >= crop.shape[1] - margem or
                y + h_bb >= crop.shape[0] - margem
            )

            print("Chamando _eh_triangulo")
            ok, info = _eh_triangulo(c, encostando_borda=encostando_borda)
            print("Resultado:", ok)
            if ok:
                info["cor"] = cor
                todas_deteccoes.append(info)

    # ---- debug visual ----
    if DEBUG_VISUAL:
        debug = desenhar_deteccoes_tri(crop, todas_deteccoes)
        cv2.imshow("Entrega - Normal", debug)
        if mask_verde is not None:
            cv2.imshow("Entrega - Mascara Verde", mask_verde)
        if mask_vermelho is not None:
            cv2.imshow("Entrega - Mascara Vermelho", mask_vermelho)
        cv2.waitKey(1)

    if not todas_deteccoes:
        return None, None, None

    melhor = max(todas_deteccoes, key=lambda d: d["area"])
    print(f"Detecções encontradas: {len(todas_deteccoes)}")
    for d in todas_deteccoes:
        print(d["area"], d["centro"])
    return melhor["centro"][0], melhor["area"], melhor

def centralizar_triangulo():
    """Igual centralizar_bola(), mas usando detectar_entrega()."""
    print(f"[centralizar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    perdida = 0
    for _ in range(TRIANGULO_CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, info = detectar_entrega(crop)

        print("cx =", cx, "| cor detectada =", info["cor"] if info else None)
        print("area =", area)

        if cx is None:
            perdida += 1
            if perdida >= TRIANGULO_CENTER_LOST_FRAMES:
                print("Perdeu o triângulo durante centralização")
                parar_motores()
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"Centralizado no triângulo (cx={cx})")
            parar_motores()
            return True

        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN)
        pulso = 0.05 + erro_norm * 0.10

        if erro < 0:
            turn_left(speed=speed)
        else:
            turn_right(speed=speed)

        time.sleep(pulso)
        parar_motores()
        time.sleep(0.05)

    print("Estourou tentativas de centralização no triângulo")
    parar_motores()
    return False


def aproximar_triangulo():
    """Igual aproximar_bola(), mas até chegar perto do triângulo pra soltar a bola."""
    print(f"[aproximar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    crop_area = CROP_W * CROP_H
    perdida = 0

    for passo in range(TRIANGULO_MAX_APPROACH_STEPS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, info = detectar_entrega(crop)

        if cx is None:
            perdida += 1
            print(f"Triângulo sumiu durante aproximação ({perdida}/{TRIANGULO_LOST_FRAMES})")
            if perdida >= TRIANGULO_LOST_FRAMES:
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"[entrega] passo={passo} area_ratio={ratio:.2f} cx={cx} cor={info['cor']}")

        if ratio >= STOP_AREA_RATIO_TRIANGULO:
            print("Perto o suficiente do triângulo, parando")
            parar_motores()
            return True

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximação da entrega, recentralizando")
            if not centralizar_triangulo():
                return False
            continue

        forward(speed=TRIANGULO_APPROACH_SPEED)
        time.sleep(TRIANGULO_APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximação da entrega")
    parar_motores()
    return False

def buscar_triangulo():
    """Gira no lugar até achar um triângulo. Retorna True se achou."""
    print(f"🔎 [buscar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    for _ in range(TRIANGULO_MAX_GIROS):
        checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, area, info = detectar_entrega(crop)
            if cx is not None:
                return True
        turn_right(speed=45)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.3)
    return False


def entregar_bola(cor_alvo=None):
    """
    Sequência completa pós-captura: busca o triângulo (canto de parede)
    da COR CERTA para a bola que está sendo carregada, centraliza,
    aproxima e solta a bola oscilando a garra pesada.

    cor_alvo: "vermelho" ou "verde". Se None, aceita qualquer triângulo
    (comportamento antigo, sem diferenciação por classe).
    """
    global ALVO_COR
    ALVO_COR = cor_alvo

    checar_parada()
    parar_motores()
    time.sleep(0.5)

    for _ in range(8):
        cap.read()

    if cor_alvo:
        print(f"Procurando área de entrega {cor_alvo.upper()}...")
    else:
        print("Procurando área de entrega (qualquer cor)...")

    while True:

        if not buscar_triangulo():
            print("Não encontrou triângulo, reposicionando...")
            mover(-25, -25)
            time.sleep(0.4)
            mover(-30,0)
            time.sleep(0.2)
            continue

        if not centralizar_triangulo():
            print("Não conseguiu centralizar, reposicionando...")
            mover(-25, -25)
            time.sleep(0.4)
            mover(-30,0)
            time.sleep(0.2)
            continue

        if not aproximar_triangulo():
            print("Não conseguiu aproximar, reposicionando...")
            mover(-25, -25)
            time.sleep(0.4)
            mover(-30,0)
            time.sleep(0.2)
            continue

        # conseguiu chegar no triângulo
        break

    print("Soltando a bola...")

    # Recentraliza uma última vez, já perto, pra garantir que a bola
    # caia no MEIO do retângulo e não na ponta.

    checar_parada()
    verificar_e_afastar_bola_bloqueio()   # <-- checa e afasta bola do canto, se houver

    checar_parada()
    print("📤 Soltando a bola...")
    mover(40,40)
    time.sleep(1.5)
    parar_motores()          # <-- adiciona os parênteses
    angulo(170)
    mover_servo(PINO_PESADO, 20, velocidade=20)
    oscilar_servo_lgpio(PINO_PESADO, 30, 60,13,0.2)
   
    mover_servo(PINO_PESADO, 150, velocidade=50)
    time.sleep(0.5)
    mover_servo(PINO_PESADO, 140, velocidade=50)
    time.sleep(1.0)
    mover(-30,-30)
    time.sleep(1.0)
    parar_motores()          # <-- adiciona essa também, no final da ré
    #parar_servo(PINO_PESADO)
    #parar_servo(PINO_SG90)
    print("✅ Bola entregue")
    ALVO_COR = None
    return True    

def mover_servo(pino, angulo, velocidade=100, pulso_min=500, pulso_max=2500):
    angulo = max(0, min(180, angulo))
    velocidade = max(1, min(100, velocidade))

    angulo_atual = _angulo_atual.get(pino, ANGULO_INICIAL)
    delay_por_passo = DELAY_BASE * (100 - velocidade) / 100.0

    passo = 2 if angulo > angulo_atual else -2

    for ang in range(int(angulo_atual), int(angulo), passo):
        checar_parada()
        largura_pulso = pulso_min + (ang / 180.0) * (pulso_max - pulso_min)
        try:
            lgpio.tx_servo(h, pino, int(largura_pulso))
        except lgpio.error as e:
            print(f"Falha ao mover servo (pino={pino}, pulso={largura_pulso:.0f}): {e}")
            continue
        if delay_por_passo > 0:
            time.sleep(delay_por_passo)

    _angulo_atual[pino] = angulo
    time.sleep(0.8)

    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error as e:
        print(f"Falha ao desligar PWM do servo (pino={pino}): {e}")

def angulo(ang):
    ang = max(0, min(180, ang))
    pulso = int(500 + (2000 * ang / 180))
    lgpio.tx_servo(h, PINO_SG90, pulso)
    time.sleep(0.1)
    lgpio.tx_servo(h, PINO_SG90, 0)
    time.sleep(0.09)
    
def mover_servo_direto(pino, angulo, pulso_min=500, pulso_max=2500):
    angulo = max(0, min(180, angulo))
    largura_pulso = pulso_min + (angulo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    time.sleep(0.15)
    parar_servo(pino)
    _angulo_atual[pino] = angulo 





def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass

def capturar_crop():
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    return center_crop(frame, CROP_W, CROP_H)


def centralizar_bola():
    """
    Gira o robô até a bola cair dentro de TOLERANCE do centro.
    Velocidade e duração do pulso escalam com o tamanho do erro
    (proporcional), em vez do pulso fixo de antes -- evita corrigir
    de menos com erro grande e passar do ponto com erro pequeno.
    Retorna True se centralizou, False se perdeu a bola.
    """
    perdida = 0
    for _ in range(CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop)

        if cx is None:
            perdida += 1
            if perdida >= CENTER_LOST_FRAMES:
                print("Perdeu a bola durante centralização")
                parar_motores()
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"Centralizado (cx={cx})")
            parar_motores()
            return True

        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)  # 0..1
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN) 
        pulso = 0.05 + erro_norm * 0.10  # erro grande -> pulso mais longo tb

        if erro < 0:
            print(f"Corrigindo esquerda (erro={erro}, speed={speed:.0f})")
            turn_left(speed=speed)
        else:
            print(f"Corrigindo direita (erro={erro}, speed={speed:.0f})")
            turn_right(speed=speed)

        time.sleep(pulso)
        parar_motores()
        time.sleep(0.05)  # deixa a vibração assentar antes de recapturar

    print("Estourou tentativas de centralização")
    parar_motores()
    return False

def detectar_bola(crop):
    """
    Roda o modelo num crop e devolve (cx, area, bbox, cls_id) ou
    (None, None, None, None). cls_id é o índice da classe detectada
    (ex.: bola preta ou bola prata), usado depois pra saber em qual
    área de entrega (cor) soltar a bola.
    """
    results = model.predict(
        crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False, show=False
    )
    r = results[0]
    boxes = getattr(r, "boxes", None)

    annotated = r.plot()
    cv2.imshow("Detecção", annotated)
    cv2.waitKey(1)

    if boxes is not None and len(boxes) > 0:
        idx = escolher_indice_mais_perto(boxes)
        x1, y1, x2, y2 = map(int, boxes.xyxy[idx].tolist())
        cx = (x1 + x2) // 2
        area = (x2 - x1) * (y2 - y1)
        cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
        return cx, area, (x1, y1, x2, y2), cls_id

    return None, None, None, None


def aproximar_bola():
    """
    Avança em passos curtos até a bola ocupar uma fração grande o
    suficiente da imagem (proxy de distância real). Sempre que a bola
    sai da tolerância central durante a aproximação, chama
    centralizar_bola() pra recentralizar antes de continuar avançando.
    Retorna True se chegou perto o suficiente, False se perdeu a bola
    ou estourou o limite de passos.
    """
    crop_area = CROP_W * CROP_H
    perdida = 0

    for passo in range(MAX_APPROACH_STEPS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            print("Erro na câmera durante aproximação")
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop)

        if cx is None:
            perdida += 1
            print(f"Bola sumiu durante aproximação ({perdida}/{LOST_BALL_FRAMES})")
            if perdida >= LOST_BALL_FRAMES:
                print("Perdeu a bola de vez, abortando aproximação")
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"passo={passo} area_ratio={ratio:.2f} cx={cx}")

        if ratio >= STOP_AREA_RATIO:
            print("Perto o suficiente pra garra, parando")
            parar_motores()
            return True

        # se saiu do centro, delega a correção pro centralizar_bola
        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximação, recentralizando")
            centralizada = centralizar_bola()
            if not centralizada:
                print("Perdeu a bola tentando recentralizar na aproximação")
                return False
            continue  # depois de recentralizar, reavalia área no próximo passo

        # já tá centralizada -> só avança
        forward(speed=APPROACH_SPEED)
        time.sleep(APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximação")
    parar_motores()
    return False
# ===================== FUNÇÕES AUXILIARES =====================
def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()

def escolher_indice_mais_perto(boxes):
    """
    Quando o YOLO detecta mais de uma bola no mesmo frame, escolhe a
    de MAIOR área de bbox (ou seja, a mais perto da câmera) e ignora
    as outras nesse ciclo, pra não ficar indeciso sobre qual perseguir.
    """
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

def verificar_e_afastar_bola_bloqueio():
    """
    Verifica, usando o modelo YOLO (mesma detecção que já usamos pra
    achar bola normalmente), se tem alguma bola bloqueando o canto/
    triângulo de entrega. Se tiver, chama o movimento de afastar antes
    de soltar a bola que estamos carregando -- senão ela pode empurrar
    ou trombar na bola que já tá lá.
    """
    crop = capturar_crop()
    if crop is None:
        return

    cx, area, bbox, cls_id = detectar_bola(crop)
    if cx is not None:
        print("Bola bloqueando a área de entrega, afastando...")
        afastar_bola_do_canto()


def afastar_bola_do_canto():
    '''checar_parada()
    mover(-30, -30)
    time.sleep(0.5)
    mover_servo(PINO_PESADO, 5, velocidade=10)
    mover(30, 30)
    time.sleep(0.6)
    mover(-30, 30)
    time.sleep(0.2)
    mover(30, -30)
    time.sleep(0.4)
    mover(-30, 30)
    time.sleep(0.4)
    mover(30, -30)
    time.sleep(0.2)'''
    pass    

# ===================== SETUP CÂMERA E MODELO =====================
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"Não consegui abrir a câmera de resgate em {CAMERA_INDEX}")
    exit(1)

# Camera de baixo (chao) - so usada dentro de acionar_saida() pra ver a
# linha preta. Se nao abrir, acionar_saida() simplesmente ignora esse
# criterio e segue com triangulo/ultrassonico.
cap_linha = None  # só é aberta dentro de acionar_saida()

def abrir_camera_linha():
    global cap_linha
    if cap_linha is not None and cap_linha.isOpened():
        return cap_linha

    cap_linha = cv2.VideoCapture(CAMERA_INDEX_LINHA, cv2.CAP_V4L2)
    while True: 
        if not cap_linha.isOpened():
            print(f"Não consegui abrir a câmera de baixo em {CAMERA_INDEX_LINHA}, tentando fallback...")
            cap_linha.release()
            cap_linha = cv2.VideoCapture(CAMERA_LINHA_INDEX_FALLBACK, cv2.CAP_V4L2)
            continue
        if cap_linha.isOpened():
            break

    if cap_linha.isOpened():
        cap_linha.set(cv2.CAP_PROP_FRAME_WIDTH, LINHA_W)
        cap_linha.set(cv2.CAP_PROP_FRAME_HEIGHT, LINHA_H)
        cap_linha.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    else:
        print("Câmera de baixo indisponível, acionar_saida() vai rodar sem detecção de linha")
        cap_linha = None

    return cap_linha

model = YOLO(MODEL_PATH)
print("📋 Classes do modelo:", model.names)

def _normalizar_nome(nome):
    return str(nome).strip().lower()

_NOME_COR_ENTREGA_NORM = {_normalizar_nome(k): v for k, v in NOME_COR_ENTREGA.items()}

# Monta CLASSE_COR_ENTREGA (id -> cor) a partir do nome de cada classe
for _cls_id, _nome in model.names.items():
    _nome_norm = _normalizar_nome(_nome)
    _cor = _NOME_COR_ENTREGA_NORM.get(_nome_norm)
    if _cor is None:
        print(f"Classe '{_nome}' (repr={_nome!r}, id={_cls_id}) não está em NOME_COR_ENTREGA, "
              f"bolas dessa classe vão ser entregues em qualquer área")
    else:
        CLASSE_COR_ENTREGA[_cls_id] = _cor
print("Mapeamento classe -> cor de entrega:", CLASSE_COR_ENTREGA)
cv2.namedWindow("Detecção", cv2.WINDOW_NORMAL)
mover_servo(PINO_PESADO, 150, velocidade=100)
angulo(0)
ENTREGAS_HERDADAS = 0
for arg in sys.argv[1:]:
    if arg.startswith("--entregas="):
        try:
            ENTREGAS_HERDADAS = int(arg.split("=", 1)[1])
        except ValueError:
            ENTREGAS_HERDADAS = 0

entregas_realizadas = ENTREGAS_HERDADAS
if entregas_realizadas > 0:
    print(f"[BOOT] Retomando resgate com {entregas_realizadas} bola(s) ja entregue(s)")
# Sobe a thread do botão antes do loop principal, igual no seguidor_debug2.py
threading.Thread(target=botao_thread, daemon=True).start()

# ===================== LOOP PRINCIPAL =====================
try:
    print("Iniciando busca por bolas...")
    while True:
        try:
            checar_parada()
            turn_right(speed=55)
            time.sleep(TURN_PULSE)

            parar_motores()

            # Espera a vibração parar
            time.sleep(0.25)

            # Descarta alguns frames antigos da câmera
            for _ in range(4):
                cap.read()

            # Captura um frame atualizado
            ret, frame = cap.read()
            if not ret:
                print("Erro na câmera")
                break
            crop = center_crop(frame, CROP_W, CROP_H)
            inicio = time.perf_counter()
            results = model.predict(
                crop,
                imgsz=(320, 256),
                conf=CONF_THRES,
                verbose=False,
                show=False
            )
            print((time.perf_counter() - inicio) * 1000)
            r = results[0]
            boxes = getattr(r, "boxes", None)

            # SEMPRE mostra a tela, com ou sem deteccao (r.plot() desenha as
            # caixas se houver, ou so a imagem crua se nao houver nada)
            annotated = r.plot()
            cv2.imshow("Detecção", annotated)
            

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if boxes is not None and len(boxes) > 0:
                idx = escolher_indice_mais_perto(boxes)
                xyxy = boxes.xyxy[idx].tolist()
                x1, y1, x2, y2 = map(int, xyxy)
                cx = (x1 + x2) // 2
                cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
                nome_classe = model.names.get(cls_id, "desconhecida") if cls_id is not None else "desconhecida"
                print(f"Bola detectada em cx={cx} | classe={cls_id} ({nome_classe})")

                if abs(cx - IMG_CENTER_X) > TOLERANCE:
                    if not centralizar_bola():
                        print("Não conseguiu centralizar, voltando pra busca")
                        continue

                print("⬆Centralizado → avançando")
                if not aproximar_bola():
                    print("Não conseguiu se aproximar, voltando pra busca")
                    continue
                crop_final = capturar_crop()
                cls_id = None
                if crop_final is not None:
                    _cx_final, _area_final, _bbox_final, cls_id = detectar_bola(crop_final)

                nome_classe_final = model.names.get(cls_id, "desconhecida") if cls_id is not None else "desconhecida"
                print(f"Bola confirmada na garra: classe={cls_id} ({nome_classe_final})")

                print("Fechando garra na bola")
                checar_parada()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                angulo(0)
                mover_servo(PINO_PESADO, 30, velocidade=10)
                mover(-30,-30)
                time.sleep(0.1)
                parar_motores()
                mover(24,24)
                time.sleep(0.65)
                parar_motores()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 0, velocidade=5)
                mover(30,30)
                time.sleep(0.5)
                parar_motores()
                mover(-30,-30)
                time.sleep(0.5)
                parar_motores()
                mover(30,30)
                time.sleep(0.1)
                parar_motores()
                mover(-30,-30)
                time.sleep(0.1)
                parar_motores()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 150, velocidade=5)
                mover(-50,-50)
                time.sleep(0.3)
                parar_motores()
                cor_alvo = CLASSE_COR_ENTREGA.get(cls_id)
                if cor_alvo is None:
                    print(f"Classe {cls_id} n�o mapeada em CLASSE_COR_ENTREGA, entregando em qualquer �rea")
                entregar_bola(cor_alvo)
                entregas_realizadas += 1

                if entregas_realizadas >= MAX_ENTREGAS:

                    print("Buscando se ainda existe alguma bola...")

                    if not buscar_bola():

                        print("Nenhuma bola encontrada.")
                        print("Iniciando busca da saída.")

                        iniciar_modo_saida()

                        while True:

                            if not buscar_saida():
                                continue

                            if not centralizar_saida():
                                continue

                            if not aproximar_saida():
                                continue

                            print("Saida encontrada")
                            saida()    
                            break

                        break
                else:
                    print("Achou mais uma bola na busca final, continuando normalmente")
                    # deixa o loop seguir e a próxima iteração já detecta e pega essa bola

            else:
                print(" Nenhuma bola — novo passo de giro")
        except RoboParado:
            print("Robo parado pelo botao,voltando pro seguidor...")
            parar_motores()
            cap.release()
            if cap_linha is not None:
                cap_linha.release()
            GPIO.gpiochip_close(h)
            os.execv(sys.executable, [
                sys.executable,
                "/home/pi/ROBO_DOGO_2026/codes_rasp/PROTOTIPAGEM_LINE/seguidor_debug2.py",
                f"--entregas={entregas_realizadas}",
            ])

except KeyboardInterrupt:
    print("\nInterrompido pelo usuário.")
finally:
    parar_motores()
    GPIO.gpiochip_close(h)
    cap.release()
    if cap_linha is not None:
        cap_linha.release()
    cv2.destroyAllWindows()
    print("Finalizado com segurança.")
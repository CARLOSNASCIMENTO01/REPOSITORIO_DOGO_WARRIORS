import cv2
import lgpio as GPIO
import time
import lgpio
import numpy as np
import sys
import os
import fcntl
import subprocess
import threading
import smbus2
import serial
import math
import serial.tools.list_ports
from threading import Thread

# ===================== CONSTANTES =====================

BUTTON = 24
PINO_SG90 = 5
PINO_PESADO = 20
ANGULO_INICIAL = 90
SERVO_PESADO_ANGULO_ATIVO = 140
SERVO_PESADO_ANGULO_PARADO = 10
DELAY_BASE = 0.03

WATCHDOG_SEGURANCA_TIMEOUT = 4.0

PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"
SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0

NOME_COR_ENTREGA = {
    "bola-preta": "vermelho",
    "bola-prata": "verde",
}

green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])
red_min1 = np.array([0, 100, 100])
red_max1 = np.array([10, 255, 255])
red_min2 = np.array([170, 100, 100])
red_max2 = np.array([180, 255, 255])
kernel_tri = np.ones((3, 3), np.uint8)

PROPORCAO_ESPERADA = 30 / 6
TOLERANCIA_PROPORCAO = 0.8
AREA_MIN_TRI = 2500 #500 - baixo demais, deixava passar ruido pequeno (blob de ~510px que nao era triangulo nenhum)
AREA_MAX_TRI = 1000000

STOP_AREA_RATIO_TRIANGULO = 0.80
TRIANGULO_MAX_GIROS = 90
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_APPROACH_SPEED = 20
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 15
TRIANGULO_CENTER_LOST_FRAMES = 15

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.onnx")
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

DEBUG_VISUAL = False

YAHBOOM_PORTA = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70
PWM_FREIO_YAHBOOM = 700
TEMPO_FREIO_YAHBOOM = 0.08

VEL_MAX = 20
VEL_MIN = -VEL_MAX

APPROACH_SPEED = 30#25
APPROACH_STEP_TIME = 0.18#0.18
STOP_AREA_RATIO = 0.85
MAX_APPROACH_STEPS = 60
LOST_BALL_FRAMES = 8
TURN_PULSE = 0.2#0.16
STOP_TIME = 0.1#0.2
FORWARD_TIME = 0.6

CENTER_TURN_SPEED_MIN = 15
CENTER_TURN_SPEED_MAX = 25
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5

MAX_ENTREGAS = 3
BUSCA_FINAL_MAX_GIROS = 20

CAMERA_INDEX_LINHA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK = 0
LINHA_W, LINHA_H = 160, 120
LIMIAR_LINHA = 80  # calibrado ao vivo, era 80
AREA_MIN_LINHA = 11000  # calibrado ao vivo, era 11000
kernel_linha = np.ones((3, 3), np.uint8)
AREA_MAX_LINHA_FRAC = 0.5  # calibrado ao vivo, era 0.5
TOLERANCIA_ANGULO_VERTICAL_LINHA = 30  # calibrado ao vivo, era 30
LIMIAR_LINHA_H = 123  # calibrado ao vivo, era 123
AREA_MIN_LINHA_H = 20000  # calibrado ao vivo, era 20000
AREA_MAX_LINHA_FRAC_H = 0.81  # calibrado ao vivo, era 0.81
TOLERANCIA_ANGULO_HORIZONTAL_LINHA = 58  # calibrado ao vivo, era 58

DIST_LATERAL_ABERTA_CM = 30
DIST_PAREDE_FRENTE_CM = 10

VEL_SAIDA = 15
TEMPO_MAX_CONFIRMAR_LATERAL = 1.0

USBDEVFS_RESET = 21780
USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3

BNO_ADDR        = 0x29
REG_CHIP_ID     = 0x00
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL    = 0x3B
GYR_DATA_X      = 0x14
ACC_DATA_X      = 0x08
EUL_DATA_HEADING = 0x1A
MODO_CONFIG     = 0x00
MODO_IMUPLUS    = 0x08
CHIP_ID_ESPERADO = 0xA0
LIMITE_DPS_PLAUSIVEL = 2000.0

FALHAS_CAMERA_ANTES_DE_DESISTIR = 10


# ===================== ESTADO GLOBAL =====================

class RoboParado(Exception):
    pass

robo_ativo = threading.Event()
robo_ativo.set()
_ultimo_heartbeat = [time.time()]

dados = {"F": 9999.0, "E": 9999.0}
ultimo_recebimento = time.time()
ser = None
_serial_thread_iniciada = [False]

ALVO_COR = None
TIPO_BOLA = None
CLASSE_COR_ENTREGA = {}
model = None

_gyro_lock = threading.Lock()
bus = smbus2.SMBus(1)
ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()
_heading_offset = 0.0
_ultimo_angulo_z = [0]
_mpu_thread_iniciada = [False]

_ultima_vel = (0, 0)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL,
    PINO_PESADO: ANGULO_INICIAL,
}

cap_linha = None
falhas_camera_seguidas = 0


# ===================== BOTAO / PARADA IMEDIATA =====================

def checar_parada():
    _ultimo_heartbeat[0] = time.time()
    if not robo_ativo.is_set():
        raise RoboParado()

def botao_thread():
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
                    try:
                        mover_servo_direto(PINO_PESADO, 90, manter_pulso=True)
                        _angulo_atual[PINO_PESADO] = 90
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao abaixar servo pesado: {e}")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_ATIVO, manter_pulso=True)
                        _angulo_atual[PINO_PESADO] = SERVO_PESADO_ANGULO_ATIVO
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao levantar servo pesado: {e}")
                while GPIO.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
        anterior = estado
        time.sleep(0.02)


# ===================== SERIAL / SONAR =====================

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
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            time.sleep(1.0)

def distancia_frente():
    return dados["F"]

def distancia_esquerda():
    return dados["E"]

def sonar_valido():
    return (time.time() - ultimo_recebimento) < SERIAL_TIMEOUT_DADO

def aguardar_leitura_fresca_sonar(timeout=0.5):
    referencia = ultimo_recebimento
    inicio = time.time()
    while ultimo_recebimento == referencia:
        checar_parada()
        if time.time() - inicio > timeout:
            break
        time.sleep(0.01)

def iniciar_serial():
    if _serial_thread_iniciada[0]:
        return
    _serial_thread_iniciada[0] = True

    def _start():
        handshake_serial()
        serial_thread()

    threading.Thread(target=_start, daemon=True).start()


# ===================== GIROSCOPIO (BNO055) =====================

def _bno_write(reg, val):
    bus.write_byte_data(BNO_ADDR, reg, val)

def _bno_read_word(reg, tentativas=3):
    for tentativa in range(tentativas):
        try:
            lo = bus.read_byte_data(BNO_ADDR, reg)
            hi = bus.read_byte_data(BNO_ADDR, reg + 1)
            valor = (hi << 8) | lo
            if valor >= 0x8000:
                valor -= 65536
            return valor
        except OSError as e:
            if tentativa == tentativas - 1:
                raise
            time.sleep(0.001)

def _normalizar(ang):
    while ang > 180: ang -= 360
    while ang < -180: ang += 360
    return ang

def _ler_heading_bruto():
    with _gyro_lock:
        return _bno_read_word(EUL_DATA_HEADING) / 16.0

def _calibrar(amostras=200):
    gx = gy = gz = 0
    for _ in range(amostras):
        gx += _bno_read_word(GYR_DATA_X)
        gy += _bno_read_word(GYR_DATA_X + 2)
        gz += _bno_read_word(GYR_DATA_X + 4)
        time.sleep(0.005)
    gyro_offset["X"] = gx / amostras
    gyro_offset["Y"] = gy / amostras
    gyro_offset["Z"] = gz / amostras

def resetar_giroscopio():
    global ang_gyro, gyro_ultimo_tempo, _heading_offset
    with _gyro_lock:
        ang_gyro = {"X": 0.0, "Y": 0.0, "Z": 0.0}
        gyro_ultimo_tempo = time.time()
    try:
        _heading_offset = _ler_heading_bruto()
    except Exception as e:
        print(f"[BNO055] Falha ao ler heading pro reset ({e}), mantendo offset anterior.")

def angulo_z():
    try:
        atual = _ler_heading_bruto()
        _ultimo_angulo_z[0] = int(_normalizar(atual - _heading_offset))
    except Exception as e:
        print(f"[BNO055] Falha ao ler heading calculado ({e}), mantendo ultimo valor.")
    return _ultimo_angulo_z[0]

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
            with _gyro_lock:
                gx = (_bno_read_word(GYR_DATA_X)     - gyro_offset["X"]) / 16.0
                gy = (_bno_read_word(GYR_DATA_X + 2) - gyro_offset["Y"]) / 16.0
                gz = (_bno_read_word(GYR_DATA_X + 4) - gyro_offset["Z"]) / 16.0
            if max(abs(gx), abs(gy), abs(gz)) > LIMITE_DPS_PLAUSIVEL:
                print(f"[BNO055] Amostra de giro implausivel descartada (X={gx:.0f} Y={gy:.0f} Z={gz:.0f} dps)")
                time.sleep(0.005)
                continue
            with _gyro_lock:
                ang_gyro["X"] = _normalizar(ang_gyro["X"] + gx * dt)
                ang_gyro["Y"] = _normalizar(ang_gyro["Y"] + gy * dt)
                ang_gyro["Z"] = _normalizar(ang_gyro["Z"] + gz * dt)
            time.sleep(0.005)
        except OSError as e:
            print(f"[BNO055] Gyro I2C: {e}")
            time.sleep(0.005)
        except Exception as e:
            print(f"[BNO055] Gyro: {e}")
            time.sleep(0.005)

def iniciar_mpu():
    _bno_write(REG_OPR_MODE, MODO_CONFIG)
    time.sleep(0.02)
    _bno_write(REG_SYS_TRIGGER, 0x20)
    time.sleep(0.65)
    for _ in range(20):
        try:
            if bus.read_byte_data(BNO_ADDR, REG_CHIP_ID) == CHIP_ID_ESPERADO:
                break
        except OSError:
            pass
        time.sleep(0.05)
    _bno_write(REG_PWR_MODE, 0x00)
    time.sleep(0.02)
    _bno_write(REG_UNIT_SEL, 0x00)
    time.sleep(0.02)
    _bno_write(REG_OPR_MODE, MODO_IMUPLUS)
    time.sleep(0.03)
    print("[BNO055] Calibrando...")
    _calibrar()
    resetar_giroscopio()
    print("[BNO055] Pronto")
    Thread(target=_gyro_thread, daemon=True).start()

def iniciar_mpu_thread():
    if _mpu_thread_iniciada[0]:
        return
    _mpu_thread_iniciada[0] = True
    Thread(target=iniciar_mpu, daemon=True).start()


# ===================== MODELO YOLO =====================

def carregar_modelo():
    global model
    os.environ.setdefault("YOLO_OFFLINE", "true")
    from ultralytics import YOLO

    def _normalizar_nome(nome):
        return str(nome).strip().lower()

    print(f"[MODELO] Carregando YOLO de {MODEL_PATH}...")
    try:
        m = YOLO(MODEL_PATH)
    except Exception as e:
        print(f"[MODELO][ERRO CRITICO] Falha ao carregar {MODEL_PATH}: {e!r}")
        print("[MODELO][ERRO CRITICO] Deteccao de bola NAO vai funcionar - "
              "o robo vai ficar girando sem parar em bola nenhuma ate isso ser corrigido")
        return
    print("Classes do modelo:", m.names)

    nome_cor_norm = {_normalizar_nome(k): v for k, v in NOME_COR_ENTREGA.items()}
    for cls_id, nome in m.names.items():
        cor = nome_cor_norm.get(_normalizar_nome(nome))
        if cor is None:
            print(f"Classe '{nome}' (id={cls_id}) nao esta em NOME_COR_ENTREGA, "
                  f"bolas dessa classe vao ser entregues em qualquer area")
        else:
            CLASSE_COR_ENTREGA[cls_id] = cor
    print("Mapeamento classe -> cor de entrega:", CLASSE_COR_ENTREGA)

    model = m
    print("[MODELO] Pronto")

def watchdog_nudge():
    while model is None:
        time.sleep(5.0)
        if model is not None:
            break
        if not robo_ativo.is_set():
            continue
        turn_right(speed=40)
        time.sleep(0.15)
        parar_motores()


# ===================== WATCHDOG DE SEGURANCA =====================

def watchdog_seguranca():
    while True:
        time.sleep(0.5)
        if time.time() - _ultimo_heartbeat[0] > WATCHDOG_SEGURANCA_TIMEOUT:
            print(f"[WATCHDOG SEGURANCA] Loop principal sem responder ha mais de "
                  f"{WATCHDOG_SEGURANCA_TIMEOUT:.0f}s - forcando parada dos motores")
            try:
                _enviar_pwm_yahboom(0, 0, 0, 0)
            except Exception:
                pass
            _ultimo_heartbeat[0] = time.time()


# ===================== MOTORES =====================

def _enviar_pwm_yahboom(m1, m2, m3, m4):
    m1, m2, m3, m4 = (max(-3600, min(3600, int(v))) for v in (m1, m2, m3, m4))
    try:
        motor_serial.write(f"$pwm:{m1},{m2},{m3},{m4}#".encode())
    except serial.SerialException as e:
        print(f"[MOTOR][ERRO] Falha ao mandar PWM pra placa Yahboom: {e}")

def aplicar_motor(velL, velR):
    velL = max(min(velL, VEL_MAX), VEL_MIN)
    velR = max(min(velR, VEL_MAX), VEL_MIN)
    pwm_esq = velL * ESCALA_PWM_YAHBOOM
    pwm_dir = velR * ESCALA_PWM_YAHBOOM
    _enviar_pwm_yahboom(pwm_dir, pwm_esq, pwm_dir, pwm_esq)

def mover(velL, velR):
    global _ultima_vel
    aplicar_motor(velL, velR)
    _ultima_vel = (velL, velR)

def parar_motores():
    global _ultima_vel
    velL, velR = _ultima_vel
    if velL != 0 or velR != 0:
        freio_vel = PWM_FREIO_YAHBOOM / ESCALA_PWM_YAHBOOM
        freio_esq = -freio_vel if velL > 0 else (freio_vel if velL < 0 else 0)
        freio_dir = -freio_vel if velR > 0 else (freio_vel if velR < 0 else 0)
        aplicar_motor(freio_esq, freio_dir)
        time.sleep(TEMPO_FREIO_YAHBOOM)
    mover(0, 0)

def forward(speed=30):
    mover(speed, speed)

def turn_left(speed=30):
    mover(-speed, speed)

def turn_right(speed=30):
    mover(speed, -speed)

def guinada2(LADO, GRAUS, VELOCIDADE):
    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    if LADO == 'E':
        while True:
            checar_parada()
            mover(-VELOCIDADE, VELOCIDADE)
            if angulo_z() <= -Graus:
                mover(2, -2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    else:
        while True:
            checar_parada()
            mover(VELOCIDADE, -VELOCIDADE)
            if angulo_z() >= Graus:
                mover(-2, 2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    resetar_giroscopio()


# ===================== SERVOS =====================

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

def mover_servo_direto(pino, angulo, pulso_min=500, pulso_max=2500, manter_pulso=False):
    angulo = max(0, min(180, angulo))
    largura_pulso = pulso_min + (angulo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    time.sleep(0.15)
    if not manter_pulso:
        parar_servo(pino)
    _angulo_atual[pino] = angulo

def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass

def oscilar_servo_lgpio(pino, angulo_a, angulo_b, repeticoes=5, delay=0.15):
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
            lgpio.tx_servo(h, pino, pulso_a)
            time.sleep(delay)
            lgpio.tx_servo(h, pino, pulso_b)
            time.sleep(delay)
    except lgpio.error as e:
        print("ERRO OSCILA SERVO:", e)
    finally:
        time.sleep(0.2)


# ===================== CAMERA / CROP =====================

def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()

def capturar_crop():
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    return center_crop(frame, CROP_W, CROP_H)

def escolher_indice_mais_perto(boxes):
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


# ===================== CAMERA DE BAIXO (LINHA) =====================

def _achar_bus_dev(video_path):
    nome = os.path.basename(os.path.realpath(video_path))
    sys_dev = f"/sys/class/video4linux/{nome}/device"
    d = os.path.realpath(sys_dev)
    for _ in range(8):
        busnum_f = os.path.join(d, "busnum")
        devnum_f = os.path.join(d, "devnum")
        if os.path.exists(busnum_f) and os.path.exists(devnum_f):
            with open(busnum_f) as f:
                busnum = int(f.read().strip())
            with open(devnum_f) as f:
                devnum = int(f.read().strip())
            return busnum, devnum
        novo_d = os.path.dirname(d)
        if novo_d == d:
            break
        d = novo_d
    return None, None

def resetar_usb_camera_linha(video_path_ou_indice):
    if isinstance(video_path_ou_indice, int):
        video_path = f"/dev/video{video_path_ou_indice}"
    else:
        video_path = video_path_ou_indice

    ok = False
    try:
        busnum, devnum = _achar_bus_dev(video_path)
        if busnum is not None:
            usb_node = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
            fd = os.open(usb_node, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                print(f"[USB] Reset via ioctl OK em {usb_node} (camera {video_path})")
                ok = True
            finally:
                os.close(fd)
        else:
            print(f"[USB][AVISO] Nao achei bus/dev USB pra {video_path}")
    except PermissionError:
        print("[USB][ERRO] Sem permissao pra resetar USB (roda como root ou "
              "cria regra de udev liberando /dev/bus/usb/*)")
    except Exception as e:
        print("[USB][ERRO] Falha no reset via ioctl:", e)

    try:
        subprocess.run(["uhubctl"], capture_output=True, timeout=2)
    except FileNotFoundError:
        pass
    except Exception:
        pass

    time.sleep(1.0)
    return ok

def _abrir_camera_linha_generica(fonte):
    cap_local = cv2.VideoCapture(fonte, cv2.CAP_V4L2)
    if cap_local.isOpened():
        ret, _ = cap_local.read()
        if ret:
            return cap_local
        cap_local.release()
    return None

def _detectar_camera_linha():
    if CAMERA_INDEX_LINHA:
        checar_parada()
        c = _abrir_camera_linha_generica(CAMERA_INDEX_LINHA)
        if c is not None:
            print(f"[Camera] Linha conectada em {CAMERA_INDEX_LINHA}")
            return c

    checar_parada()
    c = _abrir_camera_linha_generica(CAMERA_LINHA_INDEX_FALLBACK)
    if c is not None:
        print(f"[Camera] Linha conectada no indice {CAMERA_LINHA_INDEX_FALLBACK}")
        return c

    for i in range(10):
        checar_parada()
        c = _abrir_camera_linha_generica(i)
        if c is not None:
            print(f"[Camera] Linha conectada em /dev/video{i}")
            return c
    return None

def abrir_camera_linha():
    global cap_linha
    if cap_linha is not None and cap_linha.isOpened():
        return cap_linha

    tentativas = 0
    cap_local = _detectar_camera_linha()
    while cap_local is None:
        tentativas += 1
        print(f"[Camera] Linha (saida) ainda nao apareceu, tentando de novo... (tentativa {tentativas})")
        checar_parada()
        time.sleep(1)
        if tentativas % USB_RESET_TENTATIVAS_ANTES_DE_RESETAR == 0:
            print("[Camera] Varias falhas seguidas, tentando resetar USB...")
            resetar_usb_camera_linha(CAMERA_INDEX_LINHA or CAMERA_LINHA_INDEX_FALLBACK)
        cap_local = _detectar_camera_linha()

    cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap_local.set(cv2.CAP_PROP_FRAME_WIDTH, LINHA_W)
    cap_local.set(cv2.CAP_PROP_FRAME_HEIGHT, LINHA_H)
    cap_linha = cap_local
    return cap_linha

def _maior_contorno_linha(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    maior = max(contours, key=cv2.contourArea)
    area_contorno = cv2.contourArea(maior)
    area_roi = roi.shape[0] * roi.shape[1]

    if area_contorno < AREA_MIN_LINHA:
        return None
    if area_contorno >= area_roi * AREA_MAX_LINHA_FRAC:
        return None
    return maior

def detectar_linha_embaixo(frame):
    return _maior_contorno_linha(frame) is not None

def detectar_linha_vertical_embaixo(frame):
    maior = _maior_contorno_linha(frame)
    if maior is None:
        return False
    vx, vy, _, _ = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    desvio_vertical = math.degrees(math.atan2(abs(float(vx)), abs(float(vy))))
    return desvio_vertical <= TOLERANCIA_ANGULO_VERTICAL_LINHA

def _maior_contorno_linha_h(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA_H, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    maior = max(contours, key=cv2.contourArea)
    area_contorno = cv2.contourArea(maior)
    area_roi = roi.shape[0] * roi.shape[1]

    if area_contorno < AREA_MIN_LINHA_H:
        return None
    if area_contorno >= area_roi * AREA_MAX_LINHA_FRAC_H:
        return None
    return maior

def detectar_linha_horizontal_embaixo(frame):
    maior = _maior_contorno_linha_h(frame)
    if maior is None:
        return False
    vx, vy, _, _ = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    desvio_vertical = math.degrees(math.atan2(abs(float(vx)), abs(float(vy))))
    desvio_horizontal = 90 - desvio_vertical
    return desvio_horizontal <= TOLERANCIA_ANGULO_HORIZONTAL_LINHA

def confirmar_saida_lateral():
    achou = False
    inicio = time.time()
    while time.time() - inicio < 3.0:
        checar_parada()
        mover(VEL_SAIDA, VEL_SAIDA)
        if cap_linha is not None:
            ret_l3, frame_l3 = cap_linha.read()
            if ret_l3 and detectar_linha_embaixo(frame_l3):
                achou = True
                break
        time.sleep(0.02)
    parar_motores()
    return achou

def alinhar_na_linha():
    mover(20, 20)
    time.sleep(0.7)
    guinada2('E', 18, 20)

    TOLERANCIA_CENTRO_LINHA = 30
    FRAMES_CONFIRMACAO = 3
    ALINHAR_VEL = 13
    frames_centralizado_seguidos = 0
    while True:
        checar_parada()
        mover(ALINHAR_VEL, -ALINHAR_VEL)
        ret_l, frame_l = cap_linha.read()
        if ret_l:
            maior = _maior_contorno_linha(frame_l)
            cx = None
            if maior is not None:
                M = cv2.moments(maior)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
            centro = frame_l.shape[1] // 2
            centralizado = cx is not None and abs(cx - centro) < TOLERANCIA_CENTRO_LINHA
            frames_centralizado_seguidos = frames_centralizado_seguidos + 1 if centralizado else 0
            print(f"[SAIDA-ALINHA] cx={cx} centralizado={centralizado} seguidos={frames_centralizado_seguidos}")
            if frames_centralizado_seguidos >= FRAMES_CONFIRMACAO:
                print("[SAIDA] Contorno da linha encontrado de novo, alinhado")
                break
        time.sleep(0.005)
    parar_motores()


# ===================== FUNCOES DE BOLA =====================

def detectar_bola(crop):
    results = model.predict(
        crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False, show=False
    )
    r = results[0]
    boxes = getattr(r, "boxes", None)

    if boxes is not None and len(boxes) > 0:
        idx = escolher_indice_mais_perto(boxes)
        x1, y1, x2, y2 = map(int, boxes.xyxy[idx].tolist())
        cx = (x1 + x2) // 2
        area = (x2 - x1) * (y2 - y1)
        cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None

        if DEBUG_VISUAL:
            debug_img = crop.copy()
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            nome_classe = model.names.get(cls_id, str(cls_id)) if cls_id is not None else "?"
            cv2.putText(debug_img, nome_classe, (x1, max(y1 - 5, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.imshow("VITIMA (bola) - deteccao", debug_img)
            cv2.waitKey(1)

        return cx, area, (x1, y1, x2, y2), cls_id

    if DEBUG_VISUAL:
        cv2.imshow("VITIMA (bola) - deteccao", crop)
        cv2.waitKey(1)

    return None, None, None, None

def centralizar_bola():
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
                mover(-20, -20)
                time.sleep(0.15)
                guinada2('E', 5, 15)
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"Centralizado (cx={cx})")
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

    print("Estourou tentativas de centralização")
    mover(20, 20)
    time.sleep(0.15)
    guinada2('E', 5, 15)
    parar_motores()
    return False

def aproximar_bola():
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
                mover(20, 20)
                time.sleep(0.15)
                guinada2('E', 5, 15)
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"passo={passo} area_ratio={ratio:.2f} cx={cx}")

        if ratio >= STOP_AREA_RATIO:
            print(f"[APROXIMAR] area_ratio={ratio:.2f} bateu o limiar, confirmando de novo...")
            parar_motores()
            time.sleep(0.1)
            crop_confirma = capturar_crop()
            ratio_confirma = None
            if crop_confirma is not None:
                cx_confirma, area_confirma, _, _ = detectar_bola(crop_confirma)
                if cx_confirma is not None:
                    ratio_confirma = area_confirma / crop_area

            if ratio_confirma is not None and ratio_confirma >= STOP_AREA_RATIO:
                print(f"[APROXIMAR] Confirmado (area_ratio={ratio_confirma:.2f}), perto o suficiente pra garra")
                return True

            print(f"[APROXIMAR] Nao confirmou (area_ratio={ratio_confirma}), continuando aproximacao")
            continue

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximação, recentralizando")
            centralizada = centralizar_bola()
            if not centralizada:
                print("Perdeu a bola tentando recentralizar na aproximação")
                return False
            continue

        forward(speed=APPROACH_SPEED)
        time.sleep(APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximação")
    parar_motores()
    return False

def buscar_bola():
    CONFIRMACAO_BUSCA_FINAL = 3
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
        if cx is None:
            continue

        confirmacoes = 1
        for _ in range(CONFIRMACAO_BUSCA_FINAL - 1):
            checar_parada()
            crop_confirma = capturar_crop()
            if crop_confirma is None:
                break
            cx_confirma, _, _, _ = detectar_bola(crop_confirma)
            if cx_confirma is None:
                break
            confirmacoes += 1

        if confirmacoes >= CONFIRMACAO_BUSCA_FINAL:
            return True
        print(f"[BUSCA FINAL] Deteccao efemera descartada ({confirmacoes}/{CONFIRMACAO_BUSCA_FINAL} frames), continuando busca")
    return False

def verificar_e_afastar_bola_bloqueio():
    crop = capturar_crop()
    if crop is None:
        return

    cx, area, bbox, cls_id = detectar_bola(crop)
    if cx is not None:
        print("Bola bloqueando a área de entrega, afastando...")
        afastar_bola_do_canto()

def afastar_bola_do_canto():
    checar_parada()
    guinada2('E', 175, 25)
    guinada2('E', 160, 25)


# ===================== FUNCOES DE TRIANGULO =====================

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

def _eh_triangulo(contorno, encostando_borda=False):
    area = cv2.contourArea(contorno)
    if area < AREA_MIN_TRI or area > AREA_MAX_TRI:
        return False, None

    perimetro = cv2.arcLength(contorno, True)
    aproximado = cv2.approxPolyDP(contorno, 0.06 * perimetro, True)  #0.10

    min_vertices = 2 if encostando_borda else 3
    if len(aproximado) < min_vertices or len(aproximado) > 4:
        return False, None

    if not encostando_borda and not cv2.isContourConvex(aproximado):
        return False, None

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
        for c in contornos:
            x, y, w, h_bb = cv2.boundingRect(c)
            margem = 2
            encostando_borda = (
                x <= margem or y <= margem or
                x + w >= crop.shape[1] - margem or
                y + h_bb >= crop.shape[0] - margem
            )

            ok, info = _eh_triangulo(c, encostando_borda=encostando_borda)
            if ok:
                info["cor"] = cor
                todas_deteccoes.append(info)

    if DEBUG_VISUAL:
        debug_img = crop.copy()
        for d in todas_deteccoes:
            cor_bgr = (0, 255, 0) if d["cor"] == "verde" else (0, 0, 255)
            cv2.drawContours(debug_img, [d["vertices"]], -1, cor_bgr, 2)
            cv2.circle(debug_img, d["centro"], 4, (0, 255, 255), -1)
        cv2.imshow("TRI - crop + deteccoes", debug_img)
        if mask_verde is not None:
            cv2.imshow("TRI - mask verde", mask_verde)
        if mask_vermelho is not None:
            cv2.imshow("TRI - mask vermelho", mask_vermelho)
        cv2.waitKey(1)

    if not todas_deteccoes:
        return None, None, None

    melhor = max(todas_deteccoes, key=lambda d: d["area"])
    return melhor["centro"][0], melhor["area"], melhor

def centralizar_triangulo():
    perdida = 0
    for _ in range(TRIANGULO_CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, info = detectar_entrega(crop)

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
            parar_motores()
            perdida += 1
            print(f"Triângulo sumiu durante aproximação ({perdida}/{TRIANGULO_LOST_FRAMES})")
            if perdida >= TRIANGULO_LOST_FRAMES:
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

        forward(speed=20)
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximação da entrega")
    parar_motores()
    return False

def reposicionar_canto(tentativa, vel_recuo=20):
    if tentativa <= 2:
        mover(-vel_recuo, -vel_recuo)
        time.sleep(0.4)
        mover(-30, 0)
        time.sleep(0.3)
        parar_motores()
    else:
        mover(15, 15)
        time.sleep(0.4)
        parar_motores()
        time.sleep(0.1)
        mover(22, 22)
        time.sleep(0.7)
        parar_motores()

def buscar_triangulo():
    for _ in range(TRIANGULO_MAX_GIROS):
        checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, area, info = detectar_entrega(crop)
            if cx is not None:
                cor_dbg = info["cor"] if info else "?"
                area_dbg = info["area"] if info else area
                print(f"[BUSCAR_TRIANGULO] Detectado cor={cor_dbg} area={area_dbg} cx={cx}")
                return True
        turn_right(speed=15)
        time.sleep(0.3)
    return False

def entregar_bola(cor_alvo=None):
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

    tentativas_reposicionar = 0
    while True:
        if not buscar_triangulo():
            print("Não encontrou triângulo, reposicionando...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=25)
            continue

        if not centralizar_triangulo():
            print("Não conseguiu centralizar, reposicionando...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=25)
            continue

        if not aproximar_triangulo():
            print("Não conseguiu aproximar, reposicionando...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=25)
            continue

        break

    print("Soltando a bola...")

    checar_parada()
    verificar_e_afastar_bola_bloqueio()   # <-- checa e afasta bola do canto, se houver

    checar_parada()
    print("📤 Soltando a bola...")
    mover(40,40)
    time.sleep(1.5)
    parar_motores()          # <-- adiciona os parênteses
    angulo(170)
    mover_servo(PINO_PESADO, 20, velocidade=60)
    oscilar_servo_lgpio(PINO_PESADO, 30, 60,13,0.2)

    mover_servo(PINO_PESADO, 150, velocidade=50)
    time.sleep(0.5)
    mover_servo(PINO_PESADO, 140, velocidade=50)
    time.sleep(1.0)
    mover(-30,-30)
    time.sleep(1.25)
    parar_motores()          # <-- adiciona essa também, no final da ré
    print("✅ Bola entregue")
    ALVO_COR = None
    return True


# ===================== SAIDA =====================

def acionar_saida():
    global ALVO_COR
    print("[SAIDA] Procurando canto verde para se alinhar...")
    ALVO_COR = "vermelho" #MUDA AQUIIIIIIIIIIIIII!

    MAX_TENTATIVAS_ALINHAR_CANTO = 5
    alinhou_no_canto = False
    tentativas_reposicionar = 0
    for _ in range(MAX_TENTATIVAS_ALINHAR_CANTO):
        checar_parada()

        if not buscar_triangulo():
            print("[SAIDA] Canto verde nao encontrado, reposicionando e tentando de novo...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=15)
            continue

        if not centralizar_triangulo():
            print("[SAIDA] Nao conseguiu centralizar no canto verde, reposicionando e tentando de novo...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=15)
            continue

        print("[SAIDA] Indo ate o canto verde...")
        if not aproximar_triangulo():
            print("[SAIDA] Nao conseguiu chegar no canto verde, reposicionando e tentando de novo...")
            tentativas_reposicionar += 1
            reposicionar_canto(tentativas_reposicionar, vel_recuo=15)
            continue

        alinhou_no_canto = True
        break

    if not alinhou_no_canto:
        print(f"[SAIDA] Estourou {MAX_TENTATIVAS_ALINHAR_CANTO} tentativas de alinhar no canto verde, seguindo reto sem alinhar")

    cam_linha = abrir_camera_linha()
    if cam_linha is None:
        print("[SAIDA] Camera de baixo indisponivel - abortando saida, nao e seguro prosseguir sem ela")
        return "sem_camera_linha"

    print(f"[SAIDA] Andando reto (vel={VEL_SAIDA}) ate achar canto verde/vermelho, "
          f"linha preta ou sonar lateral aberto...")
    mover(20,20)
    time.sleep(1.5)
    mover(-20,-20)
    time.sleep(0.7)
    guinada2('E', 170, 20)
    mover(-25,-25)
    time.sleep(3.5)
    mover(25,25)
    time.sleep(0.15)
    guinada2('E', 90, 15)
    mover(20,20)
    time.sleep(0.8)
    guinada2('D', 45, 15)
    print(distancia_esquerda())
    if distancia_esquerda() <= 2:
        mover(20, 0)
        time.sleep(0.70)
        parar_motores()
        mover(0, 20)
        time.sleep(0.60)
        parar_motores()
    elif distancia_esquerda() <= 5:
        mover(20, 0)
        time.sleep(0.40)
        parar_motores()
        mover(0, 20)
        time.sleep(0.30)
        parar_motores()    
    elif distancia_esquerda() >= 9: 
        mover(20,20)
        time.sleep(0.5)
        mover(0, 20)
        time.sleep(0.60)
        parar_motores()
        mover(20, 0)
        time.sleep(0.60)
        parar_motores()
    print(distancia_esquerda())

    ALVO_COR = None

    TEMPO_IGNORAR_CANTO = 1.5
    ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO

    FRAMES_CONFIRMACAO_LINHA = 7  #3
    frames_vertical_seguidos = 0
    frames_horizontal_seguidos = 0

    try:
        while True:
            checar_parada()
            mover(VEL_SAIDA, VEL_SAIDA)

            ret_r, frame_r = cap.read()
            if ret_r and time.time() >= ignorar_canto_ate:
                crop = center_crop(frame_r, CROP_W, CROP_H)
                cx, area, info = detectar_entrega(crop)
                if cx is not None:
                    parar_motores()
                    cor_achada = info["cor"] if info else "?"
                    print(f"[SAIDA] Canto {cor_achada} detectado na camera de resgate")
                    print("vi um tringulozao verde ou vermelho")
                    mover(25, 25)
                    time.sleep(2.0)
                    mover(-25, -25)
                    time.sleep(0.4)
                    guinada2('D', 90, 15)
                    mover(15,15)
                    time.sleep(2.2)
                    guinada2('D', 33, 15)
                    print(distancia_esquerda())
                    if distancia_esquerda() <= 2:
                        mover(20, 0)
                        time.sleep(0.70)
                        parar_motores()
                        mover(0, 20)
                        time.sleep(0.60)
                        parar_motores()
                    elif distancia_esquerda() <= 5:
                        mover(20, 0)
                        time.sleep(0.40)
                        parar_motores()
                        mover(0, 20)
                        time.sleep(0.30)
                        parar_motores()    
                    elif distancia_esquerda() >= 9: 
                        mover(20,20)
                        time.sleep(0.5)
                        mover(0, 20)
                        time.sleep(0.60)
                        parar_motores()
                        mover(20, 0)
                        time.sleep(0.60)
                        parar_motores()
                    print(distancia_esquerda())
                    ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO

            if cam_linha is not None:
                ret_l, frame_l = cam_linha.read()

                vertical_visto = ret_l and detectar_linha_vertical_embaixo(frame_l)
                horizontal_visto = ret_l and not vertical_visto and detectar_linha_horizontal_embaixo(frame_l)
                frames_vertical_seguidos = frames_vertical_seguidos + 1 if vertical_visto else 0
                frames_horizontal_seguidos = frames_horizontal_seguidos + 1 if horizontal_visto else 0

                if frames_vertical_seguidos >= FRAMES_CONFIRMACAO_LINHA:
                    frames_vertical_seguidos = 0
                    parar_motores()
                    print("[SAIDA] Linha vertical (entrada) detectada, recuando e virando...")
                    mover(-20, -20)
                    time.sleep(0.6)
                    guinada2('D', 90, 15)
                    mover(20, 20)
                    time.sleep(1.1)

                elif frames_horizontal_seguidos >= FRAMES_CONFIRMACAO_LINHA:
                    frames_horizontal_seguidos = 0
                    parar_motores()
                    print("[SAIDA] Linha horizontal (saida) detectada na camera de baixo")
                    print("[SAIDA] Alinhando na linha...")
                    alinhar_na_linha()
                    return "linha_preta"

            if sonar_valido() and distancia_frente() < 10:
                parar_motores()
                time.sleep(0.2)
                achou_canto = False
                if ret_r and time.time() >= ignorar_canto_ate:
                    crop = center_crop(frame_r, CROP_W, CROP_H)
                    cx, area, info = detectar_entrega(crop)
                    if cx is not None:
                        achou_canto = True
                        parar_motores()
                        cor_achada = info["cor"] if info else "?"
                        print(f"[SAIDA] Canto {cor_achada} detectado na camera de resgate")
                        print("vi um tringulozao verde ou vermelho como parede")
                        mover(25, 25)
                        time.sleep(2.0)
                        mover(-25, -25)
                        time.sleep(0.4)
                        guinada2('D', 90, 15)
                        mover(15,15)
                        time.sleep(2.2)
                        guinada2('D', 33, 15)
                        print(distancia_esquerda())
                        if distancia_esquerda() <= 2:
                            mover(20, 0)
                            time.sleep(0.70)
                            parar_motores()
                            mover(0, 20)
                            time.sleep(0.60)
                            parar_motores()
                        elif distancia_esquerda() <= 5:
                            mover(20, 0)
                            time.sleep(0.40)
                            parar_motores()
                            mover(0, 20)
                            time.sleep(0.30)
                            parar_motores()    
                        elif distancia_esquerda() >= 9: 
                            mover(20,20)
                            time.sleep(0.5)
                            mover(0, 20)
                            time.sleep(0.60)
                            parar_motores()
                            mover(20, 0)
                            time.sleep(0.60)
                            parar_motores()
                        print(distancia_esquerda())
                        ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO

                if not achou_canto:
                    if sonar_valido() and distancia_esquerda() > DIST_LATERAL_ABERTA_CM:
                        print(distancia_esquerda())
                        parar_motores()
                        print("[SAIDA] Ultrassonico lateral aberto, confirmando com a camera de baixo...")
                        guinada2('E', 90, 15)
                        if confirmar_saida_lateral():
                            print("[SAIDA] Contorno preto confirmado na abertura lateral, alinhando...")
                            alinhar_na_linha()
                            return "sonar_lateral"
                        else:
                            print("[SAIDA] Nao viu contorno na abertura lateral - falso positivo do sonar")
                            mover(-15, -15)
                            time.sleep(1.3)
                            guinada2("D", 90, 20)
                    else:
                        print("vi uma paredona")
                        print(distancia_frente())
                        parar_motores()
                        mover(25, 25)
                        time.sleep(0.3)
                        mover(-14, -14)
                        time.sleep(0.4)
                        guinada2('D', 90, 15)
                        time.sleep(0.3)
                        aguardar_leitura_fresca_sonar()

            aguardar_leitura_fresca_sonar()
            if sonar_valido() and distancia_esquerda() > DIST_LATERAL_ABERTA_CM:
                print(distancia_esquerda())
                parar_motores()
                print("[SAIDA] Ultrassonico lateral aberto, confirmando com a camera de baixo...")
                mover(20, 20)
                time.sleep(0.3)
                guinada2('E', 90, 15)
                if confirmar_saida_lateral():
                    print("[SAIDA] Contorno preto confirmado na abertura lateral, alinhando...")
                    alinhar_na_linha()
                    return "sonar_lateral"
                else:
                    print("[SAIDA] Nao viu contorno na abertura lateral - falso positivo do sonar")
                    mover(-15, -15)
                    time.sleep(1.3)
                    guinada2('D', 90, 15)

            time.sleep(0.02)
    finally:
        parar_motores()
        ALVO_COR = None


# ===================== BOOT / INICIALIZACAO =====================

threading.Thread(target=carregar_modelo, daemon=True).start()

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)

iniciar_mpu_thread()

motor_serial = serial.Serial(YAHBOOM_PORTA, YAHBOOM_BAUDRATE, timeout=0.5)

threading.Thread(target=watchdog_nudge, daemon=True).start()
threading.Thread(target=watchdog_seguranca, daemon=True).start()
iniciar_serial()

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"Não consegui abrir a câmera de resgate em {CAMERA_INDEX}")
    exit(1)

mover_servo(PINO_PESADO, 150, velocidade=100)

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

threading.Thread(target=botao_thread, daemon=True).start()


# ===================== LOOP PRINCIPAL =====================

try:
    _ultimo_heartbeat[0] = time.time()
    print("Iniciando busca por bolas...")
    while True:
        try:
            checar_parada()
            turn_right(speed=25)
            time.sleep(TURN_PULSE)

            parar_motores()
            time.sleep(0.25)

            for _ in range(4):
                cap.read()

            ret, frame = cap.read()
            if not ret:
                falhas_camera_seguidas += 1
                print(f"Erro na câmera ({falhas_camera_seguidas}/{FALHAS_CAMERA_ANTES_DE_DESISTIR})")
                if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_DESISTIR:
                    print("[CAMERA] Sem resposta ha muitas tentativas, voltando pro seguidor")
                    parar_motores()
                    cap.release()
                    if cap_linha is not None:
                        cap_linha.release()
                    GPIO.gpiochip_close(h)
                    os.execv(sys.executable, [
                        sys.executable,
                        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                        f"--entregas={entregas_realizadas}",
                    ])
                time.sleep(0.2)
                continue
            falhas_camera_seguidas = 0
            if model is None:
                continue

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
                mover(-1,-1)
                time.sleep(0.09)
                parar_motores()
                angulo(0)
                mover(15,15)
                time.sleep(0.15)
                parar_motores()
                mover_servo(PINO_PESADO, 0, velocidade=60)
                mover(20,20)
                time.sleep(0.5)
                parar_motores()
                mover(-20,-20)
                time.sleep(0.5)
                parar_motores()
                mover(20,20)
                time.sleep(0.1)
                parar_motores()
                mover(-20,-20)
                time.sleep(0.1)
                parar_motores()
                mover(-1,-1)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 150, velocidade=20)
                mover(-25,-25)
                time.sleep(0.5)
                parar_motores()
                cor_alvo = CLASSE_COR_ENTREGA.get(cls_id)
                if cor_alvo is None:
                    print(f"Classe {cls_id} não mapeada em CLASSE_COR_ENTREGA, entregando em qualquer área")
                entregar_bola(cor_alvo)
                entregas_realizadas += 1

                if entregas_realizadas >= MAX_ENTREGAS:
                    print(f"[BOOT] {entregas_realizadas} entregas ja feitas, fazendo busca final antes da saida...")
                    if buscar_bola():
                        print("[BUSCA FINAL] Achou bola sobrando, indo pegar e entregar mesmo assim")
                    else:
                        print("[BUSCA FINAL] Nenhuma bola a mais na area, indo pra saida")
                        caso_saida = acionar_saida()
                        print(f"[SAIDA] Caso identificado: {caso_saida}")
                        parar_motores()
                        cap.release()
                        if cap_linha is not None:
                            cap_linha.release()
                        GPIO.gpiochip_close(h)
                        os.execv(sys.executable, [sys.executable, "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py", "--auto"])
                        break
                else:
                    print("Achou mais uma bola na busca final, continuando normalmente")

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
                "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                f"--entregas={entregas_realizadas}",
            ])
        except Exception as e:
            print(f"[ERRO CRITICO] Excecao inesperada no loop de resgate: {e!r}")
            try:
                parar_motores()
            except Exception:
                pass
            time.sleep(0.5)
            continue

except KeyboardInterrupt:
    print("\nInterrompido pelo usuário.")
finally:
    parar_motores()
    GPIO.gpiochip_close(h)
    cap.release()
    if cap_linha is not None:
        cap_linha.release()

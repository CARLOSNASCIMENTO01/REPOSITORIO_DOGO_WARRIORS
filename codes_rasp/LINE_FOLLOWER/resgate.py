import os as _os, sys as _sys
_VENV_PY = "/home/pi/ROBO_DOGO_2026/env_dogo/bin/python3"
if "env_dogo" not in _sys.prefix and not _os.environ.get("ROBO_REEXEC") and _os.path.exists(_VENV_PY):
    _os.environ["ROBO_REEXEC"] = "1"
    _os.execv(_VENV_PY, [_VENV_PY] + _sys.argv)

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
red_min1 = np.array([0, 69, 0])
red_max1 = np.array([8, 255, 255])
red_min2 = np.array([157, 123, 0])
red_max2 = np.array([179, 255, 255])
kernel_tri = np.ones((3, 3), np.uint8)

PROPORCAO_ESPERADA = 30 / 6
TOLERANCIA_PROPORCAO = 0.8
AREA_MIN_TRI = 2500 #500 - baixo demais, deixava passar ruido pequeno (blob de ~510px que nao era triangulo nenhum)
AREA_MAX_TRI = 1000000

AREA_MIN_TRI_SAIDA = 6000

STOP_AREA_RATIO_TRIANGULO = 0.80
VOLTAS_TRIANGULO_NORMAL = 3  # voltas de 360 (giroscopio) na velocidade normal antes de reposicionar
VOLTAS_TRIANGULO_LENTO = 1   # depois delas, mais essas voltas de 360 mais devagar, ai sim reposiciona
VEL_GIRO_TRIANGULO = 16
VEL_GIRO_TRIANGULO_LENTO = 12
TEMPO_MAX_VOLTA_TRIANGULO_LENTO = 40.0  # segundos - so na volta lenta: se algo travar o giro e nao completar 360 no giroscopio, desiste dela depois disso
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_APPROACH_SPEED = 20
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 15
TRIANGULO_CENTER_LOST_FRAMES = 15

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_inox.onnx")
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240

GRAVAR_PERCURSO = False
VIDEO_ROUNDS_DIR = "video_rounds"
_ARQUIVO_ROUND_VIDEO = "/tmp/robo_dogo_video_round.txt"

GRAUS_POR_PASSO_BUSCA = 20
VARREDURAS_360_ANTES_REPOSICIONAR = 2  # quantas voltas de 360 buscar_bola() da (reposicionando entre elas) antes de desistir
MAX_TENTATIVAS_BUSCA_FINAL = 3
CONF_THRES = 0.35

MODO_ANTI_BOLA_FALSA = True #MODO BOLAAAAAAAAAA!!!!!!!!
FILTRO_ALTURA_BOLA = True
Y2_MIN_BOLA_CHAO = 100
FILTRO_PARALAXE_BOLA = True
PARALAXE_CRESCIMENTO_MIN = 1.4  # largura atual / largura inicial pra fazer o teste (chegou bem mais perto)
PARALAXE_DESCIDA_MIN_PX = 15    # quanto o y2 tem que ter descido (px) nesse tempo
PARALAXE_MARGEM_BAIXO_PX = 25   # se a caixa ja comecou colada no fundo do crop, nao da pra descer - nao testa
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

DEBUG_VISUAL = False

YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70
PWM_FREIO_YAHBOOM = 700
TEMPO_FREIO_YAHBOOM = 0.08

VEL_MAX = 20
VEL_MIN = -VEL_MAX

APPROACH_RATIO_LONGE = 0.15
APPROACH_SPEED_LONGE = 45
APPROACH_STEP_TIME_LONGE = 0.28

APPROACH_SPEED = 30#25
APPROACH_STEP_TIME = 0.18#0.18
APPROACH_RATIO_PERTO = 0.5
APPROACH_SPEED_PERTO = 15
APPROACH_STEP_TIME_PERTO = 0.08
STOP_AREA_RATIO = 0.90
MAX_APPROACH_STEPS = 60

LIMIAR_AREA_BOLA_NAO_PEGA = 0.15
MAX_TENTATIVAS_PEGAR_BOLA = 3

_garra_com_captura_nao_entregue = False
LOST_BALL_FRAMES = 8
TURN_PULSE = 0.2#0.16
STOP_TIME = 0.1#0.2
FORWARD_TIME = 0.6

CENTER_TURN_SPEED_MIN = 15
CENTER_TURN_SPEED_MAX = 25
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5

QTD_PRATA_ENTREGAS = 2
QTD_PRETA_ENTREGAS = 1
MAX_ENTREGAS = QTD_PRATA_ENTREGAS + QTD_PRETA_ENTREGAS

ASR_2026 = False
TEMPO_PAUSA_VITIMA_MORTA_ASR = 180  # 3 minutos

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
TOLERANCIA_ANGULO_HORIZONTAL_LINHA = 60 #58  # calibrado ao vivo, era 58

DIST_LATERAL_ABERTA_CM = 30
DIST_PAREDE_FRENTE_CM = 10

SONAR_INVALIDO = 9999
LIMIAR_ERRO_SONAR_CM = 254  # >= isso e o "255.0" de erro do firmware (folga de arredondamento)

FRAMES_CONFIRMACAO_LATERAL = 5

VEL_SAIDA = 15
TEMPO_MAX_CONFIRMAR_LATERAL = 1.0

USBDEVFS_RESET = 21780
USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3

LIMIAR_DESVIO_FRAME_VALIDO = 3.0

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
CLASSE_COR_ENTREGA = {}
CLASSE_POR_NOME = {}
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
    # Interruptor gangorra: cada virada muda o nivel e ele fica la, entao
    # qualquer mudanca (0->1 ou 1->0) liga/desliga - igual config.botao_thread.
    anterior = GPIO.gpio_read(h, BUTTON)
    while True:
        estado = GPIO.gpio_read(h, BUTTON)
        if estado != anterior:
            time.sleep(0.05)  # repique do contato
            estado = GPIO.gpio_read(h, BUTTON)
            if estado != anterior:
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

    porta_motor = motor_serial.port if motor_serial is not None else None

    PALAVRAS_FTDI = ["ftdi", "ft232", "usb serial converter", "ftb6spl3"]
    portas = serial.tools.list_ports.comports()

    for p in portas:
        if p.device == porta_motor:
            continue
        desc = f"{p.description} {p.manufacturer or ''} {p.hwid or ''}".lower()
        if any(palavra in desc for palavra in PALAVRAS_FTDI):
            try:
                s = serial.Serial(p.device, 115200, timeout=0.04)
                print(f"[Serial] FTDI encontrado em: {p.device} - {p.description}")
                return s
            except serial.SerialException:
                continue

    for p in portas:
        if p.device == porta_motor:
            continue
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

def _detectar_porta_yahboom(sair_se_falhar=True):
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) == (0x1a86, 0x7522):
            try:
                s = serial.Serial(p.device, YAHBOOM_BAUDRATE, timeout=0.5)
                print(f"[MOTOR] Placa Yahboom encontrada em: {p.device}")
                return s
            except serial.SerialException:
                continue

    if sair_se_falhar:
        print("[MOTOR][ERRO] Placa Yahboom nao encontrada!")
        sys.exit(1)
    return None

def _aguardar_porta_yahboom(intervalo=1.0):
    porta = _detectar_porta_yahboom(sair_se_falhar=False)
    tentativas = 0
    while porta is None:
        tentativas += 1
        print(f"[MOTOR] Placa Yahboom ainda nao apareceu, tentando de novo... ({tentativas})")
        time.sleep(intervalo)
        porta = _detectar_porta_yahboom(sair_se_falhar=False)
    print("[MOTOR] Placa Yahboom encontrada, esperando ela terminar de subir...")
    time.sleep(2.0)
    print("[MOTOR] Pronta")
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
    valor = dados["F"]
    return SONAR_INVALIDO if valor >= LIMIAR_ERRO_SONAR_CM else valor

def distancia_esquerda():
    valor = dados["E"]
    return SONAR_INVALIDO if valor >= LIMIAR_ERRO_SONAR_CM else valor

def lateral_aberta():
    d = distancia_esquerda()
    return sonar_valido() and DIST_LATERAL_ABERTA_CM < d < SONAR_INVALIDO

def sonar_valido():
    return (time.time() - ultimo_recebimento) < SERIAL_TIMEOUT_DADO

def recuar_ate_ver_parede(vel_recuo=15, timeout=2.0):
    inicio = time.time()
    while lateral_aberta() and time.time() - inicio < timeout:
        checar_parada()
        mover(-vel_recuo, -vel_recuo)
        time.sleep(0.02)
    parar_motores()
    mover(vel_recuo, vel_recuo)
    time.sleep(0.6)
    parar_motores()

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
        CLASSE_POR_NOME[_normalizar_nome(nome)] = cls_id
        cor = nome_cor_norm.get(_normalizar_nome(nome))
        if cor is None:
            print(f"Classe '{nome}' (id={cls_id}) nao esta em NOME_COR_ENTREGA, "
                  f"bolas dessa classe vao ser entregues em qualquer area")
        else:
            CLASSE_COR_ENTREGA[cls_id] = cor
    print("Mapeamento classe -> cor de entrega:", CLASSE_COR_ENTREGA)
    print("Mapeamento nome -> classe:", CLASSE_POR_NOME)

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
    pwm_esq = -velL * ESCALA_PWM_YAHBOOM
    pwm_dir = -velR * ESCALA_PWM_YAHBOOM
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

# Descida da garra pra pegar a bola: abaixo do 0 (pulso < 500us) pra descer
# mais - testado no controle pelo teclado com -6.
ANGULO_GARRA_BAIXO = -6


def descer_garra_toda():
    """Desce o servo pesado ate 0 (suave) e depois pro ANGULO_GARRA_BAIXO.
    O tx_servo do lgpio nao aceita pulso < 500us, entao abaixo de 0 usa PWM de
    50 Hz. Fica segurando a forca la embaixo ate o proximo mover_servo (que
    volta pro tx_servo normal)."""
    mover_servo(PINO_PESADO, 0, velocidade=100)
    if ANGULO_GARRA_BAIXO >= 0:
        mover_servo_direto(PINO_PESADO, ANGULO_GARRA_BAIXO, manter_pulso=True)
        return
    pulso = 500 + (ANGULO_GARRA_BAIXO / 180.0) * 2000
    try:
        lgpio.tx_pwm(h, PINO_PESADO, 50, pulso / 20000 * 100)
    except lgpio.error as e:
        print(f"Falha ao descer garra abaixo de 0 (pulso={pulso:.0f}): {e}")
    _angulo_atual[PINO_PESADO] = 0  # proximo mover_servo parte do 0
    time.sleep(0.15)


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

def _id_do_round_video():
    if os.path.exists(_ARQUIVO_ROUND_VIDEO):
        try:
            with open(_ARQUIVO_ROUND_VIDEO) as f:
                round_id = f.read().strip()
            if round_id:
                return round_id
        except Exception as e:
            print(f"[GRAVACAO] Falha ao ler round salvo ({e}), gerando novo")
    round_id = time.strftime("%Y-%m-%d_%H-%M-%S")
    try:
        with open(_ARQUIVO_ROUND_VIDEO, "w") as f:
            f.write(round_id)
    except Exception as e:
        print(f"[GRAVACAO] Falha ao salvar round ({e})")
    return round_id

_writer_gravacao = None

def iniciar_gravacao_percurso():
    global _writer_gravacao
    if not GRAVAR_PERCURSO:
        return
    try:
        os.makedirs(VIDEO_ROUNDS_DIR, exist_ok=True)
        round_id = _id_do_round_video()
        n = 1
        while True:
            caminho = os.path.join(VIDEO_ROUNDS_DIR, f"{round_id}_resgate_{n}.mp4")
            if not os.path.exists(caminho):
                break
            n += 1
        largura = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
        altura = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
        _writer_gravacao = cv2.VideoWriter(caminho, cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (largura, altura))
        print(f"[GRAVACAO] Gravando percurso (resgate) em {caminho}")
    except Exception as e:
        print(f"[GRAVACAO][ERRO] Falha ao iniciar gravacao: {e}")

def _escrever_frame_gravacao(frame):
    if _writer_gravacao is not None and frame is not None:
        try:
            _writer_gravacao.write(frame)
        except Exception:
            pass

def parar_gravacao_percurso():
    global _writer_gravacao
    if _writer_gravacao is not None:
        try:
            _writer_gravacao.release()
        except Exception:
            pass
        _writer_gravacao = None

def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()

TEMPO_FRAME_PARADO_LEVANTA_GARRA = 3.0  # segundos com a imagem parada (robo travado) pra levantar a garra
LIMIAR_DIFF_FRAME_PARADO = 5.0  # diferenca media de pixel (0-255) abaixo disso = frame "igual" ao de referencia
ANGULO_GARRA_LEVANTADA = 150

_frame_parado_ref = None
_frame_parado_desde = time.time()


def _checar_frame_parado(frame):
    """Se a imagem da camera do resgate ficar TEMPO_FRAME_PARADO_LEVANTA_GARRA
    segundos sem mudar (robo travado/encostado), levanta a garra (servo
    pesado). Chamar a cada frame lido."""
    global _frame_parado_ref, _frame_parado_desde
    pequeno = cv2.cvtColor(cv2.resize(frame, (160, 90)), cv2.COLOR_BGR2GRAY)
    agora = time.time()

    if _frame_parado_ref is None or np.mean(cv2.absdiff(pequeno, _frame_parado_ref)) >= LIMIAR_DIFF_FRAME_PARADO:
        _frame_parado_ref = pequeno
        _frame_parado_desde = agora
        return

    if agora - _frame_parado_desde >= TEMPO_FRAME_PARADO_LEVANTA_GARRA:
        print(f"[TRAVADO] Imagem parada ha {agora - _frame_parado_desde:.1f}s, levantando a garra")
        mover_servo_direto(PINO_PESADO, ANGULO_GARRA_LEVANTADA, manter_pulso=True)
        time.sleep(0.6)
        parar_servo(PINO_PESADO)
        _frame_parado_ref = None
        _frame_parado_desde = time.time()


def capturar_crop():
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    _escrever_frame_gravacao(frame)
    _checar_frame_parado(frame)
    return center_crop(frame, CROP_W, CROP_H)

def classe_bola_alvo():
    nome_alvo = "bola-prata" if entregas_realizadas < QTD_PRATA_ENTREGAS else "bola-preta"
    return CLASSE_POR_NOME.get(nome_alvo)

def so_falta_uma_bola_do_alvo():
    if entregas_realizadas >= QTD_PRATA_ENTREGAS:
        return True
    return entregas_realizadas == QTD_PRATA_ENTREGAS - 1

def escolher_alvo(boxes):
    classe_prata = CLASSE_POR_NOME.get("bola-prata")
    if classe_prata is not None and boxes is not None:
        idx_prata = escolher_indice_mais_perto(boxes, classe_prata)
        if idx_prata is not None:
            return idx_prata, classe_prata

    classe_desejada = classe_bola_alvo()
    idx = escolher_indice_mais_perto(boxes, classe_desejada) if boxes is not None else None
    return idx, classe_desejada


def escolher_indice_mais_perto(boxes, classe_desejada=None):
    if boxes is None or len(boxes) == 0:
        return None
    maior_area = -1
    melhor_idx = None
    tem_cls = getattr(boxes, "cls", None) is not None
    for i in range(len(boxes)):
        if classe_desejada is not None:
            cls_i = int(boxes.cls[i].item()) if tem_cls else None
            if cls_i != classe_desejada:
                continue
        x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
        # (1) base da caixa alta demais na imagem = nao esta no chao (foto na parede)
        if MODO_ANTI_BOLA_FALSA and FILTRO_ALTURA_BOLA and y2 < Y2_MIN_BOLA_CHAO:
            print(f"[FILTRO ALTURA] Caixa descartada: y2={y2} < {Y2_MIN_BOLA_CHAO} (alta demais, foto na parede?)")
            continue
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
        for _ in range(3):
            ret, frame = cap_local.read()
            if not ret:
                break
            if frame is not None and frame.std() > LIMIAR_DESVIO_FRAME_VALIDO:
                return cap_local
        cap_local.release()
    return None

def _resolver_dev_real(fonte):
    if isinstance(fonte, int):
        fonte = f"/dev/video{fonte}"
    try:
        return os.path.realpath(fonte)
    except Exception:
        return None


def _e_camera_de_resgate(fonte):
    alvo = _resolver_dev_real(fonte)
    resgate_dev = _resolver_dev_real(CAMERA_INDEX)
    return alvo is not None and alvo == resgate_dev


def _detectar_camera_linha():
    if CAMERA_INDEX_LINHA:
        checar_parada()
        c = _abrir_camera_linha_generica(CAMERA_INDEX_LINHA)
        if c is not None:
            print(f"[Camera] Linha conectada em {CAMERA_INDEX_LINHA}")
            return c

    checar_parada()
    if not _e_camera_de_resgate(CAMERA_LINHA_INDEX_FALLBACK):
        c = _abrir_camera_linha_generica(CAMERA_LINHA_INDEX_FALLBACK)
        if c is not None:
            print(f"[Camera] Linha conectada no indice {CAMERA_LINHA_INDEX_FALLBACK}")
            return c

    for i in range(10):
        checar_parada()
        if _e_camera_de_resgate(i):
            continue
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

def _abrir_camera_resgate_generica(fonte):
    cap_local = cv2.VideoCapture(fonte, cv2.CAP_V4L2)
    if cap_local.isOpened():
        for _ in range(3):
            ret, frame = cap_local.read()
            if not ret:
                break
            if frame is not None and frame.std() > LIMIAR_DESVIO_FRAME_VALIDO:
                return cap_local
        cap_local.release()
    return None

def _detectar_camera_resgate():
    checar_parada()
    c = _abrir_camera_resgate_generica(CAMERA_INDEX)
    if c is not None:
        print(f"[Camera] Resgate conectada em {CAMERA_INDEX}")
        return c

    for i in range(10):
        checar_parada()
        c = _abrir_camera_resgate_generica(i)
        if c is not None:
            print(f"[Camera] Resgate conectada em /dev/video{i}")
            return c
    return None

def abrir_camera_resgate():
    tentativas = 0
    cap_local = _detectar_camera_resgate()
    while cap_local is None:
        tentativas += 1
        print(f"[Camera] Resgate ainda nao apareceu, tentando de novo... (tentativa {tentativas})")
        checar_parada()
        time.sleep(1)
        if tentativas % USB_RESET_TENTATIVAS_ANTES_DE_RESETAR == 0:
            print("[Camera] Varias falhas seguidas, tentando resetar USB...")
            resetar_usb_camera_linha(CAMERA_INDEX)
        cap_local = _detectar_camera_resgate()
    return cap_local

def _mask_linha_baixo(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)
    return mask

def _maior_contorno_linha(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maior = None
    if contours:
        candidato = max(contours, key=cv2.contourArea)
        area_contorno = cv2.contourArea(candidato)
        area_roi = roi.shape[0] * roi.shape[1]
        if AREA_MIN_LINHA <= area_contorno < area_roi * AREA_MAX_LINHA_FRAC:
            maior = candidato

    if DEBUG_VISUAL:
        debug_img = roi.copy()
        if maior is not None:
            cv2.drawContours(debug_img, [maior], -1, (0, 255, 0), 2)
        cv2.imshow("SAIDA - camera de baixo (vertical)", debug_img)
        cv2.imshow("SAIDA - mascara vertical", mask)
        cv2.waitKey(1)

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
    maior = None
    if contours:
        candidato = max(contours, key=cv2.contourArea)
        area_contorno = cv2.contourArea(candidato)
        area_roi = roi.shape[0] * roi.shape[1]
        if AREA_MIN_LINHA_H <= area_contorno < area_roi * AREA_MAX_LINHA_FRAC_H:
            maior = candidato

    if DEBUG_VISUAL:
        debug_img = roi.copy()
        if maior is not None:
            cv2.drawContours(debug_img, [maior], -1, (255, 0, 0), 2)
        cv2.imshow("SAIDA - camera de baixo (horizontal)", debug_img)
        cv2.imshow("SAIDA - mascara horizontal", mask)
        cv2.waitKey(1)

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
            if ret_l3 and detectar_linha_horizontal_embaixo(frame_l3):
                achou = True
                break
        time.sleep(0.02)
    parar_motores()
    return achou

def alinhar_na_linha():

    FRAMES_CONFIRMACAO = 3
    ALINHAR_VEL = 13
    frames_detectados_seguidos = 0
    while True:
        checar_parada()
        mover(ALINHAR_VEL, -ALINHAR_VEL)
        ret_l, frame_l = cap_linha.read()
        detectado = ret_l and detectar_linha_embaixo(frame_l)
        frames_detectados_seguidos = frames_detectados_seguidos + 1 if detectado else 0
        print(f"[SAIDA-ALINHA] detectado={detectado} seguidos={frames_detectados_seguidos}")
        if frames_detectados_seguidos >= FRAMES_CONFIRMACAO:
            print("[SAIDA] Contorno da linha encontrado de novo, alinhado")
            break
        time.sleep(0.005)
    parar_motores()
    guinada2('D', 10, 20)


# ===================== FUNCOES DE BOLA =====================

def detectar_bola(crop, classe_desejada=None):
    results = model.predict(
        crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False, show=False
    )
    r = results[0]
    boxes = getattr(r, "boxes", None)

    idx = escolher_indice_mais_perto(boxes, classe_desejada) if boxes is not None else None
    if idx is not None:
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

def centralizar_bola(classe_desejada=None):
    perdida = 0
    for _ in range(CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop, classe_desejada)

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
    mover(-20, -20)
    time.sleep(0.15)
    guinada2('E', 5, 15)
    parar_motores()
    return False

def aproximar_bola(classe_desejada=None):
    crop_area = CROP_W * CROP_H
    perdida = 0
    ref_paralaxe = None  # (largura, y2) da primeira deteccao - pro teste de paralaxe
    paralaxe_testada = False

    for passo in range(MAX_APPROACH_STEPS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            print("Erro na câmera durante aproximação")
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop, classe_desejada)

        if cx is None:
            perdida += 1
            print(f"Bola sumiu durante aproximação ({perdida}/{LOST_BALL_FRAMES})")
            if perdida >= LOST_BALL_FRAMES:
                print("Perdeu a bola de vez, abortando aproximação")
                mover(-20, -20)
                time.sleep(0.15)
                guinada2('E', 5, 15)
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"passo={passo} area_ratio={ratio:.2f} cx={cx}")

        # (6) Paralaxe: chegou bem mais perto (caixa cresceu) mas a base nao
        # desceu na imagem = nao esta no chao, e foto na parede.
        if MODO_ANTI_BOLA_FALSA and FILTRO_PARALAXE_BOLA and not paralaxe_testada and bbox is not None:
            bx1, _by1, bx2, by2 = bbox
            largura = max(bx2 - bx1, 1)
            if ref_paralaxe is None:
                ref_paralaxe = (largura, by2)
                if by2 >= CROP_H - PARALAXE_MARGEM_BAIXO_PX:
                    paralaxe_testada = True  # ja colada no fundo, nao tem como descer
            elif largura >= ref_paralaxe[0] * PARALAXE_CRESCIMENTO_MIN:
                paralaxe_testada = True
                descida = by2 - ref_paralaxe[1]
                print(f"[PARALAXE] largura {ref_paralaxe[0]}->{largura}, y2 {ref_paralaxe[1]}->{by2} (desceu {descida}px, minimo {PARALAXE_DESCIDA_MIN_PX})")
                if descida < PARALAXE_DESCIDA_MIN_PX:
                    print("[PARALAXE] Chegou perto mas a bola nao desceu na imagem - foto na parede, desistindo")
                    parar_motores()
                    mover(-20, -20)
                    time.sleep(0.3)
                    parar_motores()
                    return False

        if ratio >= STOP_AREA_RATIO:
            print(f"[APROXIMAR] area_ratio={ratio:.2f} bateu o limiar, confirmando de novo...")
            parar_motores()
            time.sleep(0.1)
            crop_confirma = capturar_crop()
            ratio_confirma = None
            if crop_confirma is not None:
                cx_confirma, area_confirma, _, _ = detectar_bola(crop_confirma, classe_desejada)
                if cx_confirma is not None:
                    ratio_confirma = area_confirma / crop_area

            if ratio_confirma is not None and ratio_confirma >= STOP_AREA_RATIO:
                print(f"[APROXIMAR] Confirmado (area_ratio={ratio_confirma:.2f}), perto o suficiente pra garra")
                return True

            print(f"[APROXIMAR] Nao confirmou (area_ratio={ratio_confirma}), continuando aproximacao")
            continue

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximação, recentralizando")
            centralizada = centralizar_bola(classe_desejada)
            if not centralizada:
                print("Perdeu a bola tentando recentralizar na aproximação")
                return False
            continue

        if ratio < APPROACH_RATIO_LONGE:
            forward(speed=APPROACH_SPEED_LONGE)
            time.sleep(APPROACH_STEP_TIME_LONGE)
        elif ratio < APPROACH_RATIO_PERTO:
            forward(speed=APPROACH_SPEED)
            time.sleep(APPROACH_STEP_TIME)
        else:
            forward(speed=APPROACH_SPEED_PERTO)
            time.sleep(APPROACH_STEP_TIME_PERTO)
        parar_motores()
        time.sleep(0.05)

    print("Estourou o limite de passos na aproximação")
    parar_motores()
    return False

def bola_ainda_no_chao(classe_desejada):
    crop = capturar_crop()
    if crop is None:
        return False
    cx, area, bbox, cls_id = detectar_bola(crop, classe_desejada)
    if cx is None:
        return False

    if so_falta_uma_bola_do_alvo():
        print("[GARRA-CHECK] So falta 1 bola dessa classe no campo - detectada de novo, garra errou (independente da distancia)")
        return True

    ratio = area / (CROP_W * CROP_H)
    print(f"[GARRA-CHECK] area_ratio={ratio:.2f} apos fechar garra (limiar={LIMIAR_AREA_BOLA_NAO_PEGA})")
    return ratio >= LIMIAR_AREA_BOLA_NAO_PEGA

def soltar_bola_de_seguranca():
    print("[SEGURANCA] Captura anterior nunca foi entregue - soltando a garra antes de procurar de novo")
    checar_parada()
    mover_servo(PINO_PESADO, 50, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)
    mover_servo(PINO_SG90, 180, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)
    mover_servo(PINO_SG90, 180, velocidade=100)
    mover_servo(PINO_PESADO, 140, velocidade=100)

def pegar_e_entregar_bola(classe_desejada):
    global entregas_realizadas, _garra_com_captura_nao_entregue

    for tentativa in range(1, MAX_TENTATIVAS_PEGAR_BOLA + 1):
        if _garra_com_captura_nao_entregue:
            soltar_bola_de_seguranca()
            _garra_com_captura_nao_entregue = False

        if not centralizar_bola(classe_desejada):
            print("Não conseguiu centralizar, voltando pra busca")
            return False

        print("⬆Centralizado → avançando")
        if not aproximar_bola(classe_desejada):
            print("Não conseguiu se aproximar, voltando pra busca")
            return False

        crop_final = capturar_crop()
        cls_id = None
        if crop_final is not None:
            _cx_final, _area_final, _bbox_final, cls_id = detectar_bola(crop_final, classe_desejada)

        nome_classe_final = model.names.get(cls_id, "desconhecida") if cls_id is not None else "desconhecida"
        print(f"Bola confirmada na garra: classe={cls_id} ({nome_classe_final})")

        print("Fechando garra na bola")
        checar_parada()
        mover(-1,-1)
        time.sleep(0.09)
        parar_motores()
        angulo(180)  # 0
        mover(15,15)
        time.sleep(0.15)
        parar_motores()
        descer_garra_toda()  # era mover_servo(PINO_PESADO, -2) - o -2 virava 0 (mover_servo corta em 0)
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
        mover_servo(PINO_PESADO, 150, velocidade=100)
        mover(-25,-25)
        time.sleep(0.5)
        parar_motores()
        _garra_com_captura_nao_entregue = True

        if bola_ainda_no_chao(classe_desejada):
            print(f"[GARRA] Bola ainda no chao apos tentativa {tentativa}/{MAX_TENTATIVAS_PEGAR_BOLA} - garra errou, tentando pegar de novo")
            continue

        cor_alvo = CLASSE_COR_ENTREGA.get(cls_id)
        if cor_alvo is None:
            print(f"Classe {cls_id} não mapeada em CLASSE_COR_ENTREGA, entregando em qualquer área")
        entregar_bola(cor_alvo)
        _garra_com_captura_nao_entregue = False
        entregas_realizadas += 1
        return True

    print(f"[GARRA] Nao conseguiu confirmar a bola na garra depois de {MAX_TENTATIVAS_PEGAR_BOLA} tentativas, desistindo dela")
    return False

_classe_bola_confirmada = None

def _detectar_bola_qualquer(crop):
    results = model.predict(crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False, show=False)
    boxes = getattr(results[0], "boxes", None)
    idx, classe = escolher_alvo(boxes)
    if idx is None:
        return None, None
    x1, y1, x2, y2 = map(int, boxes.xyxy[idx].tolist())
    return (x1 + x2) // 2, classe

def _bola_confirmada(confirmacoes_necessarias):
    global _classe_bola_confirmada
    crop = capturar_crop()
    if crop is None:
        return False

    cx, classe_vista = _detectar_bola_qualquer(crop)
    if cx is None:
        return False

    confirmacoes = 1
    for _ in range(confirmacoes_necessarias - 1):
        checar_parada()
        crop_confirma = capturar_crop()
        if crop_confirma is None:
            break
        cx_confirma, _, _, _ = detectar_bola(crop_confirma, classe_vista)
        if cx_confirma is None:
            break
        confirmacoes += 1

    if confirmacoes < confirmacoes_necessarias:
        print(f"[BUSCA FINAL] Deteccao efemera descartada ({confirmacoes}/{confirmacoes_necessarias} frames), continuando busca")
        return False
    _classe_bola_confirmada = classe_vista
    return True


def _ler_heading_ou_none():
    try:
        return _ler_heading_bruto()
    except Exception as e:
        print(f"[BUSCA] Falha ao ler heading ({e})")
        return None


def _graus_girados(antes, depois, padrao=GRAUS_POR_PASSO_BUSCA):
    """Quanto o robo girou DE VERDADE num passo de busca, pelo heading bruto
    do BNO055 - ele nao e zerado pelo resetar_giroscopio() da guinada2, entao
    conta tambem a inercia depois do freio (na pratica cada passo de 20 graus
    gira ~30). Sem leitura valida, conta o passo como `padrao`."""
    if antes is None or depois is None:
        return padrao
    return _normalizar(depois - antes)


def buscar_bola():
    CONFIRMACAO_BUSCA_FINAL = 3

    checar_parada()
    if _bola_confirmada(CONFIRMACAO_BUSCA_FINAL):
        return True

    girado = 0.0
    while girado < 360:
        checar_parada()
        antes = _ler_heading_ou_none()
        guinada2('D', GRAUS_POR_PASSO_BUSCA, 20)

        if _bola_confirmada(CONFIRMACAO_BUSCA_FINAL):
            return True
        girado += _graus_girados(antes, _ler_heading_ou_none())

    return False


_varredura_prata_feita = False

def varrer_360_por_prata():
    while model is None:
        checar_parada()
        time.sleep(0.1)
    classe_prata = CLASSE_POR_NOME.get("bola-prata")
    if classe_prata is None:
        return

    print("[BUSCA PRATA] Uma volta procurando bola prata antes da preta...")
    girado = 0.0
    while girado < 360:
        checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, _, _, _ = detectar_bola(crop, classe_prata)
            if cx is not None:
                print("[BUSCA PRATA] Achou prata, pegando...")
                if pegar_e_entregar_bola(classe_prata):
                    girado = 0.0
                    continue
        antes = _ler_heading_ou_none()
        guinada2('D', GRAUS_POR_PASSO_BUSCA, 20)
        time.sleep(0.25)
        girado += _graus_girados(antes, _ler_heading_ou_none())
    print("[BUSCA PRATA] Volta completa sem prata, indo atras da preta")

def verificar_e_afastar_bola_bloqueio():
    crop = capturar_crop()
    if crop is None:
        return

    cx, area, bbox, cls_id = detectar_bola(crop)

    if DEBUG_VISUAL:
        debug_img = crop.copy()
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(debug_img, "BLOQUEANDO", (x1, max(y1 - 5, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("ENTREGA - checando bola bloqueando", debug_img)
        cv2.waitKey(1)

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

def _eh_triangulo(contorno, encostando_borda=False, area_min=AREA_MIN_TRI):
    area = cv2.contourArea(contorno)
    if area < area_min or area > AREA_MAX_TRI:
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

def detectar_entrega(crop, area_min=AREA_MIN_TRI):
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

            ok, info = _eh_triangulo(c, encostando_borda=encostando_borda, area_min=area_min)
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
        time.sleep(0.8)
        print("haborgue")
        parar_motores()
    else:
        mover(20, 20)
        time.sleep(0.7)
        mover(-20, 0)
        time.sleep(0.7)
        parar_motores()

def reposicionar_brusco():
    mover(30, 30)
    time.sleep(0.7)
    parar_motores()
    time.sleep(0.1)
    guinada2('D', 45, 25)
    mover(30, 30)
    time.sleep(0.8)
    parar_motores()

def _volta_procurando_triangulo(velocidade, tempo_max=None):
    """Uma volta de 360 (medida no giroscopio) girando intermitente -
    turn_right por 0.3s, olha a camera, repete. Termina ao completar 360 OU,
    se tempo_max for passado, ao passar tempo_max segundos (se algo travar o
    giro). True se achou o triangulo."""
    girado = 0.0
    heading_anterior = _ler_heading_ou_none()
    inicio = time.time()
    while True:
        if girado >= 360:
            return False
        if tempo_max is not None and time.time() - inicio >= tempo_max:
            print(f"[BUSCAR_TRIANGULO] Volta estourou {tempo_max:.0f}s com so {girado:.0f} graus girados (travado?), seguindo")
            return False
        checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, area, info = detectar_entrega(crop)
            if cx is not None:
                cor_dbg = info["cor"] if info else "?"
                area_dbg = info["area"] if info else area
                print(f"[BUSCAR_TRIANGULO] Detectado cor={cor_dbg} area={area_dbg} cx={cx}")
                return True
        turn_right(speed=velocidade)
        time.sleep(0.3)
        # Mede de leitura em leitura (nao antes/depois do turn_right) - o
        # motor continua ligado enquanto a camera le, e esse giro tambem conta.
        heading_atual = _ler_heading_ou_none()
        girado += _graus_girados(heading_anterior, heading_atual, padrao=4.0)
        heading_anterior = heading_atual


def buscar_triangulo():
    for volta in range(VOLTAS_TRIANGULO_NORMAL):
        print(f"[BUSCAR_TRIANGULO] Volta {volta + 1}/{VOLTAS_TRIANGULO_NORMAL} (vel {VEL_GIRO_TRIANGULO})")
        if _volta_procurando_triangulo(VEL_GIRO_TRIANGULO):
            return True
    for volta in range(VOLTAS_TRIANGULO_LENTO):
        print(f"[BUSCAR_TRIANGULO] Volta lenta {volta + 1}/{VOLTAS_TRIANGULO_LENTO} (vel {VEL_GIRO_TRIANGULO_LENTO})")
        if _volta_procurando_triangulo(VEL_GIRO_TRIANGULO_LENTO, TEMPO_MAX_VOLTA_TRIANGULO_LENTO):
            return True
    print("[BUSCAR_TRIANGULO] Nao achou nas voltas, reposicionando...")
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
    parar_motores()          # <-- adiciona os parêntese
    mover_servo(PINO_PESADO, 20, velocidade=100)
    mover_servo(PINO_PESADO, 50, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)  # 60
    mover_servo(PINO_SG90, 180, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)
    mover_servo(PINO_SG90, 180, velocidade=100)
    #oscilar_servo_lgpio(PINO_PESADO, 30, 60,2,0.2)
    mover_servo(PINO_PESADO, 140, velocidade=100)
    mover(-30,-30)
    time.sleep(1.25)
    parar_motores()          # <-- adiciona essa também, no final da ré
    print("✅ Bola entregue")
    ALVO_COR = None
    return True


# ===================== SAIDA =====================

def ir_pra_saida():
    """A saida (ir ate o canto vermelho + sair da sala) mora no
    minha_saida.py, processo proprio - fecha tudo e passa pra ele, levando a
    contagem de entregas."""
    print("[SAIDA] Indo pro minha_saida.py")
    parar_motores()
    parar_gravacao_percurso()
    cap.release()
    if cap_linha is not None:
        cap_linha.release()
    GPIO.gpiochip_close(h)
    os.execv(sys.executable, [
        sys.executable,
        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/minha_saida.py",
        f"--entregas={entregas_realizadas}",
    ])

def finalizar_resgate():
    print(f"[BOOT] {entregas_realizadas} entregas ja feitas, fazendo busca final antes da saida...")
    for _ in range(MAX_TENTATIVAS_BUSCA_FINAL):
        if not buscar_bola():
            break
        classe_vista = _classe_bola_confirmada if _classe_bola_confirmada is not None else classe_bola_alvo()
        print(f"[BUSCA FINAL] Achou bola sobrando (classe {classe_vista}), tentando pegar e entregar...")
        if pegar_e_entregar_bola(classe_vista):
            print(f"[BUSCA FINAL] Entrega extra concluida ({entregas_realizadas} no total), verificando se sobrou mais alguma")
        else:
            print("[BUSCA FINAL] Deteccao nao se confirmou (bola sumiu/falso positivo), tentando de novo")

    print("[BUSCA FINAL] Nenhuma bola a mais na area, acionando a saida...")

    if ASR_2026:
        print(f"[ASR_2026] Vitima morta ja entregue e busca final concluida - parado {TEMPO_PAUSA_VITIMA_MORTA_ASR}s (regra 2026)")
        parar_motores()
        for _ in range(TEMPO_PAUSA_VITIMA_MORTA_ASR):
            checar_parada()
            time.sleep(1)
        print("[ASR_2026] Pausa terminada, acionando a saida")

    ir_pra_saida()

def voltar_pro_seguidor(motivo="Robo parado pelo botao, voltando pro seguidor..."):
    print(motivo)
    parar_motores()
    parar_gravacao_percurso()
    cap.release()
    if cap_linha is not None:
        cap_linha.release()
    GPIO.gpiochip_close(h)
    os.execv(sys.executable, [
        sys.executable,
        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
        f"--entregas={entregas_realizadas}",
    ])

# ===================== BOOT / INICIALIZACAO =====================

threading.Thread(target=carregar_modelo, daemon=True).start()

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)

iniciar_mpu_thread()

motor_serial = _aguardar_porta_yahboom()

threading.Thread(target=watchdog_nudge, daemon=True).start()
threading.Thread(target=watchdog_seguranca, daemon=True).start()
iniciar_serial()

cap = abrir_camera_resgate()
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

iniciar_gravacao_percurso()

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

if entregas_realizadas >= MAX_ENTREGAS:
    try:
        finalizar_resgate()
    except RoboParado:
        voltar_pro_seguidor()

# ===================== LOOP PRINCIPAL =====================

try:
    _ultimo_heartbeat[0] = time.time()
    print("Iniciando busca por bolas...")
    graus_busca_padrao = 0.0
    while True:
        try:
            checar_parada()

            classe_preta = CLASSE_POR_NOME.get("bola-preta")
            if not _varredura_prata_feita and classe_preta is not None and classe_bola_alvo() == classe_preta:
                varrer_360_por_prata()
                _varredura_prata_feita = True
                graus_busca_padrao = 0.0
                if entregas_realizadas >= MAX_ENTREGAS:
                    finalizar_resgate()
                    break

            heading_antes = _ler_heading_ou_none()
            guinada2('D', GRAUS_POR_PASSO_BUSCA, 20)
            time.sleep(0.25)

            graus_busca_padrao += _graus_girados(heading_antes, _ler_heading_ou_none())
            if graus_busca_padrao >= 360 * VARREDURAS_360_ANTES_REPOSICIONAR:
                print(f"[BUSCA] {VARREDURAS_360_ANTES_REPOSICIONAR} voltas de 360 sem achar NENHUMA bola, reposicionamento brusco...")
                reposicionar_brusco()
                graus_busca_padrao = 0.0

            for _ in range(4):
                cap.read()

            ret, frame = cap.read()
            if ret:
                _escrever_frame_gravacao(frame)
                _checar_frame_parado(frame)
            if not ret:
                falhas_camera_seguidas += 1
                print(f"Erro na câmera ({falhas_camera_seguidas}/{FALHAS_CAMERA_ANTES_DE_DESISTIR})")
                if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_DESISTIR:
                    print("[CAMERA] Sem resposta ha muitas tentativas, voltando pro seguidor")
                    parar_motores()
                    parar_gravacao_percurso()
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

            idx, classe_desejada = escolher_alvo(boxes)
            if idx is not None:
                xyxy = boxes.xyxy[idx].tolist()
                x1, y1, x2, y2 = map(int, xyxy)
                cx = (x1 + x2) // 2
                cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
                nome_classe = model.names.get(cls_id, "desconhecida") if cls_id is not None else "desconhecida"
                print(f"Bola detectada em cx={cx} | classe={cls_id} ({nome_classe}) | procurando={nome_classe}")

                if not pegar_e_entregar_bola(classe_desejada):
                    continue
                graus_busca_padrao = 0.0

                if entregas_realizadas >= MAX_ENTREGAS:
                    finalizar_resgate()
                    break
                else:
                    print(f"Entrega {entregas_realizadas}/{MAX_ENTREGAS} concluida, continuando busca normal")

            else:
                nome_desejado = model.names.get(classe_desejada, "?") if model is not None else "?"
                print(f" Nenhuma bola {nome_desejado} — novo passo de giro")
        except RoboParado:
            voltar_pro_seguidor()
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
    parar_gravacao_percurso()
    GPIO.gpiochip_close(h)
    cap.release()
    if cap_linha is not None:
        cap_linha.release()

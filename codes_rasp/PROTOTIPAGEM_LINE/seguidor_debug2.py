"""
ROBO_DOGO_2026 - Robo seguidor de linha (OBR)
Reorganizado em secoes, sem mudar a logica de pilotagem original.

Mudancas de comportamento pedidas (nao sao bugfix, sao features novas):
  1) Botao vira TOGGLE: aperta -> para tudo imediatamente (mesmo dentro de
     guinada/desvio/resgate). Aperta de novo -> reseta estado e reroda o
     loop principal do zero.
  2) Parada imediata em qualquer ponto do codigo via excecao RoboParado,
     checada a cada iteracao dos while True internos.
  3) Boot mais rapido: camera, serial e calibracao do MPU rodam em paralelo.
  4) Se a porta serial nao aparecer na hora do boot (arduino ainda enumerando
     na USB), fica tentando em vez de matar o processo.
  5) Erros inesperados no loop principal nao derrubam o programa: sao
     logados e o robo tenta rodar de novo (retry). Isso completa o
     Restart=always do systemd (ver robo_dogo.service).
  6) Reset de USB via software (ioctl USBDEVFS_RESET) quando uma camera
     falha ao abrir ou trava - nao precisa tirar/colocar o cabo na mao.
"""

import cv2
import numpy as np
import lgpio as GPIO
import time
import serial
import math
import serial.tools.list_ports
import sys
import os
import fcntl
import subprocess
import smbus2
import threading
from threading import Thread

try:
    from ultralytics import YOLO
    YOLO_DISPONIVEL = True
except ImportError:
    YOLO_DISPONIVEL = False
    print("[VITIMA][AVISO] ultralytics nao instalado, deteccao de vitima desativada")

# =========================================================================
# EXCECAO DE PARADA IMEDIATA
# =========================================================================
class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar QUALQUER loop na hora."""
    pass

robo_ativo = threading.Event()
robo_ativo.set()  # comeca ligado

def checar_parada():
    """Chamar a cada iteracao de todo while True que segura o robo por mais
    de um instante (guinadas, desvio, resgate, etc). Custo desprezivel."""
    if not robo_ativo.is_set():
        raise RoboParado()


# =========================================================================
# CONFIG (NAO ALTERADO)
# =========================================================================
Kp = 1.8 #1.6, 1,9
Kd = 0.7 # 0.7, 0.5, 0.9
VEL_BASE = 25 #20, 25
VEL_MAX  = 50 #45
VEL_MIN  = -VEL_MAX
VEL_GAP  = 25
LIMIAR   = 80
DEADZONE =5 # 8, 0
AREA_MIN = 11000

IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13
BUTTON = 24

green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])
black_min     = np.array([0, 0, 0])
black_max_top = np.array([90, 90, 90])
black_max_bot = np.array([135, 135, 135])
MIN_GREEN_AREA = 2500
kernal = np.ones((3, 3), np.uint8)

DEBUG_VERDE = True
DEBUG_SEM_ACAO = False
SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0  # segundos sem dado novo pra forcar reconexao da porta
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"

camera_x = 160
camera_y = 120

SHOW_PREVIEW    = True
PREVIEW_EVERY_N = 3
PREVIEW_VERDE_EVERY_N = 3

CAMERA_LINHA_PATH  = None  
CAMERA_VITIMA_PATH = None  
CAMERA_LINHA_INDEX_FALLBACK  = 0
CAMERA_VITIMA_INDEX_FALLBACK = 2

VICTIM_MODEL_PATH   = "modelon.onnx"
VICTIM_CROP_W, VICTIM_CROP_H = 320, 240
VICTIM_CONF_THRES   = 0.35
VICTIM_FRAMES_WARMUP = 3
VICTIM_FRAMES_ANALISE = 3000000000
DEBUG_VITIMA_PREVIEW =True

# --- RESET DE USB VIA SOFTWARE ---
USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3  # falhas seguidas antes de tentar resetar USB


# =========================================================================
# ESTADO (tudo que precisa ser resetado quando o robo reinicia pelo botao)
# =========================================================================
def resetar_estado():
    global tempo_sem_linha, girando, gangorra, em_rampa
    global contador_direita, contador_esquerda, contador_180, contador_gangorra
    global tempo_gap, ultimo_erro, ultimo_tempo
    global _trav_frame_ref, _trav_tempo_ref
    global _preview_contador, _preview_verde_contador

    tempo_sem_linha = 0
    girando = False
    gangorra = 0
    em_rampa = False
    contador_direita = 0
    contador_esquerda = 0
    contador_180 = 0
    contador_gangorra = 0
    tempo_gap = 0
    ultimo_erro = 0
    ultimo_tempo = time.time()
    _trav_frame_ref = None
    _trav_tempo_ref = time.time()
    _preview_contador = 0
    _preview_verde_contador = 0
    resetar_giroscopio()

tempo_sem_linha = 0
girando = False
gangorra = 0
em_rampa = False
contador_direita = 0
contador_esquerda = 0
contador_180 = 0
contador_gangorra = 0
tempo_gap = 0
ultimo_erro = 0
ultimo_tempo = time.time()
_trav_frame_ref = None
_trav_tempo_ref = time.time()
_preview_contador = 0
_preview_verde_contador = 0


# =========================================================================
# GPIO INIT
# =========================================================================
h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)
pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for p in pins:
    GPIO.gpio_claim_output(h, p)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# =========================================================================
# MOTORES (NAO ALTERADO)
# =========================================================================
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

def parar():
    mover(0, 0)


# =========================================================================
# BOTAO - thread dedicada, toggle com debounce
# =========================================================================
def botao_thread():
    anterior = 0
    while True:
        estado = GPIO.gpio_read(h, BUTTON)
        if estado == 1 and anterior == 0:
            time.sleep(0.03)
            if GPIO.gpio_read(h, BUTTON) == 1:
                if robo_ativo.is_set():
                    robo_ativo.clear()
                    parar()
                    print("[BOTAO] PAROU (aperte de novo pra reiniciar)")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                while GPIO.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
        anterior = estado
        time.sleep(0.02)


# =========================================================================
# MPU6050 DIRETO NA RASP 
# =======================================================================
_gyro_lock = threading.Lock()
MPU_ADDR    = 0x68
GYRO_XOUT_H = 0x43
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

def _mpu_write(reg, val):
    bus.write_byte_data(MPU_ADDR, reg, val)

def _mpu_read_word(reg):
    hi = bus.read_byte_data(MPU_ADDR, reg)
    lo = bus.read_byte_data(MPU_ADDR, reg + 1)
    val = (hi << 8) | lo
    if val >= 0x8000:
        val -= 65536
    return val

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
        except Exception:
            pass
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


# =========================================================================
# SERIAL - so sonar (F e E)
# =========================================================================
dados = {"F": 9999.0, "E": 9999.0}
ultimo_recebimento = time.time()
ser = None



# ==========================================================
# �NGULO Y EST�VEL (RAMPA/GANGORRA)
# Usa somente o aceler�metro + filtro exponencial
# ==========================================================

ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

_angulo_y_filtrado = None

def angulo_y_estavel(alpha=0.97):
    """
    Retorna o �ngulo Y em graus.
    alpha:
        0.95 = responde mais r�pido
        0.97 = recomendado
        0.99 = muito est�vel
    """

    global _angulo_y_filtrado

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
            alpha * _angulo_y_filtrado +
            (1 - alpha) * angulo
        )

    return int(_angulo_y_filtrado)



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
    global ser
    global ultimo_recebimento
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


# =========================================================================
# RESET DE USB VIA SOFTWARE (sem tirar/colocar cabo)
# =========================================================================
USBDEVFS_RESET = 21780  # 0x5514 -> _IO('U', 20), constante do kernel Linux

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

def resetar_usb_camera(video_path_ou_indice):
    
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


# =========================================================================
# CAMERA
# =========================================================================
def _abrir_camera(fonte):
    cap_local = cv2.VideoCapture(fonte, cv2.CAP_V4L2)
    if cap_local.isOpened():
        ret, _ = cap_local.read()
        if ret:
            return cap_local
        cap_local.release()
    return None

def detectar_camera():
    if CAMERA_LINHA_PATH:
        c = _abrir_camera(CAMERA_LINHA_PATH)
        if c is not None:
            print(f"[Camera] Linha conectada em {CAMERA_LINHA_PATH}")
            return c

    c = _abrir_camera(CAMERA_LINHA_INDEX_FALLBACK)
    if c is not None:
        print(f"[Camera] Linha conectada no indice {CAMERA_LINHA_INDEX_FALLBACK}")
        return c

    for i in range(10):
        c = _abrir_camera(i)
        if c is not None:
            print(f"[Camera] Linha conectada em /dev/video{i}")
            return c
    return None

def iniciar_camera():
    tentativas = 0
    cap_local = detectar_camera()
    while cap_local is None:
        tentativas += 1
        print(f"[Camera] Tentando novamente em 1s... (tentativa {tentativas})")
        time.sleep(1)
        if tentativas % USB_RESET_TENTATIVAS_ANTES_DE_RESETAR == 0:
            print("[Camera] Varias falhas seguidas, tentando resetar USB...")
            resetar_usb_camera(CAMERA_LINHA_PATH or CAMERA_LINHA_INDEX_FALLBACK)
        cap_local = detectar_camera()
    cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap_local.set(3, camera_x)
    cap_local.set(4, camera_y)
    return cap_local


# =========================================================================
# CAMERA DE VITIMAS (YOLO ONNX) - so aberta durante o resgate
# =========================================================================
modelo_vitima = None

def carregar_modelo_vitima():
    global modelo_vitima
    if not YOLO_DISPONIVEL:
        return
    try:
        print("[VITIMA] Carregando modelo YOLO...")
        modelo_vitima = YOLO(VICTIM_MODEL_PATH)
        print("[VITIMA] Modelo pronto")
    except Exception as e:
        print("[VITIMA][ERRO] Falha ao carregar modelo:", e)
        modelo_vitima = None

def _abrir_camera_vitima():
    fonte_principal = CAMERA_VITIMA_PATH or CAMERA_VITIMA_INDEX_FALLBACK
    c = _abrir_camera(fonte_principal)
    if c is not None:
        return c

    print("[VITIMA][AVISO] Camera de vitima nao respondeu, tentando resetar USB...")
    resetar_usb_camera(fonte_principal)

    c = _abrir_camera(fonte_principal)
    if c is not None:
        return c

    if CAMERA_VITIMA_PATH:
        return _abrir_camera(CAMERA_VITIMA_INDEX_FALLBACK)
    return None

def identificar_vitimas():
    """Abre a 2a camera, roda a inferencia YOLO em alguns frames e fecha.
    Retorna lista de deteccoes: [{"classe": str, "conf": float}, ...]
    Chamada dentro de area() assim que o robo entra na area de resgate."""
    if not YOLO_DISPONIVEL or modelo_vitima is None:
        print("[VITIMA] Modelo indisponivel, pulando deteccao")
        return []

    cap_vitima = _abrir_camera_vitima()
    if cap_vitima is None:
        print("[VITIMA][ERRO] Nao foi possivel abrir a camera de vitima")
        return []

    deteccoes = {}
    try:
        for _ in range(VICTIM_FRAMES_WARMUP):
            cap_vitima.read()

        for _ in range(VICTIM_FRAMES_ANALISE):
            checar_parada()
            ret, frame = cap_vitima.read()
            if not ret or frame is None:
                continue

            fh, fw = frame.shape[:2]
            start_x = max(fw // 2 - VICTIM_CROP_W // 2, 0)
            start_y = max(fh // 2 - VICTIM_CROP_H // 2, 0)
            cropped = frame[start_y:start_y + VICTIM_CROP_H, start_x:start_x + VICTIM_CROP_W]

            inicio = time.perf_counter()

            results = modelo_vitima.predict(
                cropped,
                imgsz=(VICTIM_CROP_W, VICTIM_CROP_H),
                conf=VICTIM_CONF_THRES,
                verbose=False
            )

            tempo = time.perf_counter() - inicio
            fps_modelo = 1.0 / tempo

            results = modelo_vitima.predict(
                cropped, imgsz=(VICTIM_CROP_W, VICTIM_CROP_H),
                conf=VICTIM_CONF_THRES, verbose=False
            )

            for box in results[0].boxes:
                classe = results[0].names[int(box.cls[0])]
                conf = float(box.conf[0])
                if classe not in deteccoes or conf > deteccoes[classe]:
                    deteccoes[classe] = conf

            if DEBUG_VITIMA_PREVIEW:
                annotated = results[0].plot()

                cv2.putText(
                annotated,
                f"YOLO FPS: {fps_modelo:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
                )
                cv2.imshow("Deteccao Vitima", annotated)
                cv2.waitKey(1)

    finally:
        cap_vitima.release()
        if DEBUG_VITIMA_PREVIEW:
            cv2.destroyWindow("Deteccao Vitima")

    lista = [{"classe": c, "conf": v} for c, v in deteccoes.items()]
    if lista:
        print(f"[VITIMA] Detectado: {lista}")
    else:
        print("[VITIMA] Nenhuma vitima detectada")
    return lista


# =========================================================================
# VISAO
# =========================================================================
def visao(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None
    if contours:
        main = max(contours, key=cv2.contourArea)
        M = cv2.moments(main)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])

    return cx, contours, roi, mask


def gap(contours):
    global tempo_gap
    if len(contours) == 0:
        tempo_gap += 1
        return True
    maior = max(contours, key=cv2.contourArea)
    if cv2.contourArea(maior) < AREA_MIN:
        tempo_gap += 1
        return True
    tempo_gap = 0
    return False


def angulo_linha(contours):
    if not contours:
        return None
    validos = [c for c in contours if cv2.contourArea(c) >= AREA_MIN]
    if not validos:
        return None

    melhor = None
    menor_angulo = 9999
    for c in validos:
        [vx, vy, x0, y0] = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
        angulo = math.degrees(math.atan2(float(vy), float(vx)))
        if angulo > 90:
            angulo -= 180
        elif angulo < -90:
            angulo += 180
        angulo = int(angulo - 90)
        if abs(angulo) < abs(menor_angulo):
            menor_angulo = angulo
            melhor = c
    return menor_angulo


def alinhar_linha(frame):
    global tempo_sem_linha
    angulo = angulo_linha(frame)

    if angulo is None:
        tempo_sem_linha += 1
        if tempo_sem_linha < 20:
            mover(-30, -30)
        else:
            parar()
        return False

    tempo_sem_linha = 0
    erro_angulo = 90 - angulo

    if abs(erro_angulo) < 2:
        parar()
        return True

    ajuste_linha = erro_angulo * 0.5
    ajuste_linha = np.sign(ajuste_linha) * (abs(ajuste_linha) + 25)
    ajuste_linha = np.clip(ajuste_linha, -50, 50)
    mover(ajuste_linha, -ajuste_linha)
    return False


# =========================================================================
# DETECCAO DE MARCADOR VERDE
# =========================================================================
def check_black(black_around_sign, i, green_box, black_image):
    gb_y = green_box[green_box[:, 1].argsort()]
    marker_height = int(gb_y[-1][1] - gb_y[0][1])
    sign_y = int(gb_y[2][1])
    black_around_sign[i, 4] = sign_y

    y1, y2 = sign_y, min(sign_y + int(marker_height * 0.8), camera_y)
    x1, x2 = int(min(gb_y[2][0], gb_y[3][0])), int(max(gb_y[2][0], gb_y[3][0]))
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 0] = 1

    y1, y2 = max(int(gb_y[1][1]) - int(marker_height * 0.8), 0), int(gb_y[1][1])
    x1, x2 = max(int(min(gb_y[0][0], gb_y[1][0])), 0), max(int(max(gb_y[0][0], gb_y[1][0])), 0)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 1] = 1

    gb_x = green_box[green_box[:, 0].argsort()]

    y1, y2 = int(min(gb_x[0][1], gb_x[1][1])), int(max(gb_x[0][1], gb_x[1][1]))
    x1, x2 = max(int(gb_x[1][0]) - int(marker_height * 0.8), 0), int(gb_x[1][0])
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 2] = 1

    y1, y2 = int(min(gb_x[2][1], gb_x[3][1])), int(max(gb_x[2][1], gb_x[3][1]))
    x1, x2 = int(gb_x[2][0]), min(int(gb_x[2][0]) + int(marker_height * 0.8), camera_x)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 3] = 1

    return black_around_sign


def determine_turn_direction(black_around_sign, green_centers):
    votes_left = 0
    votes_right = 0
    mid_x = (max(green_centers) + min(green_centers)) / 2 if len(green_centers) >= 2 else camera_x / 2

    for i, row in enumerate(black_around_sign):
        if row[1] == 1 and row[2] == 1 and row[0] == 0 and row[3] == 0:
            votes_right += 1
        elif row[1] == 1 and row[3] == 1 and row[0] == 0 and row[2] == 0:
            votes_left += 1

    if votes_left >= 1 and votes_right >= 1:
        return False, False, True
    return votes_left > 0, votes_right > 0, False


def check_green(contours_grn, black_image, green_image):
    global contador_esquerda, contador_direita, contador_180
    black_around_sign = np.zeros((len(contours_grn), 5), dtype=np.int16)
    green_centers = []

    for i, contour in enumerate(contours_grn):
        if cv2.contourArea(contour) <= MIN_GREEN_AREA:
            green_centers.append(camera_x / 2)
            continue
        green_box = cv2.boxPoints(cv2.minAreaRect(contour))
        green_centers.append(float(np.mean(green_box[:, 0])))
        black_around_sign = check_black(black_around_sign, i, green_box, black_image)

    turn_left, turn_right, turn_180 = determine_turn_direction(black_around_sign, green_centers)
    pixels_verde = cv2.countNonZero(green_image)
    print(pixels_verde)

    contador_180 = contador_180 + 1 if turn_180 else 0
    contador_esquerda = contador_esquerda + 1 if turn_left else 0
    contador_direita = contador_direita + 1 if turn_right else 0

    if turn_180:
        return "GIRAR_180"
    elif turn_left and pixels_verde < 15500 and contador_esquerda > 2 and contador_direita < 1:
        return "ESQUERDA"
    elif turn_right and pixels_verde < 15500 and contador_direita > 2 and contador_esquerda < 1:
        return "DIREITA"
    else:
        return "RETO"


def detectar_verde(roi):
    roi_h, roi_w = roi.shape[:2]
    corte_top = int(roi_h * 0.4)

    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, black_max_top)
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image, kernal, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    contours_grn, _ = cv2.findContours(green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_grn) == 0:
        return None

    acao = check_green(contours_grn, black_image, green_image)
    return acao if acao != "RETO" else None


def mostrar_preview_verde(roi):
    global _preview_verde_contador
    if not DEBUG_VERDE:
        return
    _preview_verde_contador += 1
    if _preview_verde_contador % PREVIEW_VERDE_EVERY_N != 0:
        return

    roi_h, roi_w = roi.shape[:2]
    corte_top = int(roi_h * 0.4)

    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, black_max_top)
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image, kernal, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    contours_grn, _ = cv2.findContours(green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    debug_img = roi.copy()
    for c in contours_grn:
        area_c = cv2.contourArea(c)
        cor = (0, 255, 0) if area_c > MIN_GREEN_AREA else (0, 165, 255)
        box = cv2.boxPoints(cv2.minAreaRect(c)).astype(int)
        cv2.drawContours(debug_img, [box], 0, cor, 1)
        cv2.putText(debug_img, f"{int(area_c)}", tuple(box[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, cor, 1)

    escala = 3
    debug_big = cv2.resize(debug_img, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
    black_big = cv2.resize(black_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
    green_big = cv2.resize(green_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Verde - ROI (contornos)", debug_big)
    cv2.imshow("Verde - mask preto", black_big)
    cv2.imshow("Verde - mask verde", green_big)
    cv2.waitKey(1)


def guinada2(LADO, GRAUS, VELOCIDADE):
    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS - 13)
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


def guinada_pid(LADO, GRAUS, VEL_MAX_GIRO=100, VEL_MIN_GIRO=29, ZONA_CHEGADA=10):
    parar()
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
            parar()
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


def meiavolta(LADO='E', GRAUS=90, VEL_MAX_GIRO=100, VEL_MIN_GIRO=25, ZONA_CHEGADA=5):
    Kp_g, Ki_g, Kd_g = 0.09, 0.002, 0.01
    resetar_giroscopio()
    time.sleep(0.1)

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
            time.sleep(2.0)
            parar()
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


def limpar_camera(n=5):
    for _ in range(n):
        cap.read()


def girar_ate_linha(direcao, velocidade=60, tolerancia=40, timeout=5.0):
    inicio = time.time()
    while True:
        checar_parada()
        ret, frame = cap.read()
        if not ret:
            continue
        cx, contours, roi, mask = visao(frame)
        w = roi.shape[1]
        centro = w // 2

        if cx is not None and abs(cx - centro) < tolerancia:
            parar()
            break
        if time.time() - inicio > timeout:
            parar()
            break

        if direcao == "ESQUERDA":
            mover(-velocidade, velocidade)
        elif direcao == "DIREITA":
            mover(velocidade, -velocidade)
        else:
            mover(velocidade, velocidade)
        time.sleep(0.005)


def area(contours):
    print("estou na area")
    identificar_vitimas()  # <- chama a 2a camera + inferencia assim que entra no resgate
    mover(-19, -19)
    time.sleep(0.09)
    maior = max(contours, key=cv2.contourArea)

    while True:
        checar_parada()
        mover(25, 25)
        if distancia_frente() <= 6 or (len(contours) == 1 and cv2.contourArea(maior) > AREA_MIN):
            mover(-10, -10)
            time.sleep(0.09)
            if distancia_frente() <= 6:
                guinada2("E", 45, 40)
            if len(contours) == 1 and cv2.contourArea(maior) > AREA_MIN:
                mover(-10, -10)
                time.sleep(0.09)
            break
        time.sleep(0.01)


def desvio():
    if distancia_frente() <= 6:
        time.sleep(0.09)
        if distancia_frente() <= 5:
            mover(30, 30)
            mover(-30, -30)
            time.sleep(0.2)
            mover(-10, -10)
            time.sleep(0.1)

            guinada_pid("D", 90)

            while True:
                checar_parada()
                mover(30, 30)
                if distancia_esquerda() < 15:
                    mover(-10, -10)
                    time.sleep(0.2)
                    break
                time.sleep(0.005)

            while True:
                checar_parada()
                mover(35, 35)
                if distancia_esquerda() > 20:
                    mover(-10, -10)
                    time.sleep(0.2)
                    break
                time.sleep(0.005)

            guinada_pid("E", 90)
            while True:
                checar_parada()
                mover(30, 30)
                if distancia_esquerda() < 20:
                    mover(30, 30)
                    time.sleep(0.2)
                    mover(-10, -10)
                    time.sleep(0.3)
                    break
                time.sleep(0.005)

            while True:
                checar_parada()
                mover(30, 30)
                if distancia_esquerda() > 17:
                    mover(40, 40)
                    time.sleep(0.4)
                    mover(-10, -10)
                    time.sleep(0.3)
                    break
                time.sleep(0.005)

            guinada_pid("E", 84)
            while True:
                checar_parada()
                mover(30, 30)
                if distancia_esquerda() < 20:
                    mover(-40, -40)
                    time.sleep(0.3)
                    mover(10, 10)
                    time.sleep(0.3)
                    break
                time.sleep(0.005)

            guinada_pid("D", 90)
            mover(-30, -30)
            time.sleep(0.4)


def verde(roi):
   
    acao_verde = detectar_verde(roi)
    if not acao_verde:
        return

    if DEBUG_SEM_ACAO:
        print(f"[DEBUG SEM ACAO] Verde detectado: {acao_verde} "
              f"| esq={contador_esquerda} dir={contador_direita} 180={contador_180}")
        return

    print(f"[VERDE] Primeira deteccao: {acao_verde}")
    inicio_confirmacao = time.time()
    acao_confirmada = acao_verde

    while time.time() - inicio_confirmacao < 0.01:
        checar_parada()
        ret, frame_conf = cap.read()
        if not ret:
            continue
        cx_c, contours_c, roi_c, _ = visao(frame_conf)
        controle(cx_c, contours_c, roi_c)
        nova_acao = detectar_verde(roi_c)
        if nova_acao:
            acao_confirmada = nova_acao

    acao_verde = acao_confirmada
    parar()
  

    if acao_verde == "ESQUERDA":
        mover(-10, -10)
        time.sleep(0.1)
        mover(30, 30)
        time.sleep(0.18)
        parar()
        guinada2('E', 85, 40)
        inicio = time.time()
        while time.time() - inicio < 1.0:
            checar_parada()
            ret, frame = cap.read()
            if not ret:
                continue
            cx, contours, roi_loop, mask = visao(frame)
            controle(cx, contours, roi_loop)

    elif acao_verde == "DIREITA":
        limpar_camera(10)
        mover(-10, -10)
        time.sleep(0.1)
        mover(30, 30)
        time.sleep(0.18)
        parar()
        guinada2('D', 85, 40)
        inicio = time.time()
        while time.time() - inicio < 1.0:
            checar_parada()
            ret, frame = cap.read()
            if not ret:
                continue
            cx, contours, roi_loop, mask = visao(frame)
            controle(cx, contours, roi_loop)

    elif acao_verde == "GIRAR_180":
        print("gira 180")
        mover(-12, -12)
        time.sleep(0.09)
        guinada_pid("D", 170)
        mover(30, 30)
        time.sleep(0.19)
        mover(-10, -10)
        time.sleep(0.2)
        parar()
        limpar_camera(10)


def detectar_fita_vermelha(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
    )
    h_, w_ = mask.shape
    for y in range(0, h_, 5):
        if cv2.countNonZero(mask[y, :]) > w_ * 0.6:
            print("[VERMELHO] Fita detectada - pausando 6s")
            parar()
            time.sleep(6)
            limpar_camera(10)
            return True
    return False


def andar_reto(velocidade=30, segundos=1.0):
    resetar_giroscopio()
    Kp_, Ki_, Kd_ = -2, -0.2, -0.4
    erro_anterior = 0
    integral = 0
    ultimo_tempo_ = time.time()
    inicio = ultimo_tempo_

    while time.time() - inicio < segundos:
        checar_parada()
        agora = time.time()
        dt = agora - ultimo_tempo_
        if dt <= 0:
            dt = 0.001
        erro = angulo_z()
        integral += erro * dt
        integral = np.clip(integral, -30, 30)
        derivada = (erro - erro_anterior) / dt
        ajuste = Kp_ * erro + Ki_ * integral + Kd_ * derivada
        velL = velocidade - ajuste
        velR = velocidade + ajuste
        mover(velL, velR)
        erro_anterior = erro
        ultimo_tempo_ = agora
        time.sleep(0.01)
    parar()


def gap_debug(contours):
    if len(contours) == 0:
        return True
    maior = max(contours, key=cv2.contourArea)
    area_ = cv2.contourArea(maior)
    if area_ < AREA_MIN:
        return True
    return False


def detectar_travamento(roi):
    global _trav_frame_ref, _trav_tempo_ref
    LIMIAR_DIFF = 8
    TEMPO_TRAV = 4.0
    INTERVALO_REF = 2.0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    agora = time.time()

    if _trav_frame_ref is None:
        _trav_frame_ref = gray.copy()
        _trav_tempo_ref = agora
        return False

    movimento = np.mean(cv2.absdiff(gray, _trav_frame_ref))

    if agora - _trav_tempo_ref >= INTERVALO_REF:
        if movimento >= LIMIAR_DIFF:
            _trav_frame_ref = gray.copy()
            _trav_tempo_ref = agora
            return False
        else:
            tempo_parado = agora - _trav_tempo_ref
            print(f"[TRAV] diff={movimento:.1f} | parado ha {tempo_parado:.1f}s")
            if tempo_parado >= TEMPO_TRAV:
                print("[TRAVAMENTO] Robo travado!")
                _trav_frame_ref = None
                _trav_tempo_ref = agora
                return True
    return False


# CONTROLE (segue linha) - NAO ALTERADO na logica
# =========================================================================
def controle(cx, contours, roi):
    global ultimo_erro, ultimo_tempo, em_rampa, contador_gangorra

    w = roi.shape[1]
    tempo_atual = time.time()
    dt = tempo_atual - ultimo_tempo
    if dt <= 0:
        dt = 0.0001

    if cx is not None:
        centro = w // 2
        erro = (cx - centro) / centro * 100
    else:
        erro = ultimo_erro

    if abs(erro) < DEADZONE:
        erro = 0
    if gap_debug(contours):
        erro = 0

    derivada = (erro - ultimo_erro) / dt
    derivada = np.clip(derivada, -300, 300)
    ajuste = (Kp * erro) + (Kd * derivada)

    if -5 < angulo_y() < 5:
        contador_gangorra = 0

    if angulo_y() <= -10:
        velocidade_base = 60
        velL = np.clip(velocidade_base + ajuste, 20, 100)
        velR = np.clip(velocidade_base - ajuste, 20, 100)
    elif angulo_y() >= 10 and contador_gangorra == 0:
        contador_gangorra = 1
        mover(-10, -10)
        time.sleep(1.0)
        mover(-40, -40)
        time.sleep(0.3)
        velocidade_base = 20
        velL = velocidade_base + ajuste
        velR = velocidade_base - ajuste
    else:
        velocidade_base = VEL_BASE
        velL = np.clip(velocidade_base + ajuste, VEL_MIN, VEL_MAX)
        velR = np.clip(velocidade_base - ajuste, VEL_MIN, VEL_MAX)

    mover(velL, velR)
    ultimo_erro = erro
    ultimo_tempo = tempo_atual
    return ajuste, contours


# =========================================================================
# LOOP PRINCIPAL
# =========================================================================
FALHAS_CAMERA_ANTES_DE_RESETAR = 15  # ~ alguns segundos de frames perdidos seguidos

def loop_principal():
    global ultimo_erro, ultimo_tempo, cap
    resetar_estado()
    parar()
    Thread(target=serial_thread, daemon=True).start() if not _serial_thread_iniciada[0] else None
    _serial_thread_iniciada[0] = True

    falhas_camera_seguidas = 0

    while True:
        checar_parada()

        ret, frame = cap.read()
        if not ret:
            falhas_camera_seguidas += 1
            if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_RESETAR:
                print("[Camera] Linha de camera parou de responder, resetando USB...")
                try:
                    cap.release()
                except Exception:
                    pass
                resetar_usb_camera(CAMERA_LINHA_PATH or CAMERA_LINHA_INDEX_FALLBACK)
                cap = iniciar_camera()
                falhas_camera_seguidas = 0
            continue
        falhas_camera_seguidas = 0

        if detectar_fita_vermelha(frame):
            continue

        cx, contours, roi, mask = visao(frame)

        if detectar_travamento(roi):
            print("[TRAVAMENTO] Tentando se soltar...")
            while True:
                checar_parada()
                mover(100, 100)
                time.sleep(0.3)
                ret, frame_trav = cap.read()
                if ret:
                    _, _, roi_trav, _ = visao(frame_trav)
                    if not detectar_travamento(roi_trav):
                        print("[TRAVAMENTO] Solto!")
                        parar()
                        ultimo_erro = 0
                        ultimo_tempo = time.time()
                break

        verde(roi)
        ajuste, contours = controle(cx, contours, roi)
        desvio()

        if gap_debug(contours) and distancia_esquerda() < 20:
            inicio_gap = time.time()
            while True:
                checar_parada()
                ret, frame = cap.read()
                if not ret:
                    continue
                cx, contours, roi, mask = visao(frame)
                if not gap_debug(contours):
                    print("[GAP] Saiu do gap, voltando a linha")
                    break
                if time.time() - inicio_gap > 1:
                    parar()
                    print("resgate imediatamente")
                    time.sleep(3.0)
                    area(contours)
                    break
                mover(VEL_GAP, VEL_GAP)

        if cv2.waitKey(1) == 27:
            break


_serial_thread_iniciada = [False]


def iniciar_hardware():
    
    global cap

    t_mpu = Thread(target=iniciar_mpu)
    t_mpu.start()
    t_yolo = Thread(target=carregar_modelo_vitima)
    t_yolo.start()

    handshake_serial()      # serial primeiro, igual no codigo original
    cap = iniciar_camera()  # so a camera de LINHA e aberta no boot

    t_mpu.join()
    t_yolo.join()
    print("[BOOT] Hardware pronto")


if __name__ == "__main__":
    t0 = time.time()
    iniciar_hardware()
    print(f"[BOOT] Tempo de inicializacao: {time.time() - t0:.1f}s")

    Thread(target=botao_thread, daemon=True).start()

    try:
        while True:
            try:
                robo_ativo.wait()
                loop_principal()
                #print(angulo_y())

            except RoboParado:
                parar()
                print("[SISTEMA] Robo parado. Aguardando novo aperto do botao...")
                robo_ativo.wait()
                continue

            except Exception as e:
                print("[ERRO] loop principal falh:", repr(e))
                parar()
                time.sleep(0.5)
                continue

    except KeyboardInterrupt:
        print("Finalizado")

    finally:
        parar()
        GPIO.gpiochip_close(h)
        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()
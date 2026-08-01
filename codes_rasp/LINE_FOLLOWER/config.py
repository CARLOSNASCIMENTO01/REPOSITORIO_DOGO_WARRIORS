"""
Hardware do seguidor de linha: estado de execucao/parada, GPIO, motores
(placa Yahboom), servos, botao, giroscopio (MPU6050), sonar (serial do
Arduino) e camera de linha. Tudo que os outros modulos (movimentos_bases,
seguimento_de_linha, obstaculos_trajeto, resgate_de_vitimas) importam pra
poder mexer no robo de verdade.

`cap` e dono deste modulo: iniciar_camera() atribui com `global cap`. Quem
precisar ler frame em outro modulo usa `import config` e `config.cap.read()`
- nunca `from config import cap` (isso capturaria o `None` inicial e nunca
veria a camera real depois de aberta).
"""

import fcntl
import math
import os
import subprocess
import sys
import threading
import time
from threading import Thread

import cv2
import lgpio
import serial
import serial.tools.list_ports
import smbus2

# =========================================================================
# ESTADO DE EXECUCAO / PARADA IMEDIATA
# =========================================================================
class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar QUALQUER loop na hora."""
    pass


robo_ativo = threading.Event()
ENTREGAS_HERDADAS = 0
if "--auto" in sys.argv:
    robo_ativo.set()
else:
    robo_ativo.clear()

for arg in sys.argv[1:]:
    if arg.startswith("--entregas="):
        try:
            ENTREGAS_HERDADAS = int(arg.split("=", 1)[1])
        except ValueError:
            ENTREGAS_HERDADAS = 0


def checar_parada():
    """Chamar a cada iteracao de todo while True que segura o robo por mais
    de um instante (guinadas, desvio, resgate, etc). Custo desprezivel."""
    if not robo_ativo.is_set():
        raise RoboParado()


# =========================================================================
# GPIO
# =========================================================================
PINO_SG90 = 5
PINO_PESADO = 20
BUTTON = 24
TCRT = 0  # GPIO BCM 5

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)
lgpio.gpio_claim_input(h, BUTTON, lgpio.SET_PULL_UP)
lgpio.gpio_claim_input(h, TCRT)
# Pinos de motor (IN1..ENB) nao sao claimed aqui - motores vao pela serial
# da placa Yahboom, nao por GPIO direto.


# =========================================================================
# MOTORES: placa Yahboom via serial (PWM) + SERVOS + BOTAO
# =========================================================================
# Por enquanto so PWM (malha aberta) - sem depender do encoder, que ainda
# tem problema no lado direito (M1 sem sinal, M3 fraco). Mapeamento real
# desse robo (nao e o padrao de fabrica da Yahboom):
#   M1 = dianteira direita (sem encoder) | M2 = traseira esquerda
#   M3 = traseira direita               | M4 = dianteira esquerda
# Todo canal usa sinal invertido nesse robo (motor montado espelhado no
# chassi) - por isso o -1 em cada valor antes de mandar.
YAHBOOM_PORTA = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70  # converte a faixa antiga do L298N (-50..50) pra PWM Yahboom (-3600..3600)
PWM_FREIO_YAHBOOM = 700   # forca do pulso de freio reverso ao parar
TEMPO_FREIO_YAHBOOM = 0.08  # segundos do pulso de freio

VEL_MAX = 20 #45
VEL_MIN = -VEL_MAX

ANGULO_INICIAL = 90

# Servo pesado como indicador visual de ligado/desligado: levantado (150)
# enquanto o robo esta ativo, abaixado (0) enquanto parado/esperando o botao.
SERVO_PESADO_ANGULO_ATIVO = 140
SERVO_PESADO_ANGULO_PARADO = 10

motor_serial = serial.Serial(YAHBOOM_PORTA, YAHBOOM_BAUDRATE, timeout=0.5)


def _enviar_pwm_yahboom(m1, m2, m3, m4):
    m1, m2, m3, m4 = (max(-3600, min(3600, int(v))) for v in (m1, m2, m3, m4))
    try:
        motor_serial.write(f"$pwm:{m1},{m2},{m3},{m4}#".encode())
    except serial.SerialException as e:
        print(f"[MOTOR][ERRO] Falha ao mandar PWM pra placa Yahboom: {e}")


_ultima_vel = (0, 0)  # (velL, velR) do ultimo mover(), pra parar() saber pra que lado frear


def aplicar_motor(velL, velR):
    """Converte velocidade esquerda/direita (faixa antiga do L298N,
    -VEL_MAX..VEL_MAX) pra PWM da placa Yahboom nos 4 canais. Lado
    esquerdo = M2+M4, lado direito = M1+M3 (mapeamento real desse robo -
    ver comentario acima). Sinal invertido em todos os canais porque o
    motor fica montado espelhado no chassi."""
    velL = max(min(velL, VEL_MAX), VEL_MIN)
    velR = max(min(velR, VEL_MAX), VEL_MIN)
    pwm_esq = velL * ESCALA_PWM_YAHBOOM
    pwm_dir = velR * ESCALA_PWM_YAHBOOM
    # (M1, M2, M3, M4) = (direita, esquerda, direita, esquerda).
    # OBS: o $pwm (malha aberta) saiu com o sinal ao contrario do $spd que
    # tinhamos validado (robo foi de re) - aqui SEM o "-" extra, sinal
    # direto, ate confirmar de novo qual convencao bate.
    _enviar_pwm_yahboom(pwm_dir, pwm_esq, pwm_dir, pwm_esq)


def mover(velL, velR):
    global _ultima_vel
    aplicar_motor(velL, velR)
    _ultima_vel = (velL, velR)


def parar():
    global _ultima_vel
    velL, velR = _ultima_vel
    if velL != 0 or velR != 0:
        # pulso curto reverso antes de zerar, reduz a inercia em vez de so
        # cortar e deixar o robo deslizando - forca do freio convertida de
        # PWM pra "unidade vel" (mesma escala do aplicar_motor) pra reusar
        # a mesma conversao/inversao de sinal, sem duplicar logica
        freio_vel = PWM_FREIO_YAHBOOM / ESCALA_PWM_YAHBOOM
        freio_esq = -freio_vel if velL > 0 else (freio_vel if velL < 0 else 0)
        freio_dir = -freio_vel if velR > 0 else (freio_vel if velR < 0 else 0)
        aplicar_motor(freio_esq, freio_dir)
        time.sleep(TEMPO_FREIO_YAHBOOM)
    mover(0, 0)


def angulo(ang):
    ang = max(0, min(180, ang))
    pulso = int(500 + (2000 * ang / 180))
    lgpio.tx_servo(h, PINO_SG90, pulso)
    time.sleep(0.1)
    lgpio.tx_servo(h, PINO_SG90, 0)
    time.sleep(0.09)


def mover_servo(pino, angulo_alvo, velocidade=100, pulso_min=500, pulso_max=2500, delay_base=0.02):
    if not hasattr(mover_servo, "_angulo_atual"):
        mover_servo._angulo_atual = {}

    angulo_alvo = max(0, min(180, angulo_alvo))
    velocidade = max(1, min(100, velocidade))

    atual = mover_servo._angulo_atual.get(pino, ANGULO_INICIAL)
    delay_por_passo = delay_base * (100 - velocidade) / 100.0

    passo = 2 if angulo_alvo > atual else -2

    for ang in range(int(atual), int(angulo_alvo), passo):
        checar_parada()
        largura_pulso = pulso_min + (ang / 180.0) * (pulso_max - pulso_min)
        try:
            lgpio.tx_servo(h, pino, int(largura_pulso))
        except lgpio.error as e:
            print(f"Falha ao mover servo (pino={pino}, pulso={largura_pulso:.0f}): {e}")
            continue
        if delay_por_passo > 0:
            time.sleep(delay_por_passo)

    # range() acima nunca inclui angulo_alvo em si (e exclusivo no limite) -
    # sem esse pulso final, o ultimo comando enviado ficava 1 passo (2 graus)
    # curto do alvo, mas _angulo_atual[pino] já registrava o alvo completo.
    # Some servo ficava "preso" segurando 2 graus fora da posicao marcada
    # como atual - proximo movimento a partir dai comecava com esse erro
    # acumulado, o que da bem a cara de "fica tentando corrigir sozinho".
    largura_pulso = pulso_min + (angulo_alvo / 180.0) * (pulso_max - pulso_min)
    try:
        lgpio.tx_servo(h, pino, int(largura_pulso))
    except lgpio.error as e:
        print(f"Falha ao mover servo (pino={pino}, pulso={largura_pulso:.0f}): {e}")

    mover_servo._angulo_atual[pino] = angulo_alvo
    time.sleep(0.8)

    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error as e:
        print(f"Falha ao desligar PWM do servo (pino={pino}): {e}")


def mover_servo_direto(pino, angulo_alvo, pulso_min=500, pulso_max=2500):
    """Igual mover_servo(), mas sem o passo-a-passo e sem checar_parada() -
    pra usar de dentro do botao_thread(), onde chamar checar_parada() logo
    apos limpar robo_ativo abortaria na hora e mataria essa thread."""
    if not hasattr(mover_servo, "_angulo_atual"):
        mover_servo._angulo_atual = {}
    angulo_alvo = max(0, min(180, angulo_alvo))
    largura_pulso = pulso_min + (angulo_alvo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    mover_servo._angulo_atual[pino] = angulo_alvo


def sensor_e():
    return lgpio.gpio_read(h, TCRT)


def botao_thread():
    anterior = 0
    while True:
        estado = lgpio.gpio_read(h, BUTTON)
        if estado == 1 and anterior == 0:
            time.sleep(0.03)
            if lgpio.gpio_read(h, BUTTON) == 1:
                if robo_ativo.is_set():
                    robo_ativo.clear()
                    parar()
                    print("[BOTAO] PAROU (aperte de novo pra reiniciar)")
                    # mover_servo() normal chama checar_parada() a cada passo -
                    # abortaria na hora (acabamos de limpar robo_ativo) e
                    # mataria essa propria thread. Por isso a versao direta.
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_PARADO)
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao abaixar servo pesado: {e}")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_ATIVO)
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao levantar servo pesado: {e}")
                while lgpio.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
        anterior = estado
        time.sleep(0.02)


# =========================================================================
# MPU6050 DIRETO NA RASP
# =========================================================================
_gyro_lock = threading.Lock()
MPU_ADDR    = 0x68
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

_angulo_y_filtrado = None


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


# ==========================================================
# ANGULO Y ESTAVEL (RAMPA/GANGORRA)
# Usa somente o acelerometro + filtro exponencial
# ==========================================================
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


# =========================================================================
# SERIAL - so sonar (F e E)
# =========================================================================
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"
SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0  # segundos sem dado novo pra forcar reconexao da porta

dados = {"F": 9999.0, "E": 9999.0}
ultimo_recebimento = time.time()
ser = None


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
# CAMERA DE LINHA
# =========================================================================
CAMERA_LINHA_PATH  = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK  = 0
CAMERA_VITIMA_INDEX_FALLBACK = 2  # so pra nunca "roubar" esse indice na varredura abaixo

camera_x = 160
camera_y = 120

USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3  # falhas seguidas antes de tentar resetar USB

cap = None  # atribuido por iniciar_camera()


def abrir_camera_generica(fonte):
    """Helper generico de abertura (tenta ler um frame antes de aceitar) -
    usado tanto pra camera de linha (detectar_camera abaixo) quanto pra
    camera de vitima (resgate_de_vitimas.py)."""
    cap_local = cv2.VideoCapture(fonte, cv2.CAP_V4L2)
    if cap_local.isOpened():
        ret, _ = cap_local.read()
        if ret:
            return cap_local
        cap_local.release()
    return None


def detectar_camera():
    if CAMERA_LINHA_PATH:
        c = abrir_camera_generica(CAMERA_LINHA_PATH)
        if c is not None:
            print(f"[Camera] Linha conectada em {CAMERA_LINHA_PATH}")
            return c

    c = abrir_camera_generica(CAMERA_LINHA_INDEX_FALLBACK)
    if c is not None:
        print(f"[Camera] Linha conectada no indice {CAMERA_LINHA_INDEX_FALLBACK}")
        return c

    # Ultimo recurso: varre indices, mas NUNCA aceita o indice reservado pra
    # camera de vitima - senao a busca da camera de linha pode "roubar" a
    # camera de vitima sem perceber (ela abre e le frame normal, so que da
    # camera errada), e o robo anda reto sem enxergar a linha.
    for i in range(10):
        if i == CAMERA_VITIMA_INDEX_FALLBACK:
            continue
        c = abrir_camera_generica(i)
        if c is not None:
            print(f"[Camera] Linha conectada em /dev/video{i}")
            return c
    return None


def iniciar_camera():
    global cap
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
    cap = cap_local
    return cap_local

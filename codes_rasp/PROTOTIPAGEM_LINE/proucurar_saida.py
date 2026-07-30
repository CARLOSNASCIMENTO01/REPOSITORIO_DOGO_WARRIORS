"""
ROBO_DOGO_2026 - BASE DE HARDWARE
Extraido do codigo original do seguidor de linha, mantendo APENAS o que
e infraestrutura: GPIO/motores, botao, camera(s), IMU (MPU6050), serial
com o Arduino (sonar) e reset de USB.

Ficou de fora, de proposito: visao() de linha, deteccao de verde,
guinadas/desvio, controle PD de seguir linha, area()/resgate.py.
A ideia e voce montar em cima disso a funcao de "achar saida" da sala
de resgate.

Uso tipico:
    import robo_base as base

    base.iniciar_hardware()          # sobe MPU + serial + camera de linha em paralelo
    Thread(target=base.botao_thread, daemon=True).start()

    while True:
        base.robo_ativo.wait()
        try:
            achar_saida()            # sua funcao nova, usando as pecas abaixo
        except base.RoboParado:
            pass
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


# =========================================================================
# EXCECAO DE PARADA IMEDIATA (botao toggle)
# =========================================================================
class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar QUALQUER loop na hora."""
    pass

robo_ativo = threading.Event()
robo_ativo.set()  # comeca ativo; troque pra .clear() se quiser depender do botao


def checar_parada():
    """Chamar a cada iteracao de todo while True que segura o robo por mais
    de um instante. Custo desprezivel."""
    if not robo_ativo.is_set():
        raise RoboParado()


# =========================================================================
# CONFIG DE HARDWARE (pinos e camera)
# =========================================================================
IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13
BUTTON = 24
TCRT = 6  # sensor de linha extra (GPIO BCM 5/6, mantive como no original)

VEL_MAX = 50
VEL_MIN = -VEL_MAX

camera_x = 160
camera_y = 120

SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0  # segundos sem dado novo pra forcar reconexao da porta
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"

CAMERA_LINHA_PATH = None
CAMERA_VITIMA_PATH = None
CAMERA_LINHA_INDEX_FALLBACK = 0
CAMERA_VITIMA_INDEX_FALLBACK = 2

USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3


# =========================================================================
# GPIO INIT
# =========================================================================
h = GPIO.gpiochip_open(0)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)
GPIO.gpio_claim_input(h, TCRT)
for p in (IN1, IN2, IN3, IN4, ENA, ENB):
    GPIO.gpio_claim_output(h, p)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# =========================================================================
# MOTORES
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


def sensor_e():
    return GPIO.gpio_read(h, TCRT)


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
# MPU6050 (giroscopio + angulo estavel via acelerometro)
# =========================================================================
_gyro_lock = threading.Lock()
MPU_ADDR = 0x68
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
bus = smbus2.SMBus(1)

ang_gyro = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset = {"X": 0.0, "Y": 0.0, "Z": 0.0}
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
        except OSError:
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
def angulo_z(): return int(ang_gyro["Z"])


def angulo_y_estavel(alpha=0.97):
    """Angulo de inclinacao (rampa/gangorra) so pelo acelerometro, filtrado."""
    global _angulo_y_filtrado
    try:
        ax = _mpu_read_word(ACCEL_XOUT_H) / 16384.0
        ay = _mpu_read_word(ACCEL_YOUT_H) / 16384.0
        az = _mpu_read_word(ACCEL_ZOUT_H) / 16384.0
        angulo = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if _angulo_y_filtrado is None:
            _angulo_y_filtrado = angulo
        else:
            _angulo_y_filtrado = alpha * _angulo_y_filtrado + (1 - alpha) * angulo
    except OSError as e:
        print(f"[MPU] Falha I2C ({e}), mantendo ultimo valor.")
    return int(_angulo_y_filtrado) if _angulo_y_filtrado is not None else 0


def angulo_y():
    return angulo_y_estavel()


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
            gx = (_mpu_read_word(GYRO_XOUT_H) - gyro_offset["X"]) / 131.0
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


# =========================================================================
# SERIAL - leitura do sonar (F = frente, E = esquerda) do Arduino
# =========================================================================
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
    """Fica tentando ate o Arduino aparecer na USB, sem matar o processo."""
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
    """Handshake + sobe a thread de leitura continua (so uma vez)."""
    handshake_serial()
    if not _serial_thread_iniciada[0]:
        Thread(target=serial_thread, daemon=True).start()
        _serial_thread_iniciada[0] = True


# =========================================================================
# RESET DE USB VIA SOFTWARE (sem tirar/colocar cabo)
# =========================================================================
USBDEVFS_RESET = 21780  # 0x5514


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
# CAMERA(S)
# =========================================================================
def _abrir_camera(fonte):
    cap_local = cv2.VideoCapture(fonte, cv2.CAP_V4L2)
    if cap_local.isOpened():
        ret, _ = cap_local.read()
        if ret:
            return cap_local
        cap_local.release()
    return None


def detectar_camera(path_fixo=None, indice_fallback=0):
    if path_fixo:
        c = _abrir_camera(path_fixo)
        if c is not None:
            print(f"[Camera] Conectada em {path_fixo}")
            return c

    c = _abrir_camera(indice_fallback)
    if c is not None:
        print(f"[Camera] Conectada no indice {indice_fallback}")
        return c

    for i in range(10):
        c = _abrir_camera(i)
        if c is not None:
            print(f"[Camera] Conectada em /dev/video{i}")
            return c
    return None


def iniciar_camera(path_fixo=None, indice_fallback=CAMERA_LINHA_INDEX_FALLBACK):
    """Abre a camera, tentando de novo (com reset de USB) se falhar."""
    tentativas = 0
    cap_local = detectar_camera(path_fixo, indice_fallback)
    while cap_local is None:
        tentativas += 1
        print(f"[Camera] Tentando novamente em 1s... (tentativa {tentativas})")
        time.sleep(1)
        if tentativas % USB_RESET_TENTATIVAS_ANTES_DE_RESETAR == 0:
            print("[Camera] Varias falhas seguidas, tentando resetar USB...")
            resetar_usb_camera(path_fixo or indice_fallback)
        cap_local = detectar_camera(path_fixo, indice_fallback)
    cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap_local.set(3, camera_x)
    cap_local.set(4, camera_y)
    return cap_local


def limpar_camera(cap, n=5):
    """Descarta n frames (buffer de camera velho) antes de ler algo novo."""
    for _ in range(n):
        cap.read()


# =========================================================================
# DETECCAO DE CHAO PRETO (achar saida na sala de resgate)
# =========================================================================
BLACK_MIN = np.array([0, 0, 0], dtype=np.uint8)
BLACK_MAX = np.array([180, 255, 60], dtype=np.uint8)
_kernel_preto = np.ones((3, 3), np.uint8)


def contar_pixels_pretos(frame, mostrar_debug=False):
    """Aplica ROI nos 50% inferiores do frame e retorna (pixels_pretos,
    porcentagem) de preto na mascara HSV. Um frame so."""
    altura, largura = frame.shape[:2]
    y_inicio = altura // 2
    roi = frame[y_inicio:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, BLACK_MIN, BLACK_MAX)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel_preto)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel_preto)

    pixels_pretos = cv2.countNonZero(mask)
    pixels_totais = mask.shape[0] * mask.shape[1]
    porcentagem = (pixels_pretos / pixels_totais) * 100 if pixels_totais else 0.0

    if mostrar_debug:
        cv2.rectangle(frame, (0, y_inicio), (largura, altura), (255, 0, 0), 2)
        cv2.putText(frame, f"Pixels Pretos: {pixels_pretos}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"{porcentagem:.2f}% Preto", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Camera", frame)
        cv2.imshow("ROI", roi)
        cv2.imshow("Mascara", mask)
        cv2.waitKey(1)

    return pixels_pretos, porcentagem


def processar_pixels_pretos(cap, duracao=4.0, mostrar_debug=False):
    """Le da camera durante `duracao` segundos, somando os pixels pretos
    de cada frame processado, e retorna o total acumulado nesse periodo
    (junto com a media por frame, caso seja util pra decisao)."""
    checar_parada()
    inicio = time.time()
    total_pixels = 0
    amostras = 0

    while time.time() - inicio < duracao:
        checar_parada()
        ret, frame = cap.read()
        if not ret:
            continue
        pixels_pretos, _ = contar_pixels_pretos(frame, mostrar_debug=mostrar_debug)
        total_pixels += pixels_pretos
        amostras += 1

    media_pixels = (total_pixels / amostras) if amostras else 0.0
    return total_pixels, media_pixels


# =========================================================================
# TROCA DE PROGRAMA (ex: sair do modulo atual e cair no de resgate)
# =========================================================================
def trocar_para(script_path, *args_extra):
    """Substitui o processo atual pelo outro script, mantendo o mesmo PID
    (util pra ir do seguidor de linha pro modulo de resgate sem deixar
    processo/handles pendurados)."""
    os.execv(sys.executable, [sys.executable, script_path, *args_extra])


# =========================================================================
# BOOT (sobe MPU + serial + camera de linha em paralelo)
# =========================================================================
def iniciar_hardware():
    global cap

    t_mpu = Thread(target=iniciar_mpu)
    t_mpu.start()

    iniciar_serial()
    cap = iniciar_camera(CAMERA_LINHA_PATH, CAMERA_LINHA_INDEX_FALLBACK)

    t_mpu.join()
    print("[BOOT] Hardware pronto")
    return cap


def finalizar_hardware(cap=None):
    parar()
    GPIO.gpiochip_close(h)
    try:
        if cap is not None:
            cap.release()
    except Exception:
        pass
    cv2.destroyAllWindows()


# =========================================================================
# ACHAR SAIDA - rotina que roda quando o codigo inicia
# =========================================================================
def achar_saida():
    cap_resgate = iniciar_camera(CAMERA_VITIMA_PATH, CAMERA_VITIMA_INDEX_FALLBACK)

    while True:
        checar_parada()
        mover(-40, 40)
        time.sleep(0.09)
        mover(5, -5)
        time.sleep(0.3)
        print("distancia", distancia_frente())

        if distancia_frente() > 55:
            mover(25, 25)
            time.sleep(1.5)
            mover(5, -5)
            time.sleep(0.6)
            while True :
                mover(-30,30)
                #print("distancia da frente ", distancia_frente())
                if distancia_frente() < 50:
                    mover(5, -5)
                    time.sleep(0.1)
                    break
            while True :
                mover(30,-30)
                #print("distancia da frente ", distancia_frente())
                if distancia_frente() > 45:
                    mover(-5, 5)
                    time.sleep(0.7)
                    break
            while True :
                mover(35,-35)
                if distancia_frente() < 50:
                    mover(-35, 35)
                    time.sleep(0.2)
                    mover(5, -5)
                    time.sleep(2)
                    break
                


            """ret, frame = cap_resgate.read()
            if ret:
                pixels_pretos, _ = contar_pixels_pretos(frame)
                print("quantidade de preto",pixels_pretos)
                if pixels_pretos > 200:   # <-- aqui é onde você usa o valor no if
                    print("chao preto detectado, confirma saida")
                    time.sleep(3.0)"""

                
                    

            print("achei potencial saida ")
            mover(0, 0)
            time.sleep(2.0)
            break

    cap_resgate.release()

if __name__ == "__main__":
    iniciar_serial()
    Thread(target=botao_thread, daemon=True).start()

    try:
        while True:
            try:
                robo_ativo.wait()
                achar_saida()
            except RoboParado:
                print("[SAIDA] Parado pelo botao")
                parar()
            except KeyboardInterrupt:
                print("Finalizado")
                break
            except Exception as e:
                print("[ERRO] achar_saida:", repr(e))
                parar()
    finally:
        parar()
        GPIO.gpiochip_close(h)
        cv2.destroyAllWindows()
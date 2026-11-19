
import fcntl
import math
import os
import subprocess
import sys
import threading
import time
from threading import Thread

import cv2
try:
    cv2.setLogLevel(2)  # 2 = ERROR
except Exception:
    pass
import lgpio
import serial
import serial.tools.list_ports
import smbus2

# =========================================================================
# ESTADO DE EXECUCAO / PARADA IMEDIATA
# =========================================================================
class RoboParado(Exception):
    pass


robo_ativo = threading.Event()
ENTREGAS_HERDADAS = 0

# Existe = a saida ja comecou nessa rodada: ao achar a sala de resgate de novo
# o seguidor vai direto pro minha_saida.py, sem refazer o resgate. Criado pelo
# minha_saida.py, apagado pelo main.py quando ele e' iniciado do zero.
ARQUIVO_SAIDA_INICIADA = "/tmp/robo_dogo_saida_iniciada"
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
    if not robo_ativo.is_set():
        raise RoboParado()


# =========================================================================
# AJUSTE FINO
# =========================================================================
# NAO e' aqui que muda: o VEL_MAX de verdade fica no seguimento_de_linha.py
# (junto do VEL_BASE) e sobrescreve este assim que ele e' importado. Este so'
# vale pra script que importa config sem o seguidor (ex: teste_velocidade_reta.py).
VEL_MAX = 21
VEL_MIN = -VEL_MAX

GANHO_RODA_TRAS = 1.00  # 7% a mais - testado ~10 unidades a mais na faixa de velocidade usual

camera_x = 160
camera_y = 120


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


# =========================================================================
# MOTORES: placa Yahboom via serial
# =========================================================================
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70  # converte a faixa antiga do L298N (-50..50) pra PWM Yahboom (-3600..3600)
PWM_FREIO_YAHBOOM = 700   # forca do pulso de freio reverso ao parar
TEMPO_FREIO_YAHBOOM = 0.08  # segundos do pulso de freio

ANGULO_INICIAL = 90

SERVO_PESADO_ANGULO_ATIVO = 140
SERVO_PESADO_ANGULO_PARADO = 90
TEMPO_ASSENTAMENTO_SERVO_DIRETO = 0.4

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


motor_serial = _aguardar_porta_yahboom()


def _enviar_pwm_yahboom(m1, m2, m3, m4):
    m1, m2, m3, m4 = (max(-3600, min(3600, int(v))) for v in (m1, m2, m3, m4))
    try:
        motor_serial.write(f"$pwm:{m1},{m2},{m3},{m4}#".encode())
    except serial.SerialException as e:
        print(f"[MOTOR][ERRO] Falha ao mandar PWM pra placa Yahboom: {e}")
        reiniciar_processo()


_ultima_vel = (0, 0)


rampa_ativa = False  # True so enquanto o seguidor esta subindo a rampa/gangorra: libera o teto de velocidade
VEL_MAX_RAMPA = 80   # teto por roda na rampa (o PWM da placa satura em 3600, ~51 unidades)


def aplicar_motor(velL, velR, vel_tras=None, vel_m2=None):
    """vel_tras != None: rodas TRASEIRAS (M2/M3) nessa velocidade fixa, so
    nesse comando (o seguidor passa na rampa mais inclinada). Qualquer outro
    comando - parar(), pausa, gap, re - vem sem ele e as 4 rodas obedecem.
    vel_m2 != None: so a M2 (traseira esquerda) nessa velocidade, nesse comando."""
    teto = VEL_MAX_RAMPA if rampa_ativa else VEL_MAX
    velL = max(min(velL, teto), -teto)
    velR = max(min(velR, teto), -teto)
    pwm_esq = -velL * ESCALA_PWM_YAHBOOM
    pwm_dir = -velR * ESCALA_PWM_YAHBOOM
    pwm_tras_esq = pwm_esq * GANHO_RODA_TRAS
    pwm_tras_dir = pwm_dir * GANHO_RODA_TRAS
    if vel_tras is not None:
        pwm_tras_esq = pwm_tras_dir = -vel_tras * ESCALA_PWM_YAHBOOM
    if vel_m2 is not None:
        pwm_tras_esq = -vel_m2 * ESCALA_PWM_YAHBOOM
    _enviar_pwm_yahboom(
        pwm_dir,                       # M1 dianteira direita
        pwm_tras_esq,                  # M2 traseira esquerda
        pwm_tras_dir,                  # M3 traseira direita
        pwm_esq,                       # M4 dianteira esquerda
    )


def mover(velL, velR, vel_tras=None, vel_m2=None):
    global _ultima_vel
    aplicar_motor(velL, velR, vel_tras, vel_m2)
    _ultima_vel = (velL, velR)


def parar():
    global _ultima_vel
    velL, velR = _ultima_vel
    if velL != 0 or velR != 0:
        freio_vel = PWM_FREIO_YAHBOOM / ESCALA_PWM_YAHBOOM
        freio_esq = -freio_vel if velL > 0 else (freio_vel if velL < 0 else 0)
        freio_dir = -freio_vel if velR > 0 else (freio_vel if velR < 0 else 0)
        aplicar_motor(freio_esq, freio_dir)
        time.sleep(TEMPO_FREIO_YAHBOOM)
    mover(0, 0)


parar_motores = parar


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
    if not hasattr(mover_servo, "_angulo_atual"):
        mover_servo._angulo_atual = {}
    angulo_alvo = max(0, min(180, angulo_alvo))
    largura_pulso = pulso_min + (angulo_alvo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    mover_servo._angulo_atual[pino] = angulo_alvo

    time.sleep(TEMPO_ASSENTAMENTO_SERVO_DIRETO)
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error as e:
        print(f"Falha ao desligar PWM do servo (pino={pino}): {e}")


def reiniciar_processo():
    print("[RECONEXAO] Falha de conexao (motor/camera) - reiniciando processo do zero...")
    try:
        parar()
    except Exception as e:
        print(f"[RECONEXAO][ERRO] Falha ao parar motores antes de reiniciar: {e}")
    try:
        if cap is not None:
            cap.release()
    except Exception as e:
        print(f"[RECONEXAO][ERRO] Falha ao liberar camera antes de reiniciar: {e}")
    try:
        if ser is not None:
            ser.close()
    except Exception as e:
        print(f"[RECONEXAO][ERRO] Falha ao fechar serial do sonar antes de reiniciar: {e}")
    try:
        if motor_serial is not None:
            motor_serial.close()
    except Exception as e:
        print(f"[RECONEXAO][ERRO] Falha ao fechar serial do motor antes de reiniciar: {e}")
    try:
        lgpio.gpiochip_close(h)
    except Exception as e:
        print(f"[RECONEXAO][ERRO] Falha ao fechar GPIO antes de reiniciar: {e}")
    os.execv(sys.executable, [sys.executable] + sys.argv)


def alternar_botao():
    if robo_ativo.is_set():
        robo_ativo.clear()
        parar()
        print("[BOTAO] PAUSADO (aperte de novo pra retomar)")
        try:
            mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_PARADO)
        except Exception as e:
            print(f"[BOTAO][ERRO] Falha ao abaixar servo pesado: {e}")
    else:
        print("[BOTAO] RETOMANDO...")
        robo_ativo.set()
        try:
            mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_ATIVO)
        except Exception as e:
            print(f"[BOTAO][ERRO] Falha ao levantar servo pesado: {e}")
    return robo_ativo.is_set()


def botao_thread():
    # O "botao" e' um interruptor gangorra: cada virada muda o nivel do pino e
    # ele FICA la. Entao qualquer mudanca de nivel (0->1 ou 1->0) liga/desliga
    # o robo - reagir so a 0->1 ignorava metade das viradas (dois toques).
    # `anterior` comeca no nivel real do pino pra nao ver uma virada falsa no
    # boot (era isso que fazia o robo sair andando sozinho).
    anterior = lgpio.gpio_read(h, BUTTON)
    while True:
        estado = lgpio.gpio_read(h, BUTTON)
        if estado != anterior:
            time.sleep(0.05)  # repique do contato
            estado = lgpio.gpio_read(h, BUTTON)
            if estado != anterior:
                alternar_botao()
        anterior = estado
        time.sleep(0.02)


# =========================================================================
# BNO055 DIRETO NA RASP
# =========================================================================
_gyro_lock = threading.Lock()
BNO_ADDR        = 0x29  # 0x28 se o pino ADR do modulo estiver em GND
REG_CHIP_ID     = 0x00
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL    = 0x3B
GYR_DATA_X      = 0x14  # giroscopio cru (x,y,z) - 16 LSB = 1 grau/s
ACC_DATA_X      = 0x08  # acelerometro cru (x,y,z)
EUL_DATA_HEADING = 0x1A  # heading (yaw) ja calculado pela fusao - 16 LSB = 1 grau
MODO_CONFIG     = 0x00
MODO_ACCGYRO    = 0x05
MODO_IMUPLUS    = 0x08  # fusao 6DOF (acel+giro) sem magnetometro - da heading/roll/pitch prontos
CHIP_ID_ESPERADO = 0xA0
LIMITE_DPS_PLAUSIVEL = 2000.0
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

_angulo_y_filtrado = None
_angulo_y_rapido = None
_heading_offset = 0.0
_ultimo_angulo_z = [0]  # cache do ultimo angulo_z() valido, pra nao quebrar guinada numa falha I2C isolada


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


def angulo_y(): return angulo_y_estavel()


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
    _bno_write(REG_SYS_TRIGGER, 0x20)  # reset
    time.sleep(0.65)
    for _ in range(20):
        try:
            if bus.read_byte_data(BNO_ADDR, REG_CHIP_ID) == CHIP_ID_ESPERADO:
                break
        except OSError:
            pass
        time.sleep(0.05)
    _bno_write(REG_PWR_MODE, 0x00)  # normal
    time.sleep(0.02)
    _bno_write(REG_UNIT_SEL, 0x00)  # graus, graus/s, m/s^2
    time.sleep(0.02)
    _bno_write(REG_OPR_MODE, MODO_IMUPLUS)
    time.sleep(0.03)
    print("[BNO055] Calibrando...")
    _calibrar()
    resetar_giroscopio()
    print("[BNO055] Pronto")
    Thread(target=_gyro_thread, daemon=True).start()


# ==========================================================
# ANGULO Y ESTAVEL
# ==========================================================
def angulo_y_estavel(alpha=0.97):
    global _angulo_y_filtrado

    try:
        ax = _bno_read_word(ACC_DATA_X)
        ay = _bno_read_word(ACC_DATA_X + 2)
        az = _bno_read_word(ACC_DATA_X + 4)

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
        print(f"[BNO055] Falha I2C ({e}), mantendo último valor.")
        # NÃO derruba o programa

    if _angulo_y_filtrado is None:
        return 0

    return int(_angulo_y_filtrado)

def angulo_y_rapido(alpha=0.8):
    """Mesmo angulo do angulo_y_estavel, mas com filtro bem mais leve (0.8 em
    vez de 0.97) e com estado proprio - reage rapido, usado so pra detectar a
    saida da rampa. Nao mexe no angulo_y() normal."""
    global _angulo_y_rapido
    try:
        ax = _bno_read_word(ACC_DATA_X)
        ay = _bno_read_word(ACC_DATA_X + 2)
        az = _bno_read_word(ACC_DATA_X + 4)
        angulo = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        if _angulo_y_rapido is None:
            _angulo_y_rapido = angulo
        else:
            _angulo_y_rapido = alpha * _angulo_y_rapido + (1 - alpha) * angulo
    except OSError as e:
        print(f"[BNO055] Falha I2C ({e}), mantendo último valor.")
    if _angulo_y_rapido is None:
        return 0
    return _angulo_y_rapido



# =========================================================================
# SERIAL - so sonar (F e E)
# =========================================================================
PORTA_FIXA = "/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0"
SERIAL_TIMEOUT_DADO = 0.5
WATCHDOG_RECONEXAO = 2.0

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
    porta_motor = motor_serial.port if motor_serial is not None else None

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
# RESET DE USB VIA SOFTWARE
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


def _achar_bus_dev_por_vid(vid):
    base = "/sys/bus/usb/devices"
    try:
        for nome in os.listdir(base):
            d = os.path.join(base, nome)
            try:
                with open(os.path.join(d, "idVendor")) as f:
                    if f.read().strip().lower() != vid:
                        continue
                with open(os.path.join(d, "busnum")) as f:
                    busnum = int(f.read().strip())
                with open(os.path.join(d, "devnum")) as f:
                    devnum = int(f.read().strip())
                return busnum, devnum
            except (OSError, ValueError):
                continue
    except OSError:
        pass
    return None, None


def _achar_porta_usb_por_vid(vid):
    base = "/sys/bus/usb/devices"
    try:
        for nome in os.listdir(base):
            if ":" in nome or nome.startswith("usb"):
                continue
            try:
                with open(os.path.join(base, nome, "idVendor")) as f:
                    if f.read().strip().lower() != vid:
                        continue
            except OSError:
                continue
            if "." in nome:
                hub, porta = nome.rsplit(".", 1)
            else:
                hub, porta = nome.split("-", 1)
            return hub, porta
    except OSError:
        pass
    return None, None


USB_CICLO_ENERGIA_SEGUNDOS = 3


def ciclar_energia_usb(vid):
    hub, porta = _achar_porta_usb_por_vid(vid)
    if hub is None:
        print(f"[USB][AVISO] Ciclo de energia: nao achei device {vid} no USB (cabo solto?)")
        return False
    uhubctl = "/usr/sbin/uhubctl"
    if not os.path.exists(uhubctl):
        print("[USB][AVISO] uhubctl nao instalado - sem ciclo de energia")
        return False
    print(f"[USB] Cortando energia da porta {hub}.{porta} por {USB_CICLO_ENERGIA_SEGUNDOS}s e religando...")
    try:
        r = subprocess.run(
            ["sudo", "-n", uhubctl, "-l", hub, "-p", porta, "-a", "cycle",
             "-d", str(USB_CICLO_ENERGIA_SEGUNDOS)],
            capture_output=True, text=True, timeout=USB_CICLO_ENERGIA_SEGUNDOS + 15)
    except Exception as e:
        print(f"[USB][ERRO] Falha no ciclo de energia: {e}")
        return False
    if r.returncode != 0:
        print(f"[USB][ERRO] uhubctl retornou {r.returncode}: {(r.stderr or r.stdout).strip()[:150]}")
        return False
    time.sleep(4.0)
    print("[USB] Ciclo de energia concluido")
    return True


def resetar_usb_camera(video_path_ou_indice, vid=None):

    if isinstance(video_path_ou_indice, int):
        video_path = f"/dev/video{video_path_ou_indice}"
    else:
        video_path = video_path_ou_indice

    ok = False
    try:
        busnum, devnum = (None, None)
        if os.path.exists(video_path):
            busnum, devnum = _achar_bus_dev(video_path)
        if busnum is None and vid:
            busnum, devnum = _achar_bus_dev_por_vid(vid)
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

    time.sleep(1.0)
    return ok


# =========================================================================
# CAMERA DE LINHA
# =========================================================================
CAMERA_LINHA_PATH  = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK  = 0
CAMERA_VITIMA_INDEX_FALLBACK = 2
CAMERA_VITIMA_PATH = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CAMERA_LINHA_USB_VID = "4c4a"

USB_CICLO_ENERGIA_A_PARTIR_DA_TENTATIVA = 2
USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 1  # 3 - falhas seguidas antes de tentar resetar USB (cada falha custa ~20 s de select() timeout, 3 = mais de 1 min travado no boot)

LIMIAR_DESVIO_FRAME_VALIDO = 3.0

cap = None  # atribuido por iniciar_camera()


def abrir_camera_generica(fonte):
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


def _e_camera_de_vitima(fonte):
    alvo = _resolver_dev_real(fonte)
    vitima = _resolver_dev_real(CAMERA_VITIMA_PATH)
    return alvo is not None and alvo == vitima


def _e_no_de_captura(i):
    if not os.path.exists(f"/dev/video{i}"):
        return False
    try:
        with open(f"/sys/class/video4linux/video{i}/index") as f:
            if f.read().strip() != "0":
                return False
        driver = os.path.realpath(f"/sys/class/video4linux/video{i}/device/driver")
        return os.path.basename(driver) == "uvcvideo"
    except OSError:
        return True  # sem info no sysfs - deixa tentar


def detectar_camera():
    if CAMERA_LINHA_PATH and os.path.exists(CAMERA_LINHA_PATH):
        c = abrir_camera_generica(CAMERA_LINHA_PATH)
        if c is not None:
            print(f"[Camera] Linha conectada em {CAMERA_LINHA_PATH}")
            return c

    elif CAMERA_LINHA_PATH:
        print(f"[Camera][AVISO] Camera de linha sumiu do USB ({CAMERA_LINHA_PATH} nao existe) - cabo/conector?")

    if _e_no_de_captura(CAMERA_LINHA_INDEX_FALLBACK) and not _e_camera_de_vitima(CAMERA_LINHA_INDEX_FALLBACK):
        c = abrir_camera_generica(CAMERA_LINHA_INDEX_FALLBACK)
        if c is not None:
            print(f"[Camera] Linha conectada no indice {CAMERA_LINHA_INDEX_FALLBACK}")
            return c

    for i in range(10):
        if i == CAMERA_LINHA_INDEX_FALLBACK or not _e_no_de_captura(i) or _e_camera_de_vitima(i):
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
            if tentativas >= USB_CICLO_ENERGIA_A_PARTIR_DA_TENTATIVA:
                print("[Camera] Reset por software nao resolveu, cortando a energia da porta USB...")
                if not ciclar_energia_usb(CAMERA_LINHA_USB_VID):
                    resetar_usb_camera(CAMERA_LINHA_PATH or CAMERA_LINHA_INDEX_FALLBACK,
                                       vid=CAMERA_LINHA_USB_VID)
            else:
                print("[Camera] Falha na camera, tentando resetar USB...")
                resetar_usb_camera(CAMERA_LINHA_PATH or CAMERA_LINHA_INDEX_FALLBACK,
                                   vid=CAMERA_LINHA_USB_VID)
        cap_local = detectar_camera()
    cap_local.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap_local.set(3, camera_x)
    cap_local.set(4, camera_y)
    cap = cap_local
    return cap_local


from movimentos_bases import guinada2

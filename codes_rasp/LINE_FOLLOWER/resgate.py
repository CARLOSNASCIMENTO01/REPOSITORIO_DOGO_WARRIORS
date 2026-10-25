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

# So pra acionar_saida(): area minima maior que o normal, exigindo um blob
# bem mais confiante antes de aceitar como canto - na saida um falso
# positivo (sombra/reflexo/vazamento de cor perto da abertura de verdade)
# custa caro (desvia o robo da saida real), entao vale ser mais rigoroso
# aqui do que durante a entrega normal (onde o triangulo as vezes ainda
# esta longe/pequeno e precisa do limiar mais baixo pra ser visto a tempo).
AREA_MIN_TRI_SAIDA = 6000

STOP_AREA_RATIO_TRIANGULO = 0.80
TRIANGULO_MAX_GIROS = 90  # passos de turn_right(15)+0.3s por volta de 360 no buscar_triangulo()
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_APPROACH_SPEED = 20
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 15
TRIANGULO_CENTER_LOST_FRAMES = 15

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best_inox.onnx")
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240

# Grava o percurso inteiro com essa mesma camera (o main.py tem a MESMA
# chave, independente, pro trecho do seguidor) - True liga, False desliga
# sem apagar codigo. Aqui NAO abre uma segunda captura (a camera ja esta
# aberta em cap pra bola/triangulo) - so escreve os frames que ja sao lidos
# normalmente num video_rounds/<round>_resgate_N.mp4, mesmo <round> do
# main.py (persistido em /tmp, ver config.py/movimentos_bases.py pro mesmo
# esquema com o norte absoluto).
GRAVAR_PERCURSO = False
VIDEO_ROUNDS_DIR = "video_rounds"
_ARQUIVO_ROUND_VIDEO = "/tmp/robo_dogo_video_round.txt"

# camera de resgate: 70 graus de FOV nominal, capturada em 1280x720 mas
# center_crop() so usa o centro em 320x240 pra deteccao - ou seja, so 25%
# da largura capturada. FOV de verdade usado na deteccao (geometria de
# lente): 2*atan(0.25*tan(35)) ~= 20 graus, bem menor que os 70 nominais.
# Girar em passos desse tamanho (em vez de tempo fixo/chute) garante que a
# varredura de 360 graus nao deixa buraco entre um giro e outro.
GRAUS_POR_PASSO_BUSCA = 20
MAX_PASSOS_BUSCA_360 = int(360 / GRAUS_POR_PASSO_BUSCA) + 1  # +1 de folga contra deriva do giroscopio
VARREDURAS_360_ANTES_REPOSICIONAR = 1  # quantas voltas de 360 buscar_bola() da (reposicionando entre elas) antes de desistir
VARREDURAS_360_TRIANGULO_ANTES_REPOSICIONAR = 2  # idem, mas pra buscar_triangulo() (giro continuo, nao guinada2)
MAX_TENTATIVAS_BUSCA_FINAL = 3  # quantas "bolas sobrando" a busca final tenta pegar antes de desistir e ir pra saida
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

DEBUG_VISUAL = False

YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70
PWM_FREIO_YAHBOOM = 700
TEMPO_FREIO_YAHBOOM = 0.08

VEL_MAX = 20
VEL_MIN = -VEL_MAX

# 3 niveis de aproximacao pela ratio (area da bola / area do crop - quanto
# menor, mais longe): longe (passo grande, mais tempo, cobre distancia
# rapido) -> medio (passo atual, "normal") -> perto (passo curto/lento, pra
# nao passar do ponto certo perto do STOP_AREA_RATIO).
APPROACH_RATIO_LONGE = 0.15  # abaixo disso e "longe" - usa o passo grande
APPROACH_SPEED_LONGE = 45
APPROACH_STEP_TIME_LONGE = 0.28

APPROACH_SPEED = 30#25
APPROACH_STEP_TIME = 0.18#0.18
# Perto do limiar de parada (STOP_AREA_RATIO), pulso maior/velocidade alta
# facilmente passa do ponto certo - a partir de APPROACH_RATIO_PERTO troca
# pra passos mais curtos e mais lentos, pra acertar a distancia certinho.
APPROACH_RATIO_PERTO = 0.5
APPROACH_SPEED_PERTO = 15
APPROACH_STEP_TIME_PERTO = 0.08
STOP_AREA_RATIO = 0.70
MAX_APPROACH_STEPS = 60

# Depois de fechar/levantar a garra e recuar (fim da tentativa de pegar),
# confere de novo se ainda ve uma bola grande na frente - se ver, a garra
# errou (bola ficou no chao, nao dentro da garra). Calibrar ao vivo - e a
# area esperada de uma bola no chao logo na frente apos o recuo do final
# da sequencia de garra (mover(-25,-25) em pegar_e_entregar_bola()).
LIMIAR_AREA_BOLA_NAO_PEGA = 0.15
MAX_TENTATIVAS_PEGAR_BOLA = 3  # quantas vezes tenta pegar de novo se a garra errar antes de desistir dessa bola

# True logo depois do MOVIMENTO de captura (fechar garra), False so depois
# de entregar_bola() rodar de verdade. Se o robo for comecar a procurar de
# novo com isso ainda True (fez a captura mas nunca chegou a entregar -
# seja porque bola_ainda_no_chao() avaliou que errou, seja por qualquer
# outro motivo), solta a garra ANTES de procurar - defensivo, pra nao
# ficar carregando uma bola presa por engano enquanto vai atras de outra.
_garra_com_captura_nao_entregue = False
LOST_BALL_FRAMES = 8
TURN_PULSE = 0.2#0.16
STOP_TIME = 0.1#0.2
FORWARD_TIME = 0.6

CENTER_TURN_SPEED_MIN = 15
CENTER_TURN_SPEED_MAX = 25
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5

# Ordem de entrega: as duas primeiras bolas tem que ser prata, a ultima
# preta (ver classe_bola_alvo() mais abaixo). MAX_ENTREGAS tava em 0 por
# engano - com isso, a primeira entrega ja disparava "busca final" e ia
# direto pra saida depois de UMA bola so, nunca entregando as outras duas.
QTD_PRATA_ENTREGAS = 2
QTD_PRETA_ENTREGAS = 1
MAX_ENTREGAS = QTD_PRATA_ENTREGAS + QTD_PRETA_ENTREGAS

# Regra ASR_2026: se True, ao entregar a vitima MORTA (bola preta) o robo
# fica parado TEMPO_PAUSA_VITIMA_MORTA_ASR segundos antes de continuar.
# False desliga a pausa sem precisar apagar o codigo.
ASR_2026 = True
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

# codigo_arduino.ino manda exatamente 255.0 quando o eco do ultrassonico nao
# volta a tempo (pulseIn timeout) OU quando a leitura sai da faixa valida -
# ou seja, "255" tanto pode ser "nao tem nada por perto" quanto "sensor
# falhou nessa leitura", sem distincao nenhuma. Isso fazia toda falha de
# leitura (rotina em sonar ultrassonico perto de parede angulada) ser lida
# como "lateral aberta" pelo DIST_LATERAL_ABERTA_CM, causando falso positivo
# ~4 em cada 10 vezes. distancia_frente()/distancia_esquerda() abaixo
# reconhecem esse 255 explicito e devolvem SONAR_INVALIDO (9999, mesmo
# sentinela ja usado em dados{} antes do primeiro dado chegar) no lugar -
# quem checa "lateral aberta" tem que excluir esse valor, nao so comparar
# ">" com DIST_LATERAL_ABERTA_CM.
SONAR_INVALIDO = 9999
LIMIAR_ERRO_SONAR_CM = 254  # >= isso e o "255.0" de erro do firmware (folga de arredondamento)

FRAMES_CONFIRMACAO_LATERAL = 5  # leituras seguidas exigidas antes de confiar que a lateral abriu

VEL_SAIDA = 15
TEMPO_MAX_CONFIRMAR_LATERAL = 1.0

USBDEVFS_RESET = 21780
USB_RESET_TENTATIVAS_ANTES_DE_RESETAR = 3

# So "ret=True" no read() nao basta pra aceitar uma camera - depois de um
# reconnect/reset USB o device pode abrir e ler frame "com sucesso" mas
# entregar imagem toda preta/travada (interface errada, sensor ainda nao
# assentou). frame.std() de uma imagem preta/uniforme fica ~0; qualquer
# cena real (mesmo escura) tem variacao de pixel bem maior que isso.
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
CLASSE_POR_NOME = {}  # nome normalizado ("bola-prata"/"bola-preta") -> class_id do modelo
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

    # A porta da placa Yahboom (motor_serial) nunca pode ser tambem
    # escolhida aqui pro Arduino - senao os dois ficam abrindo o MESMO
    # device concorrentemente (visto ao vivo: PWM do motor falhando com
    # I/O error e o sonar nunca recebendo dado valido, preso reconectando
    # em loop). So acontece quando o ID fixo acima falha e cai nas buscas
    # genericas abaixo, mas vale excluir sempre. Mesma correcao ja aplicada
    # em config.py:detectar_porta().
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
    """Acha a porta serial da placa Yahboom pelo VID:PID (1a86:7522, CH340
    generico), nao por um caminho fisico fixo (by-path) - esse caminho e
    atrelado ao SLOT USB, nao ao dispositivo, entao se a placa mudar de
    porta um by-path antigo passaria a apontar pra outro device qualquer
    (ex: o Arduino), que abre serial sem erro nenhum mas nao e a Yahboom -
    o sintoma seria o robo "conectar" normal e os motores simplesmente nao
    se mexerem. Mesma correcao ja aplicada em config.py."""
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
    """Espera a placa Yahboom aparecer em vez de derrubar o processo
    inteiro se ela nao estiver plugada nesse instante do boot - antes,
    serial.Serial(YAHBOOM_PORTA, ...) direto no boot crashava o resgate
    inteiro se a placa nao estivesse pronta bem na hora."""
    porta = _detectar_porta_yahboom(sair_se_falhar=False)
    tentativas = 0
    while porta is None:
        tentativas += 1
        print(f"[MOTOR] Placa Yahboom ainda nao apareceu, tentando de novo... ({tentativas})")
        time.sleep(intervalo)
        porta = _detectar_porta_yahboom(sair_se_falhar=False)
    # CH340/firmware da placa costuma resetar quando a serial e reaberta -
    # sem essa espera os primeiros $pwm: saem enquanto ela ainda esta
    # subindo e sao ignorados sem erro nenhum.
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
    """True so com leitura valida (nao o 255 de erro do firmware) e de
    verdade acima do limiar - usado com confirmacao em varios frames
    seguidos (FRAMES_CONFIRMACAO_LATERAL) antes de agir, pra nao reagir a
    uma falha de leitura isolada do ultrassonico."""
    d = distancia_esquerda()
    return sonar_valido() and DIST_LATERAL_ABERTA_CM < d < SONAR_INVALIDO

def sonar_valido():
    return (time.time() - ultimo_recebimento) < SERIAL_TIMEOUT_DADO

def recuar_ate_ver_parede(vel_recuo=15, timeout=2.0):
    """Antes de virar pra dentro de uma abertura lateral: recua enquanto o
    sonar ainda le "aberto" (o robo pode ter avancado ate o pivo de giro
    ficar no meio do buraco) ate a leitura fechar de novo (parede solida
    logo atras da abertura) ou estourar o tempo limite. Depois avanca um
    pouco pra reposicionar antes do guinada2 de virar - assim o giro fica
    centrado na abertura, nao encostando na quina dela."""
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
    """So escreve os frames que o cap (ja aberto pra bola/triangulo) ja le
    de qualquer jeito - NAO abre uma segunda captura na mesma camera."""
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

def capturar_crop():
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    _escrever_frame_gravacao(frame)
    return center_crop(frame, CROP_W, CROP_H)

def classe_bola_alvo():
    """Qual classe de bola o robo deve procurar AGORA, baseado em quantas
    entregas ja foram feitas (entregas_realizadas - que ja honra
    ENTREGAS_HERDADAS entre reinicios via --entregas=N, entao sobrevive a
    troca de processo). Ordem exigida: QTD_PRATA_ENTREGAS bolas prata
    primeiro, so depois QTD_PRETA_ENTREGAS preta. Devolve o class_id do
    modelo, ou None se o modelo ainda nao carregou ou essa classe nao
    existir nele (nesse caso quem chamar deve tratar como "sem filtro")."""
    nome_alvo = "bola-prata" if entregas_realizadas < QTD_PRATA_ENTREGAS else "bola-preta"
    return CLASSE_POR_NOME.get(nome_alvo)

def so_falta_uma_bola_do_alvo():
    """True quando so resta 1 bola da classe alvo atual (classe_bola_alvo())
    em todo o campo - a preta sempre (so existe 1 no campo inteiro) e a
    prata so na ultima (quando ja entregou QTD_PRATA_ENTREGAS-1). Nesse
    caso, qualquer bola dessa classe vista, PERTO OU LONGE, so pode ser
    ela mesma - nao faz sentido ignorar so por estar longe (ver
    bola_ainda_no_chao())."""
    if entregas_realizadas >= QTD_PRATA_ENTREGAS:
        return True  # alvo e preta - so existe 1 no campo inteiro
    return entregas_realizadas == QTD_PRATA_ENTREGAS - 1

def escolher_alvo(boxes):
    """Prata tem prioridade sobre preta SEMPRE - mesmo que o alvo oficial do
    momento (classe_bola_alvo()) ja seja preta (quota de QTD_PRATA_ENTREGAS
    ja cumprida), se aparecer uma prata no frame (ex: uma sobrando alem da
    quota) o robo pega e entrega ela primeiro, so depois volta a procurar a
    preta. Devolve (idx, classe_desejada) prontos pra usar em
    centralizar_bola()/aproximar_bola()/pegar_e_entregar_bola()."""
    classe_prata = CLASSE_POR_NOME.get("bola-prata")
    if classe_prata is not None and boxes is not None:
        idx_prata = escolher_indice_mais_perto(boxes, classe_prata)
        if idx_prata is not None:
            return idx_prata, classe_prata

    classe_desejada = classe_bola_alvo()
    idx = escolher_indice_mais_perto(boxes, classe_desejada) if boxes is not None else None
    return idx, classe_desejada


def escolher_indice_mais_perto(boxes, classe_desejada=None):
    """Entre as caixas detectadas, escolhe a de maior area (mais perto).
    Se classe_desejada (class_id) for passada, SO considera caixas dessa
    classe - ignora as outras mesmo que maiores/mais pertas. Se nenhuma
    caixa bater com a classe desejada, devolve None (nao cai pra
    qualquer bola por engano)."""
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
    """Resolve indice/path pro /dev/videoN real por tras (segue symlink de
    by-id). None se nao existir/nao conseguir resolver."""
    if isinstance(fonte, int):
        fonte = f"/dev/video{fonte}"
    try:
        return os.path.realpath(fonte)
    except Exception:
        return None


def _e_camera_de_resgate(fonte):
    """True se `fonte` (indice ou path) resolve pro MESMO dispositivo
    fisico da camera de resgate (CAMERA_INDEX, ja aberta como `cap` nesse
    processo). A ordem de enumeracao USB NAO e fixa entre reconexoes (ja
    vimos mudar na pratica) - um indice fixo decorado nao bastava pra
    evitar que a busca da camera de baixo "roubasse" a camera de resgate
    sem perceber."""
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
    """Mesmo esquema robusto de abrir_camera_linha() (by-id -> varredura de
    indice -> reset USB a cada USB_RESET_TENTATIVAS_ANTES_DE_RESETAR falhas)
    - antes a camera de resgate so tentava abrir UMA vez no boot e, se
    falhasse (USB flakey, camera ainda enumerando), o processo inteiro
    morria com exit(1). Como main.py/systemd reinicia do zero (sem --auto)
    quando resgate.py morre assim, isso perdia a rodada inteira por uma
    falha transitoria de camera - exatamente o tipo de coisa que
    iniciar_camera()/abrir_camera_linha() ja sabem tentar de novo."""
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
    """Mascara preto/branco da camera de baixo (saida) - mesma logica de
    seguimento_de_linha.visao() (mesmo limiar/kernel/corte de topo, ver
    LIMIAR/kernal la), so que devolvendo so a mascara: acionar_saida()
    decide por CONTAGEM de pixel preto (cv2.countNonZero), nao por
    contorno, entao nao usa _maior_contorno_linha().

    Nao importa seguimento_de_linha.visao() de verdade de proposito - esse
    modulo importa config.py, que reivindica os MESMOS pinos GPIO que
    resgate.py ja reivindica sozinho no proprio boot (ver secao BOOT/
    INICIALIZACAO mais abaixo) - importar entraria em conflito de recurso
    na hora (dois claims na mesma linha GPIO, dentro do mesmo processo)."""
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
    """So confirma abertura lateral (sonar) como saida de verdade se o
    contorno visto na camera de baixo for HORIZONTAL - vertical e "entrada"
    (falso positivo, mesmo criterio de acionar_saida()) e a ausencia de
    contorno tambem nao confirma nada, so ignora e continua tentando."""
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

    # Nao precisa centralizar o contorno (cx) - so ver a linha aparecer ja
    # basta, igual confirmar_saida_lateral() ja fazia com detectar_linha_embaixo().
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
    """classe_desejada (class_id, opcional) filtra pra so considerar bolas
    dessa classe - usado pra manter o robo mirando no MESMO tipo de bola
    (prata ou preta, conforme classe_bola_alvo()) durante centralizacao/
    aproximacao, em vez de trocar de alvo se uma bola de outra cor aparecer
    mais perto no meio do caminho. None (padrao) = qualquer bola serve,
    igual antes - usado por buscar_bola()/verificar_e_afastar_bola_bloqueio()
    (busca final e desobstrucao aceitam qualquer bola de propósito)."""
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
    """Depois de fechar/levantar a garra e recuar, tira uma foto nova e
    confere se AINDA ve uma bola da classe procurada - se ver, a garra
    errou (a bola continua no chao, nao esta dentro da garra de verdade).

    Normalmente so conta como "ainda no chao" se a bola estiver PERTO
    (area grande) - uma vista longe pode ser outra bola da mesma classe em
    outro canto do campo, entao nao teria como saber se e a mesma que
    acabou de tentar pegar. MAS se so_falta_uma_bola_do_alvo() (so resta
    UMA bola dessa classe no campo inteiro), qualquer deteccao - perto ou
    longe - so pode ser ela mesma, entao conta como "nao pegou" direto,
    sem olhar a distancia."""
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
    """So defensivo - NAO e uma entrega de verdade (nao anda nem procura
    area de entrega, so abre/sacode a garra onde o robo estiver). Chamada
    antes de comecar a procurar de novo quando _garra_com_captura_nao_entregue
    ainda ta True, pra garantir que nao fica carregando por aí uma bola que
    uma captura anterior fechou em cima mas nunca chegou a entregar."""
    print("[SEGURANCA] Captura anterior nunca foi entregue - soltando a garra antes de procurar de novo")
    checar_parada()
    mover_servo(PINO_PESADO, 50, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)
    mover_servo(PINO_SG90, 180, velocidade=100)
    mover_servo(PINO_SG90, 120, velocidade=100)
    mover_servo(PINO_SG90, 180, velocidade=100)
    mover_servo(PINO_PESADO, 140, velocidade=100)

def pegar_e_entregar_bola(classe_desejada):
    """Centraliza, aproxima, fecha a garra e entrega uma bola ja avistada
    (pelo loop principal ou por buscar_bola() na busca final). Devolve True
    se pegou e entregou DE VERDADE (so ai incrementa entregas_realizadas) -
    False se perdeu a bola no meio do caminho (saiu do campo de visao,
    ou era so uma deteccao efemera/falso positivo que nao resistiu a
    centralizacao/aproximacao de verdade). Quem chamar deve tratar False
    como "nao tinha bola nenhuma ali", nao como erro.

    centralizar_bola() ja checa sozinha se a bola ja esta centralizada (cx
    dentro de TOLERANCE) e retorna True na hora sem girar - chamar ela aqui
    sempre, mesmo sem saber o cx atual de antemao, e seguro e barato.

    Depois de fechar a garra, confirma com bola_ainda_no_chao() que ela
    pegou de verdade - se a garra errou e a bola ficou no chao, tenta a
    sequencia inteira de novo (recentraliza/reaproxima/fecha garra) ate
    MAX_TENTATIVAS_PEGAR_BOLA vezes antes de desistir dessa bola."""
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
        angulo(180) #0 - esquema novo do copo: 0 antigo agora e 180
        mover(15,15)
        time.sleep(0.15)
        parar_motores()
        mover_servo(PINO_PESADO, 0, velocidade=100)
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
        _garra_com_captura_nao_entregue = True  # fechou a garra agora - so volta False depois de entregar_bola() de verdade

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

def _bola_confirmada(confirmacoes_necessarias):
    """Captura + detecta, e so aceita se a bola aparecer em varios frames
    seguidos sem girar mais (evita ruido/reflexo de 1 frame so)."""
    crop = capturar_crop()
    if crop is None:
        return False

    cx, area, bbox, cls_id = detectar_bola(crop)
    if cx is None:
        return False

    confirmacoes = 1
    for _ in range(confirmacoes_necessarias - 1):
        checar_parada()
        crop_confirma = capturar_crop()
        if crop_confirma is None:
            break
        cx_confirma, _, _, _ = detectar_bola(crop_confirma)
        if cx_confirma is None:
            break
        confirmacoes += 1

    if confirmacoes < confirmacoes_necessarias:
        print(f"[BUSCA FINAL] Deteccao efemera descartada ({confirmacoes}/{confirmacoes_necessarias} frames), continuando busca")
        return False
    return True


def buscar_bola():
    """Varredura de 360 graus por giros precisos do giroscopio (guinada2),
    nao mais por tempo/velocidade chutados - GRAUS_POR_PASSO_BUSCA e do
    tamanho do FOV de verdade da camera de resgate depois do center_crop
    (ver calculo onde a constante e definida), entao nao fica buraco cego
    entre uma foto e a proxima. Primeiro checa a posicao atual antes de
    girar - a bola pode ja estar no campo de visao sem precisar mexer.

    So uma volta de 360 - e a busca final (depois de MAX_ENTREGAS), nao
    precisa reposicionar/tentar de novo: se nao achou, e porque nao tem
    bola sobrando mesmo, e vai direto pra saida."""
    CONFIRMACAO_BUSCA_FINAL = 3

    checar_parada()
    if _bola_confirmada(CONFIRMACAO_BUSCA_FINAL):
        return True

    for _ in range(MAX_PASSOS_BUSCA_360):
        checar_parada()
        guinada2('D', GRAUS_POR_PASSO_BUSCA, 20)

        if _bola_confirmada(CONFIRMACAO_BUSCA_FINAL):
            return True

    return False

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
    """Reposicionamento mais agressivo que reposicionar_canto() - so pra
    quando a busca padrao completa VARREDURAS_360_ANTES_REPOSICIONAR voltas
    de 360 SEM ACHAR NENHUMA bola (nao e o caso de achar e perder no meio do
    caminho - isso continua usando reposicionar_canto() do jeito que ja
    estava). Se nem uma volta inteira acha nada, o robo provavelmente esta
    preso num canto/parede tampando a visao de verdade - recuo maior/mais
    rapido + giro de verdade (guinada2, nao so um pivot cru) pra sair logo
    dali, bem mais brusco que os passos curtos de reposicionar_canto()."""
    mover(30, 30)
    time.sleep(0.7)
    parar_motores()
    time.sleep(0.1)
    guinada2('D', 45, 25)
    mover(30, 30)
    time.sleep(0.8)
    parar_motores()

def buscar_triangulo():
    """Giro continuo (turn_right a velocidade fixa, sem parar entre fotos) -
    como era antes, nao os passos precisos de guinada2 usados em
    buscar_bola(). Da ate VARREDURAS_360_TRIANGULO_ANTES_REPOSICIONAR voltas
    de 360 (TRIANGULO_MAX_GIROS passos cada, com reposicionar_canto() entre
    elas) antes de desistir e devolver False pro entregar_bola()."""
    for varredura in range(VARREDURAS_360_TRIANGULO_ANTES_REPOSICIONAR):
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

        if varredura < VARREDURAS_360_TRIANGULO_ANTES_REPOSICIONAR - 1:
            print(f"[BUSCAR_TRIANGULO] Volta {varredura + 1}/{VARREDURAS_360_TRIANGULO_ANTES_REPOSICIONAR} sem achar, reposicionando...")
            reposicionar_canto(varredura + 1, vel_recuo=25)
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
    mover_servo(PINO_SG90, 120, velocidade=100) #60 - max, estava parando no meio do caminho #140, 130
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

def acionar_saida():
    mover_servo(PINO_PESADO, 130)
    abrir_camera_linha()  # cap_linha (camera de baixo) - nunca era chamada antes, ficava None pra sempre

    try:
        #movimentos_bases.guinada2('D', 80, 15)
        while True:
            mover(-15,15)
            if distancia_frente() < 80:
   
                break
   
        mover(28,30)
        time.sleep(4.0)
        mover(-17,-17)
        time.sleep(0.7)
        guinada2('D', 85, 15)
        while True:
            checar_parada()
   
            if not sonar_valido():
                time.sleep(0.02)
                continue
   
            frente = distancia_frente()
            esquerda = distancia_esquerda()
   
   
   
            ret, frame_baixo = cap_linha.read()
            if ret:
                mask = _mask_linha_baixo(frame_baixo)
                pixels_pretos = cv2.countNonZero(mask)
                if pixels_pretos > 20000:
                    print("muito pixel preto")
            else:
                pixels_pretos = 0
            print(f"pixels_pretos={pixels_pretos}")
   
   
            mover(16, 16)
            print(angulo_z())
            if frente < 9 :
                print("paredeeee")
                resetar_giroscopio()
                angulozero =angulo_z()
                mover(30, 30)
                time.sleep(1.5)
                if (angulo_z() - angulozero) < -20:
                    guinada2('D', 86, 15)
                    mover(19, 19)
                    time.sleep(0.9)
                    guinada2('D', 40, 15)
                else:
                    mover(-15, -15)
                    time.sleep(0.7)
                    guinada2('D', 84, 15)
   
            if 24000 < pixels_pretos < 28000:
                print("saida frente")
                mover(-15,-15)
                time.sleep(0.2)
                inicio_frente = time.time()
                while pixels_pretos <= 29000 and time.time() - inicio_frente <= 0.9:
                    checar_parada()
                    mover(15, 15)
                    ret2, frame2 = cap_linha.read()
                    if ret2:
                        mask2 = _mask_linha_baixo(frame2)
                        pixels_pretos = cv2.countNonZero(mask2)
                    time.sleep(0.02)
                parar_motores()
                print("pixels_pretos=", pixels_pretos)
                if pixels_pretos >= 29000:
                    print("saiiiiiii")
                    mover(15,15)
                    time.sleep(0.9)
                    guinada2('E', 20, 15)
                    alinhar_na_linha()
                    time.sleep(2.0)
                    parar_motores()
                    parar_gravacao_percurso()
                    cap.release()
                    if cap_linha is not None:
                        cap_linha.release()
                    GPIO.gpiochip_close(h)
                    os.execv(sys.executable, [
                        sys.executable,
                        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                        "--auto",
                        f"--entregas={entregas_realizadas}",
                    ])

                else:
                    mover(-15,-15)
                    time.sleep(2.0)
                    guinada2('D', 86, 15)
                    mover(18,18)
                    time.sleep(1.2)
            if esquerda > 40:
                print("buracooooo")
                mover(-2,-2)
                time.sleep(0.4)
                mover(15,15)
                time.sleep(0.4)
                guinada2('E', 86, 15)
   
                inicio_frente = time.time()
                while pixels_pretos <= 30000 and time.time() - inicio_frente <= 2.1:
                    checar_parada()
                    mover(15, 15)
                    ret2, frame2 = cap_linha.read()
                    if ret2:
                        mask2 = _mask_linha_baixo(frame2)
                        pixels_pretos = cv2.countNonZero(mask2)
                    time.sleep(0.02)
                parar_motores()
                print("pixels_pretos=", pixels_pretos)
                if pixels_pretos > 29000:
                    print("saiiiiiii")
                    mover(15,15)
                    time.sleep(0.9)
                    guinada2('E', 20, 15)
                    alinhar_na_linha()
                    time.sleep(2.0)
                    parar_motores()
                    parar_gravacao_percurso()
                    cap.release()
                    if cap_linha is not None:
                        cap_linha.release()
                    GPIO.gpiochip_close(h)
                    os.execv(sys.executable, [
                        sys.executable,
                        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                        "--auto",
                        f"--entregas={entregas_realizadas}",
                    ])
   
                else:
   
                    mover(-15,-15)
                    time.sleep(1.6)
                    guinada2('D', 86, 15)
                    if distancia_frente() < 15:
                        mover(-15,-15)
                        time.sleep(0.3) #0.3
                        guinada2('D', 86, 15)
   
                    mover(18,18)
                    time.sleep(1.2)
                    
    finally:
     parar_motores()

def finalizar_resgate():
    """Ja bateu MAX_ENTREGAS (seja pegando a ultima bola agora no loop
    principal, seja logo no BOOT por causa de um --entregas=N ja no maximo -
    reinicio apos falha de progresso/botao no meio/depois da saida, onde o
    seguidor achou a sala de novo e chamou resgate.py de novo por cima).
    Tenta pegar bola(s) sobrando com buscar_bola() ate MAX_TENTATIVAS_BUSCA_FINAL
    vezes e, sem achar mais nada, aciona a saida - nunca mais volta a
    procurar bola do zero. Nunca retorna normalmente (acionar_saida() so sai
    por execv ou RoboParado, ver comentario dela)."""
    print(f"[BOOT] {entregas_realizadas} entregas ja feitas, fazendo busca final antes da saida...")
    for _ in range(MAX_TENTATIVAS_BUSCA_FINAL):
        if not buscar_bola():
            break
        print("[BUSCA FINAL] Achou bola sobrando, tentando pegar e entregar...")
        if pegar_e_entregar_bola(classe_bola_alvo()):
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

    acionar_saida()

def voltar_pro_seguidor(motivo="Robo parado pelo botao, voltando pro seguidor..."):
    """Fecha tudo (motores/cameras/GPIO) e devolve o controle pro main.py,
    preservando entregas_realizadas via --entregas=N - usada tanto pelo
    RoboParado do loop principal quanto pelo boot-time finalizar_resgate()
    (se o botao for apertado durante a busca final/saida logo no boot)."""
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

# Reinicio (falha de progresso / botao) com as entregas ja no maximo - o
# seguidor achou a sala de novo e chamou resgate.py de volta, mas nao tem
# bola nenhuma pra procurar. Vai direto pra busca final/saida, sem entrar
# no loop de busca normal (que nunca ia chegar la sozinho, ja que aquela
# checagem so roda depois de PEGAR uma bola de verdade).
if entregas_realizadas >= MAX_ENTREGAS:
    try:
        finalizar_resgate()
    except RoboParado:
        voltar_pro_seguidor()

# ===================== LOOP PRINCIPAL =====================

try:
    _ultimo_heartbeat[0] = time.time()
    print("Iniciando busca por bolas...")
    passos_busca_padrao = 0
    while True:
        try:
            checar_parada()
            # Mesmo giro preciso pelo giroscopio de buscar_bola()/
            # buscar_triangulo() - GRAUS_POR_PASSO_BUSCA e do tamanho do FOV
            # real da camera de resgate depois do center_crop, entao a
            # varredura nao deixa buraco cego entre uma foto e a proxima.
            guinada2('D', GRAUS_POR_PASSO_BUSCA, 20)
            time.sleep(0.25)

            # Busca padrao roda pra sempre (nao desiste como a busca final) -
            # se der VARREDURAS_360_ANTES_REPOSICIONAR voltas inteiras sem
            # achar nada, o ponto onde o robo esta pode estar tampando a
            # visao (parede/quina perto de mais). Reposiciona e continua.
            passos_busca_padrao += 1
            if passos_busca_padrao >= MAX_PASSOS_BUSCA_360 * VARREDURAS_360_ANTES_REPOSICIONAR:
                print(f"[BUSCA] {VARREDURAS_360_ANTES_REPOSICIONAR} voltas de 360 sem achar NENHUMA bola, reposicionamento brusco...")
                reposicionar_brusco()
                passos_busca_padrao = 0

            for _ in range(4):
                cap.read()

            ret, frame = cap.read()
            if ret:
                _escrever_frame_gravacao(frame)
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
                # So reseta o contador de "sem achar nada" numa entrega DE
                # VERDADE (bola pega e entregue) - resetar so por causa de
                # uma deteccao (linha antiga fazia isso) deixava um falso
                # positivo persistente (sombra/reflexo classificado como
                # bola) impedir pra sempre o reposicionamento, porque toda
                # hora achava "alguma coisa" e zerava o contador de novo -
                # o robo ficava girando pra sempre atras de uma bola que
                # nao existe, sem nunca completar as voltas necessarias
                # pra reposicionar e tentar de outro angulo.
                passos_busca_padrao = 0

                # entregar_bola()/pegar_e_entregar_bola() acima ja podem ter
                # levantado RoboParado antes de chegar aqui - se bateu
                # MAX_ENTREGAS, finalizar_resgate() cuida da busca final e
                # da saida (nunca retorna normalmente).
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

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

# "Batimento" atualizado a cada checar_parada() - se ficar muito tempo sem
# atualizar, watchdog_seguranca() entende que o loop principal travou e
# forca os motores a pararem por conta propria.
_ultimo_heartbeat = [time.time()]
WATCHDOG_SEGURANCA_TIMEOUT = 4.0  # segundos sem heartbeat antes de forcar parada

def checar_parada():
    """Chamar a cada iteracao de todo while/for que segura o robo por mais
    de um instante (buscas, centralizacao, aproximacao, entrega, etc)."""
    _ultimo_heartbeat[0] = time.time()
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
                    # mover_servo() normal checa checar_parada() a cada passo -
                    # chamada aqui abortaria na hora (acabamos de limpar
                    # robo_ativo) e mataria essa propria thread. Por isso usa
                    # a versao direta, sem checagem.
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_PARADO)
                        _angulo_atual[PINO_PESADO] = SERVO_PESADO_ANGULO_PARADO
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao abaixar servo pesado: {e}")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_ATIVO)
                        _angulo_atual[PINO_PESADO] = SERVO_PESADO_ANGULO_ATIVO
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao levantar servo pesado: {e}")
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
            # Erro de I/O na porta (ex: [Errno 5], USB caiu) nem sempre vem
            # como serial.SerialException - sem zerar ser aqui, ficava
            # relendo a mesma porta morta pra sempre em loop apertado,
            # consumindo CPU/I2C e atrasando a thread do giroscopio.
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
    """Espera pelo menos uma leitura NOVA do sonar chegar antes de deixar
    o chamador confiar em distancia_frente()/distancia_esquerda() - sem
    isso, um giro (guinada2 etc) podia terminar e o valor ainda no dados[]
    ser de um instante ANTES, com o sonar ainda apontando pro angulo
    velho. Perto de canto/parede em diagonal isso e pior ainda (o eco
    muda muito com um angulo pequeno de diferenca), entao pode ler
    "corredor aberto" ou "parede" com o robo ja tendo virado pra outro
    lugar. So retorna quando ultimo_recebimento avancar de verdade.
    """
    referencia = ultimo_recebimento
    inicio = time.time()
    while ultimo_recebimento == referencia:
        checar_parada()
        if time.time() - inicio > timeout:
            break
        time.sleep(0.01)


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
# Preenchido automaticamente dentro de carregar_modelo() (ver mais abaixo):
# vira {cls_id: cor}.
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
TRIANGULO_APPROACH_SPEED = 20
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 15
TRIANGULO_CENTER_LOST_FRAMES = 15
# ===================== CONFIGURAÇÕES =====================
# Caminho absoluto baseado na localizacao do proprio arquivo, nao no
# diretorio de trabalho do processo - o script e chamado via execv() a
# partir de lugares diferentes (LINE_FOLLOWER/main.py, systemd com
# WorkingDirectory=PROTOTIPAGEM_LINE/, ou rodado a mao de outra pasta) e
# um caminho relativo simples ja quebrou o carregamento do modelo
# silenciosamente (FileNotFoundError numa thread, sem derrubar o processo).
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.onnx")
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

# IN1/IN2/ENA/IN3/IN4/ENB (L298N via GPIO) removidos - motores agora vao
# pela placa Yahboom via serial/USB (PWM), portado do seguidor_debug2_yahboom.py.
YAHBOOM_PORTA = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70  # converte a faixa antiga do L298N (-VEL_MAX..VEL_MAX) pra PWM Yahboom (-3600..3600)
PWM_FREIO_YAHBOOM = 700   # forca do pulso de freio reverso ao parar
TEMPO_FREIO_YAHBOOM = 0.08  # segundos do pulso de freio

VEL_MAX = 20
VEL_MIN = -VEL_MAX

# ===================== MODELO YOLO (carregamento em background) =====================
# "from ultralytics import YOLO" sozinho ja custa varios segundos (importa
# torch inteiro) e so roda depois do robo ja estar parado na area de resgate.
# Pra nao ficar parado esperando (tempo maximo parado e curto), o import e o
# YOLO(MODEL_PATH) rodam numa thread separada logo apos o setup do GPIO/motor;
# o loop principal ja comeca a girar procurando bola e so roda deteccao
# quando "model" deixar de ser None.
model = None

def carregar_modelo():
    global model
    # Evita o check de internet do ultralytics no import (is_online() faz
    # resolucao de DNS - se o wifi da competicao nao tiver internet isso pode
    # travar alguns segundos por timeout). O modelo e local (.onnx), nao
    # precisa de rede.
    os.environ.setdefault("YOLO_OFFLINE", "true")
    from ultralytics import YOLO

    def _normalizar_nome(nome):
        return str(nome).strip().lower()

    print(f"[MODELO] Carregando YOLO de {MODEL_PATH}...")
    try:
        m = YOLO(MODEL_PATH)
    except Exception as e:
        # Essa thread e daemon - se essa excecao nao for pega aqui, ela so
        # mata a thread em silencio, "model" fica None pra sempre e o robo
        # gira em busca de bola sem nunca detectar nada, sem nenhum aviso
        # claro do motivo (ja aconteceu: MODEL_PATH relativo quebrado pelo
        # diretorio de trabalho errado quando chamado via execv).
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
    """Enquanto o modelo ainda esta carregando (model is None), garante que
    o robo se mexe um pouquinho a cada 5s - nao pode passar de 10s parado
    ou a prova declara falha. Roda numa thread propria com timer fixo pra
    nao depender do loop de busca (que pode atrasar durante o import pesado
    do torch/ultralytics). Para sozinha assim que o modelo fica pronto."""
    while model is None:
        time.sleep(5.0)
        if model is not None:
            break
        if not robo_ativo.is_set():
            continue
        turn_right(speed=40)
        time.sleep(0.15)
        parar_motores()

def watchdog_seguranca():
    """Se o loop principal ficar mais de WATCHDOG_SEGURANCA_TIMEOUT segundos
    sem chamar checar_parada() (ex: travado no meio de um comando de motor),
    manda PWM 0 direto pra placa Yahboom, por conta propria."""
    while True:
        time.sleep(0.5)
        if time.time() - _ultimo_heartbeat[0] > WATCHDOG_SEGURANCA_TIMEOUT:
            print(f"[WATCHDOG SEGURANCA] Loop principal sem responder ha mais de "
                  f"{WATCHDOG_SEGURANCA_TIMEOUT:.0f}s - forcando parada dos motores")
            try:
                _enviar_pwm_yahboom(0, 0, 0, 0)
            except Exception:
                pass
            _ultimo_heartbeat[0] = time.time()  # evita disparar de novo a cada 0.5s

# ===================== APROXIMAÇÃO POR TAMANHO DE BBOX =====================
APPROACH_SPEED = 20        # velocidade do avanço durante a aproximação
APPROACH_STEP_TIME = 0.15  # duração de cada "passo" de avanço/correção
STOP_AREA_RATIO = 0.35     # bbox_area / crop_area -> ajustar na prática!
MAX_APPROACH_STEPS = 60    # trava de segurança (evita loop infinito)
LOST_BALL_FRAMES = 8       # quantos frames sem detectar até desistir
TURN_PULSE = 0.2
STOP_TIME = 0.2
FORWARD_TIME = 0.6

# ===================== CENTRALIZAÇÃO PROPORCIONAL =====================
CENTER_TURN_SPEED_MIN = 15   # giro mais fraco quando o erro é pequeno
CENTER_TURN_SPEED_MAX = 25   # giro mais forte quando o erro é grande
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5


MAX_ENTREGAS = 3
BUSCA_FINAL_MAX_GIROS = 20  # ajustar na prática

# ===================== CÂMERA DE BAIXO (linha/chão) - usada só na saída =====================
# Nao existe uma terceira camera fisica dedicada nesse robo - so tem a
# camera da linha (Jieli, apontada pro chao) e a de vitima/resgate. A do
# seguidor_debug2.py fica livre durante o resgate.py (os dois nunca rodam
# ao mesmo tempo, um substitui o outro via execv), entao reaproveitamos ela
# aqui, so ligada dentro de acionar_saida().
CAMERA_INDEX_LINHA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
USAR_TCRT_PARA_LINHA = False
CAMERA_LINHA_INDEX_FALLBACK = 0
LINHA_W, LINHA_H = 160, 120
LIMIAR_LINHA = 80
AREA_MIN_LINHA = 11000
kernel_linha = np.ones((3, 3), np.uint8)

# ===================== LIMIARES DA SAÍDA (acionar_saida) =====================

DIST_LATERAL_ABERTA_CM = 30   # muita distancia no ultrassonico lateral = campo aberto
DIST_PAREDE_FRENTE_CM = 10    # pouca distancia no ultrassonico da frente = parede
# Todas as janelas visuais de debug (deteccao de bola, canto verde/vermelho
# da entrega) foram removidas - nao dependem mais de display nenhum, entao
# nao tem mais risco do cv2.namedWindow() derrubar o processo rodando sem
# tela (ex: pelo robo.service).

# Pinos
PINO_SG90 = 5
PINO_PESADO = 20
TCRT = 6  # GPIO BCM 5

ANGULO_INICIAL = 90

# Servo pesado como indicador visual de ligado/desligado: levantado (150)
# enquanto o robo esta ativo, abaixado (0) quando parado pelo botao.
SERVO_PESADO_ANGULO_ATIVO = 140
SERVO_PESADO_ANGULO_PARADO = 10

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
MODO_ACCGYRO    = 0x05  # acel + giro crus, sem magnetometro e sem fusao interna (nao usado mais - so referencia)
MODO_IMUPLUS    = 0x08  # fusao 6DOF (acel+giro) sem magnetometro - da heading/roll/pitch prontos
CHIP_ID_ESPERADO = 0xA0
LIMITE_DPS_PLAUSIVEL = 2000.0  # fundo de escala do giroscopio - acima disso e leitura corrompida
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()
_heading_offset = 0.0  # heading (fusao) no momento do ultimo resetar_giroscopio() - vira o "zero" da guinada
_ultimo_angulo_z = [0]  # cache do ultimo angulo_z() valido, pra nao quebrar guinada numa falha I2C isolada

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


# ===================== SETUP MOTORES (placa Yahboom via serial) =====================
motor_serial = serial.Serial(YAHBOOM_PORTA, YAHBOOM_BAUDRATE, timeout=0.5)

# Dispara o carregamento do YOLO o mais cedo possivel (antes ate da camera
# abrir), pra dar o maximo de tempo de sobreposicao com o resto do boot -
# o robo nao fica esperando parado, ver comentario em carregar_modelo().
threading.Thread(target=carregar_modelo, daemon=True).start()
threading.Thread(target=watchdog_nudge, daemon=True).start()
threading.Thread(target=watchdog_seguranca, daemon=True).start()
# Sobe a serial do sonar cedo tambem - handshake_serial() sozinho leva uns 4s,
# entao por o handshake pra rodar em paralelo com o resto do boot em vez de
# so no acionar_saida() deixa o sonar ja "quente" (sonar_valido() == True)
# quando o robo finalmente precisar dele.
iniciar_serial()


# ===================== MOTORES (lgpio) =====================

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
def _bno_write(reg, val):
    bus.write_byte_data(BNO_ADDR, reg, val)

def _bno_read_word(reg, tentativas=3):
    for tentativa in range(tentativas):
        try:
            # BNO055 guarda LSB no endereco base e MSB no endereco+1 (ordem
            # OPOSTA do MPU6050, que guarda MSB primeiro) - inverter isso
            # aqui e o que fazia o giroscopio ler lixo (byte trocado) e o
            # angulo Z "explodir" pra qualquer valor entre -180 e 180.
            lo = bus.read_byte_data(BNO_ADDR, reg)
            hi = bus.read_byte_data(BNO_ADDR, reg + 1)

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

def _ler_heading_bruto():
    """Heading (yaw) 0-360 graus ja calculado pela fusao interna do chip -
    substitui a integracao manual do giro cru pra esse eixo."""
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

def angulo_x(): return int(ang_gyro["X"])
def angulo_y(): return angulo_y_estavel()

def angulo_z():
    """Yaw relativo ao ultimo resetar_giroscopio(), vindo do heading ja
    calculado pela fusao do chip (nao mais integracao de giro cru)."""
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
            # Lock tambem durante a leitura crua (nao so na atualizacao do
            # dict) - angulo_z() faz leitura I2C do heading fundido a partir
            # de qualquer thread que chame guinada, concorrente com essa
            # thread. Sem o lock aqui, as duas podiam intercalar leituras
            # multi-byte no mesmo barramento e corromper o valor lido.
            with _gyro_lock:
                gx = (_bno_read_word(GYR_DATA_X)     - gyro_offset["X"]) / 16.0
                gy = (_bno_read_word(GYR_DATA_X + 2) - gyro_offset["Y"]) / 16.0
                gz = (_bno_read_word(GYR_DATA_X + 4) - gyro_offset["Z"]) / 16.0
            # Fundo de escala do giroscopio e 2000 dps - qualquer valor acima
            # disso e fisicamente impossivel, e leitura corrompida (ruido
            # eletrico dos motores no barramento I2C, mais comum durante
            # guinada com motor puxando corrente). Integrar isso jogava o
            # angulo acumulado centenas de graus com 1 amostra so.
            if max(abs(gx), abs(gy), abs(gz)) > LIMITE_DPS_PLAUSIVEL:
                print(f"[BNO055] Amostra de giro implausivel descartada (X={gx:.0f} Y={gy:.0f} Z={gz:.0f} dps)")
                time.sleep(0.005)
                continue
            with _gyro_lock:
                ang_gyro["X"] = _normalizar(ang_gyro["X"] + gx * dt)
                ang_gyro["Y"] = _normalizar(ang_gyro["Y"] + gy * dt)
                ang_gyro["Z"] = _normalizar(ang_gyro["Z"] + gz * dt)
            # BNO055 so produz amostra nova de giro a ~100Hz (a cada 10ms) -
            # sem essa pausa a thread ficava martelando o I2C em loop fechado
            # o tempo todo (parado ou andando, nao tem como desligar), so
            # relendo a MESMA amostra varias vezes, ocupando um nucleo
            # inteiro a toa e disputando I2C/CPU com o resto do sistema.
            time.sleep(0.005)
        except OSError as e:
            print(f"[BNO055] Gyro I2C: {e}")
            time.sleep(0.005)

        except Exception as e:
            print(f"[BNO055] Gyro: {e}")
            time.sleep(0.005)

def iniciar_mpu():
    """Nome mantido por compatibilidade com iniciar_mpu_thread() logo
    abaixo - por dentro agora inicializa o BNO055."""
    _bno_write(REG_OPR_MODE, MODO_CONFIG)
    time.sleep(0.02)
    _bno_write(REG_SYS_TRIGGER, 0x20)  # reset
    time.sleep(0.65)
    for _ in range(20):  # espera o chip voltar a responder apos o reset
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

_mpu_thread_iniciada = [False]

def iniciar_mpu_thread():
    """Sobe a calibração + thread de leitura do giroscópio, uma vez só, sob demanda."""
    if _mpu_thread_iniciada[0]:
        return
    _mpu_thread_iniciada[0] = True
    Thread(target=iniciar_mpu, daemon=True).start()

# Estava definida mas nunca era chamada - sem isso ang_gyro["Z"] fica travado
# em 0.0 pra sempre (a thread do giroscopio nunca sobe) e angulo_z() nunca
# sai de 0. Toda guinada baseada em angulo_z() (guinada2, guinada_pid) nunca
# bate a condicao de parada e o robo gira infinitamente - era esse o motivo
# da guinada de 175 (e qualquer outra) nao terminar nunca. Chamada aqui, logo
# apos a definicao, pra calibracao (~1s de leitura parada do giroscopio) ja
# estar pronta bem antes da primeira guinada em acionar_saida().
iniciar_mpu_thread()

_angulo_y_filtrado = None

def angulo_y_estavel(alpha=0.97):
    global _angulo_y_filtrado

    try:
        # Sem normalizar pela escala (100 LSB = 1 m/s^2 no BNO055) porque
        # atan2 usa so a RAZAO entre os eixos - o fator de escala cancela.
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
            resetar_giroscopio()
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


def _enviar_pwm_yahboom(m1, m2, m3, m4):
    m1, m2, m3, m4 = (max(-3600, min(3600, int(v))) for v in (m1, m2, m3, m4))
    try:
        motor_serial.write(f"$pwm:{m1},{m2},{m3},{m4}#".encode())
    except serial.SerialException as e:
        print(f"[MOTOR][ERRO] Falha ao mandar PWM pra placa Yahboom: {e}")


_ultima_vel = (0, 0)  # (velL, velR) do ultimo mover(), pra parar_motores() saber pra que lado frear


def aplicar_motor(velL, velR):
    """Converte velocidade esquerda/direita (faixa antiga do L298N,
    -VEL_MAX..VEL_MAX) pra PWM da placa Yahboom nos 4 canais. Lado
    esquerdo = M2+M4, lado direito = M1+M3 (mapeamento real desse robo -
    ver comentario no bloco de CONFIG). Sinal invertido em todos os canais
    porque o motor fica montado espelhado no chassi."""
    velL = max(min(velL, VEL_MAX), VEL_MIN)
    velR = max(min(velR, VEL_MAX), VEL_MIN)
    pwm_esq = velL * ESCALA_PWM_YAHBOOM
    pwm_dir = velR * ESCALA_PWM_YAHBOOM
    # (M1, M2, M3, M4) = (direita, esquerda, direita, esquerda).
    _enviar_pwm_yahboom(pwm_dir, pwm_esq, pwm_dir, pwm_esq)


def mover(velL, velR):
    global _ultima_vel
    aplicar_motor(velL, velR)
    _ultima_vel = (velL, velR)


def parar_motores():
    global _ultima_vel
    velL, velR = _ultima_vel
    if velL != 0 or velR != 0:
        # pulso curto reverso antes de zerar, reduz a inercia em vez de so
        # cortar e deixar o robo deslizando
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
    # Zera de novo ao terminar - sem isso o angulo_z() ficava preso no valor
    # da guinada (ex: 90) durante todo o segmento reto seguinte, ate a
    # PROXIMA guinada zerar - confundia debug e deixava lixo acumulado
    # exposto pra qualquer outro codigo que va ler angulo_z() nesse meio tempo.
    resetar_giroscopio()


AREA_MAX_LINHA_FRAC = 0.5  # linha de verdade e fina, nao deveria cobrir mais que isso da ROI

def detectar_linha_embaixo(frame):
    """
    Mesma logica de deteccao de linha usada em obstaculos_trajeto.desvio()
    (LINE_FOLLOWER) pra voltar pra linha - nao so o maior contorno preto
    passar de AREA_MIN_LINHA, tambem rejeita contorno grande demais (mais
    da metade da ROI). Sem esse teto, um fundo/branco mal iluminado sendo
    lido como "preto" pelo LIMIAR_LINHA baixo passava por linha de
    verdade (mesmo bug ja visto e corrigido no desvio() do obstaculo).
    """
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False

    maior = max(contours, key=cv2.contourArea)
    area_contorno = cv2.contourArea(maior)
    area_roi = roi.shape[0] * roi.shape[1]

    if area_contorno < AREA_MIN_LINHA:
        return False
    if area_contorno >= area_roi * AREA_MAX_LINHA_FRAC:
        return False
    return True


VEL_SAIDA = 15  # velocidade continua do trecho reto de acionar_saida()


def acionar_saida():
    """
    Roda uma vez so, depois da ultima entrega. Primeiro procura o canto
    verde mais proximo, centraliza nele e VAI ATE ELE (mesmas buscar/
    centralizar/aproximar triangulo usadas em entregar_bola(), so que
    filtradas pra 'verde') - so depois de chegar no canto e que comeca o
    movimento continuo de identificacao dos tres sinais de saida:
      - canto verde OU vermelho na camera de resgate (achou outra quina,
        de qualquer cor - o filtro fica so em 'verde' durante o alinhamento
        inicial, depois solta pra None)
      - linha preta na camera de baixo (achou a pista)
      - ultrassonico lateral aberto (saiu pra um corredor)
    Retorna uma string dizendo qual dos tres foi o gatilho.
    """
    global ALVO_COR
    print("[SAIDA] Procurando canto verde para se alinhar...")
    ALVO_COR = "verde"

    # Nao pode desistir na primeira falha de centralizar/aproximar - reposiciona
    # (mesmo recuo usado em entregar_bola()) e tenta achar o canto de novo, ate
    # um limite de tentativas (pra nao travar pra sempre se o canto sumiu de
    # vez do alcance da camera).
    MAX_TENTATIVAS_ALINHAR_CANTO = 5
    alinhou_no_canto = False
    for _ in range(MAX_TENTATIVAS_ALINHAR_CANTO):
        checar_parada()

        if not buscar_triangulo():
            print("[SAIDA] Canto verde nao encontrado, reposicionando e tentando de novo...")
            mover(-15, -15)
            time.sleep(0.4)
            mover(-30, 0)
            time.sleep(0.2)
            continue

        if not centralizar_triangulo():
            print("[SAIDA] Nao conseguiu centralizar no canto verde, reposicionando e tentando de novo...")
            mover(-15, -15)
            time.sleep(0.4)
            mover(-30, 0)
            time.sleep(0.2)
            continue

        print("[SAIDA] Indo ate o canto verde...")
        if not aproximar_triangulo():
            print("[SAIDA] Nao conseguiu chegar no canto verde, reposicionando e tentando de novo...")
            mover(-15, -15)
            time.sleep(0.4)
            mover(-30, 0)
            time.sleep(0.2)
            continue

        alinhou_no_canto = True
        break

    if not alinhou_no_canto:
        print(f"[SAIDA] Estourou {MAX_TENTATIVAS_ALINHAR_CANTO} tentativas de alinhar no canto verde, seguindo reto sem alinhar")

    cam_linha = abrir_camera_linha()
    if cam_linha is None:
        # Sem a camera de baixo o robo nao tem como confirmar a linha preta
        # (um dos 3 sinais de saida) - sem ela, nao e seguro prosseguir com
        # a saida as cegas so no sonar/canto.
        print("[SAIDA] Camera de baixo indisponivel - abortando saida, nao e seguro prosseguir sem ela")
        return "sem_camera_linha"

    print(f"[SAIDA] Andando reto (vel={VEL_SAIDA}) ate achar canto verde/vermelho, "
          f"linha preta ou sonar lateral aberto...")
    mover(20,20)
    time.sleep(1.5)
    mover(-20,-20)
    time.sleep(0.7)
    guinada2('E', 175, 20)
    mover(-25,-25)
    time.sleep(3.5)
    guinada2('E', 90, 15)
    mover(20,20)
    time.sleep(0.7)
    guinada2('D', 45, 15)
    

    # A partir daqui o alinhamento no canto verde inicial ja foi feito -
    # solta o filtro de cor (None = aceita verde OU vermelho, ver
    # detectar_entrega()) pro loop continuo reconhecer qualquer um dos dois
    # como sinal de saida, nao so verde.
    ALVO_COR = None

    # Depois de uma guinada de alinhamento no canto, a camera ainda enxerga
    # o MESMO canto por um instante (nao sumiu do quadro so porque o robo
    # virou) - sem essa carencia, o primeiro frame do loop disparava
    # "achou canto" na hora, mesmo sem o robo ter andado pra frente de
    # verdade. So volta a valer a deteccao depois de TEMPO_IGNORAR_CANTO
    # segundos andando reto.
    TEMPO_IGNORAR_CANTO = 1.5
    ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO

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
                    mover(-20, -20)
                    time.sleep(0.2)
                    guinada2('D', 90, 15)
                    mover(15,15)
                    time.sleep(2.5)
                    guinada2('D', 42, 15)
                    ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO

            if cam_linha is not None:
                ret_l, frame_l = cam_linha.read()
                if ret_l and detectar_linha_embaixo(frame_l):
                    parar_motores()
                    print("[SAIDA] Linha preta detectada na camera de baixo")
                    print("[SAIDA] Alinhando na linha...")
                    # Gira pra esquerda pra passar da linha de proposito, depois
                    # varre devagar de volta pra direita ate ver o contorno de
                    # novo - assim o robo para alinhado EM CIMA da linha, nao
                    # so na primeira leitura (que pode ter pego ela de lado/
                    # angulo ruim, achando por acaso e nao alinhado de verdade).
                    mover(20, 20)
                    time.sleep(0.2)
                    guinada2('E', 20, 15)
                    ALINHAR_VEL = 8
                    ALINHAR_MAX_PASSOS = 60
                    for _ in range(ALINHAR_MAX_PASSOS):
                        checar_parada()
                        mover(ALINHAR_VEL, -ALINHAR_VEL)
                        time.sleep(0.05)
                        parar_motores()
                        ret_l2, frame_l2 = cam_linha.read()
                        if ret_l2 and detectar_linha_embaixo(frame_l2):
                            print("[SAIDA] Contorno da linha encontrado de novo, alinhado")
                            break
                    parar_motores()
                    return "linha_preta"

            if sonar_valido() and distancia_frente() < 10:
                parar_motores()
                time.sleep(0.2)
                if ret_r and time.time() >= ignorar_canto_ate:
                    crop = center_crop(frame_r, CROP_W, CROP_H)
                    cx, area, info = detectar_entrega(crop)
                    if cx is not None:
                        parar_motores()
                        cor_achada = info["cor"] if info else "?"
                        print(f"[SAIDA] Canto {cor_achada} detectado na camera de resgate")
                        print("vi um tringulozao verde ou vermelho")
                        mover(25, 25)
                        time.sleep(1.5)
                        mover(-20, -20)
                        time.sleep(0.2)
                        guinada2('D', 90, 15)
                        mover(15,15)
                        time.sleep(2.4)
                        guinada2('D', 43, 15)
                        ignorar_canto_ate = time.time() + TEMPO_IGNORAR_CANTO
                else:        
                    print("vi uma paredona")
                    print(distancia_frente())
                    parar_motores()
                    mover(-14, -14)
                    time.sleep(0.2)
                    guinada2('D', 90, 15)
                    time.sleep(0.3)
                    aguardar_leitura_fresca_sonar()

            # Antes de confiar no lateral pra decidir "corredor aberto", garante
            # que a leitura e de DEPOIS de qualquer giro/movimentacao desse loop
            # (canto verde/vermelho, guinada da paredona) - perto de canto em
            # diagonal um valor desatualizado pode ler falso "aberto".
            aguardar_leitura_fresca_sonar()
            if sonar_valido() and distancia_esquerda() > DIST_LATERAL_ABERTA_CM:
                print(distancia_esquerda())
                parar_motores()
                print("[SAIDA] Ultrassonico lateral aberto")
                print()
                return "sonar_lateral"

            time.sleep(0.02)
    finally:
        parar_motores()
        ALVO_COR = None


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

        forward(speed=20)
        '''time.sleep(TRIANGULO_APPROACH_STEP_TIME)
        parar_motores()'''
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
        turn_right(speed=15)
        '''time.sleep(TURN_PULSE)
        parar_motores()'''
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
    if not cap_linha.isOpened():
        print(f"Não consegui abrir a câmera de baixo em {CAMERA_INDEX_LINHA}, tentando fallback...")
        cap_linha.release()
        cap_linha = cv2.VideoCapture(CAMERA_LINHA_INDEX_FALLBACK, cv2.CAP_V4L2)

    if cap_linha.isOpened():
        cap_linha.set(cv2.CAP_PROP_FRAME_WIDTH, LINHA_W)
        cap_linha.set(cv2.CAP_PROP_FRAME_HEIGHT, LINHA_H)
        cap_linha.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    else:
        print("Câmera de baixo indisponível, acionar_saida() vai rodar sem detecção de linha")
        cap_linha.release()
        cap_linha = None

    return cap_linha

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
# Sobe a thread do botão antes do loop principal, igual no seguidor_debug2.py
threading.Thread(target=botao_thread, daemon=True).start()

# ===================== LOOP PRINCIPAL =====================
FALHAS_CAMERA_ANTES_DE_DESISTIR = 10  # falhas seguidas antes de voltar pro seguidor
falhas_camera_seguidas = 0

try:
    # O watchdog_seguranca ja esta rodando desde o boot (MPU, camera,
    # modelo carregando em paralelo) - nada disso chama checar_parada(),
    # entao sem esse reset aqui o watchdog dispara falso alarme antes do
    # loop principal sequer comecar. So passa a valer a partir daqui.
    _ultimo_heartbeat[0] = time.time()
    print("Iniciando busca por bolas...")
    while True:
        try:
            checar_parada()
            '''turn_right(speed=25)
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
                falhas_camera_seguidas += 1
                print(f"Erro na câmera ({falhas_camera_seguidas}/{FALHAS_CAMERA_ANTES_DE_DESISTIR})")
                if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_DESISTIR:
                    # Camera nao volta de jeito nenhum - em vez de so morrer
                    # sem chance de recuperacao (o que deixava o robo parado
                    # pra sempre no meio do resgate), volta pro seguidor
                    # igual o botao faria.
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
                # Modelo ainda carregando em background - continua girando/
                # procurando em vez de ficar parado esperando o import.
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
                mover_servo(PINO_PESADO, 30, velocidade=10)
                mover(-15,-15)
                time.sleep(0.1)
                parar_motores()
                mover(17,17)
                time.sleep(0.78)
                parar_motores()
                mover(-1,-1)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 0, velocidade=5)
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
                mover_servo(PINO_PESADO, 150, velocidade=5)
                mover(-25,-25)
                time.sleep(0.5)
                parar_motores()
                cor_alvo = CLASSE_COR_ENTREGA.get(cls_id)
                if cor_alvo is None:
                    print(f"Classe {cls_id} n�o mapeada em CLASSE_COR_ENTREGA, entregando em qualquer �rea")
                entregar_bola(cor_alvo)
                entregas_realizadas += 1

                if entregas_realizadas >= MAX_ENTREGAS:
                    print(f"[BOOT] {entregas_realizadas} entregas ja feitas, fazendo busca final antes da saida...")
                    if buscar_bola():
                        # Ainda tem bola na area - o limite de entregas nao
                        # impede pegar e entregar ela tambem. Nao vai pra
                        # saida: deixa o loop seguir, a proxima iteracao ja
                        # detecta essa bola e faz a captura/entrega normal.
                        print("[BUSCA FINAL] Achou bola sobrando, indo pegar e entregar mesmo assim")
                    else:
                        # SAIDA DESATIVADA (a pedido) - em vez de ir pra saida
                        # quando bater o limite de entregas, o robo continua
                        # buscando bolas. Nada foi apagado, so comentado.
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
                    # deixa o loop seguir e a próxima iteração já detecta e pega essa bola

            else:
                print(" Nenhuma bola — novo passo de giro")'''
            caso_saida = acionar_saida()
            print(f"[SAIDA] Caso identificado: {caso_saida}")
            parar_motores()
            cap.release()
            if cap_linha is not None:
                cap_linha.release()
            GPIO.gpiochip_close(h)
            os.execv(sys.executable, [sys.executable, "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py", "--auto"])
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
            # Qualquer erro inesperado (ex: lgpio.error por chip GPIO
            # sobrecarregado) - antes so RoboParado era tratado aqui, entao
            # qualquer outra excecao derrubava o processo inteiro sem
            # garantir que os motores pararam. Tenta parar e continua
            # tentando o resgate em vez de morrer travado.
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
    print("Finalizado com segurança.")
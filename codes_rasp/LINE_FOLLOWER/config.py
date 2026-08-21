"""
Hardware do seguidor de linha: estado de execucao/parada, GPIO, motores
(placa Yahboom), servos, botao, giroscopio (BNO055), sonar (serial do
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
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70  # converte a faixa antiga do L298N (-50..50) pra PWM Yahboom (-3600..3600)
PWM_FREIO_YAHBOOM = 700   # forca do pulso de freio reverso ao parar
TEMPO_FREIO_YAHBOOM = 0.08  # segundos do pulso de freio

VEL_MAX = 22#22
VEL_MIN = -VEL_MAX

ANGULO_INICIAL = 90

# Servo pesado como indicador visual de ligado/desligado: levantado (150)
# enquanto o robo esta ativo, abaixado (0) enquanto parado/esperando o botao.
SERVO_PESADO_ANGULO_ATIVO = 140
SERVO_PESADO_ANGULO_PARADO = 90
TEMPO_ASSENTAMENTO_SERVO_DIRETO = 0.4  # tempo pra segurar o pulso ate o servo assentar antes de cortar o PWM

def _detectar_porta_yahboom(sair_se_falhar=True):
    """Acha a porta serial da placa Yahboom pelo VID:PID (1a86:7522, CH340
    generico), nao por um caminho fisico fixo - assim funciona em
    qualquer porta USB, nao so a que foi usada quando o caminho foi
    medido. Mesmo padrao usado pra achar o Arduino mais abaixo
    (detectar_porta/aguardar_porta).

    De proposito SEM atalho por caminho fixo (by-path): esse caminho e
    atrelado ao SLOT fisico da porta USB, nao ao dispositivo - se a placa
    for trocada de porta, um by-path antigo passaria a apontar pra outro
    dispositivo qualquer (ex: o Arduino), que abre serial sem erro nenhum
    mas nao e a Yahboom do outro lado. O sintoma seria o robo "conectar"
    normal e os motores simplesmente nao se mexerem, sem nenhum erro no
    log - so o VID:PID garante que e o dispositivo certo."""
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
    inteiro se ela nao estiver ligada/conectada nesse instante do boot -
    antes, um serial.Serial() direto aqui crashava o servico inteiro e o
    systemd ficava reiniciando em loop toda vez que a placa nao estava
    plugada bem na hora do boot."""
    porta = _detectar_porta_yahboom(sair_se_falhar=False)
    tentativas = 0
    while porta is None:
        tentativas += 1
        print(f"[MOTOR] Placa Yahboom ainda nao apareceu, tentando de novo... ({tentativas})")
        time.sleep(intervalo)
        porta = _detectar_porta_yahboom(sair_se_falhar=False)
    # O dispositivo USB aparece na hora, mas o CH340/firmware da placa
    # costuma resetar quando a serial e reaberta - sem essa espera, os
    # primeiros comandos $pwm: saem enquanto ela ainda esta subindo e sao
    # ignorados (sem erro nenhum, so nao mexe). Mesmo padrao usado no
    # handshake_serial() do Arduino (ver mais abaixo).
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

    # mover_servo() sempre corta o PWM depois de assentar (evita o servo
    # "brigando" com o sinal o resto da execucao) - essa versao "direta"
    # deixava o pulso ligado pra sempre depois da chamada, e era isso que
    # fazia o servo pesado tremer continuamente enquanto o processo rodava
    # (o tremor sumia so quando o processo morria e o PWM parava de ser
    # gerado). Segura o pulso so o tempo de assentar e corta, igual mover_servo().
    time.sleep(TEMPO_ASSENTAMENTO_SERVO_DIRETO)
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error as e:
        print(f"Falha ao desligar PWM do servo (pino={pino}): {e}")


def sensor_e():
    return lgpio.gpio_read(h, TCRT)


def reiniciar_processo():
    """Reinicia o processo do zero via os.execv - equivalente a apertar
    Ctrl+C no terminal e rodar `python3 main.py` de novo. Um resume "por
    dentro" (so robo_ativo.set()) carrega estado velho de threads/sensores
    (referencia de travamento, offset do giro, buffer da camera, etc.);
    reiniciar o processo garante que tudo comeca limpo, sem herdar nada
    disso. Igual ao resto do codigo que usa execv (main.py <-> resgate.py),
    limpa motor/camera/GPIO antes de trocar a imagem do processo."""
    print("[BOTAO] Reiniciando processo do zero...")
    try:
        parar()
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao parar motores antes de reiniciar: {e}")
    try:
        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_PARADO)
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao abaixar servo antes de reiniciar: {e}")
    try:
        if cap is not None:
            cap.release()
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao liberar camera antes de reiniciar: {e}")
    try:
        if ser is not None:
            ser.close()
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao fechar serial do sonar antes de reiniciar: {e}")
    try:
        if motor_serial is not None:
            motor_serial.close()
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao fechar serial do motor antes de reiniciar: {e}")
    try:
        lgpio.gpiochip_close(h)
    except Exception as e:
        print(f"[BOTAO][ERRO] Falha ao fechar GPIO antes de reiniciar: {e}")
    os.execv(sys.executable, [sys.executable] + sys.argv)


def botao_thread():
    anterior = 0
    # --auto ja comeca com robo_ativo ligado (vindo do resgate) - nesse caso
    # a "primeira ativacao" ja aconteceu antes dessa thread nem existir.
    ja_foi_ativado = robo_ativo.is_set()
    while True:
        estado = lgpio.gpio_read(h, BUTTON)
        if estado == 1 and anterior == 0:
            time.sleep(0.03)
            if lgpio.gpio_read(h, BUTTON) == 1:
                # Espera soltar ANTES de agir. reiniciar_processo() faz
                # execv e nao volta mais pra essa thread - se agisse com o
                # dedo ainda no botao, o processo novo nasceria com
                # anterior=0 e leria esse MESMO nivel alto como um toque
                # novo, reiniciando em loop enquanto o botao nao fosse
                # solto de verdade. Por isso o release tem que vir antes.
                while lgpio.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
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
                elif not ja_foi_ativado:
                    # Primeira ativacao desse processo (boot fresco, nunca
                    # rodou ainda) - so liga normal, sem reiniciar. Reiniciar
                    # aqui nao teria nenhum estado velho pra limpar e so
                    # ficava reiniciando em loop sem nunca chegar a andar.
                    print("[BOTAO] INICIANDO...")
                    ja_foi_ativado = True
                    robo_ativo.set()
                    try:
                        mover_servo_direto(PINO_PESADO, SERVO_PESADO_ANGULO_ATIVO)
                    except Exception as e:
                        print(f"[BOTAO][ERRO] Falha ao levantar servo pesado: {e}")
                else:
                    # Ja tinha rodado antes e foi parado - agora sim reinicia
                    # o processo inteiro do zero, igual parar no terminal e
                    # rodar de novo. Ver reiniciar_processo().
                    reiniciar_processo()
        anterior = estado
        time.sleep(0.02)


# =========================================================================
# BNO055 DIRETO NA RASP (modo IMUPLUS: fusao interna de acel+giro, SEM
# magnetometro). O modo de fusao NDOF usa o magnetometro pra achar o norte -
# perto dos motores/ponte H a interferencia magnetica fazia o heading
# "pular"/resetar sozinho quando o robo mexia, entao o magnetometro continua
# fora. Mas o eixo Z (heading/yaw, usado pelas guinadas) agora vem PRONTO da
# fusao interna do chip em vez de integrar o giro cru em software - o chip
# faz compensacao de deriva/temperatura melhor do que a integracao manual.
# X e Y continuam exatamente como antes (giro cru integrado em software e
# acelerometro cru, respectivamente) - nada mudou neles.
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
MODO_ACCGYRO    = 0x05  # acel + giro crus, sem magnetometro e sem fusao interna (nao usado mais - so referencia)
MODO_IMUPLUS    = 0x08  # fusao 6DOF (acel+giro) sem magnetometro - da heading/roll/pitch prontos
CHIP_ID_ESPERADO = 0xA0
LIMITE_DPS_PLAUSIVEL = 2000.0  # fundo de escala do giroscopio - acima disso e leitura corrompida
bus = smbus2.SMBus(1)

ang_gyro          = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_offset       = {"X": 0.0, "Y": 0.0, "Z": 0.0}
gyro_ultimo_tempo = time.time()

_angulo_y_filtrado = None
_heading_offset = 0.0  # heading (fusao) no momento do ultimo resetar_giroscopio() - vira o "zero" da guinada
_ultimo_angulo_z = [0]  # cache do ultimo angulo_z() valido, pra nao quebrar guinada numa falha I2C isolada


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
            # dict) - agora angulo_z() faz leitura I2C do heading fundido a
            # partir da thread principal (guinada), concorrente com essa
            # thread. Sem o lock aqui, as duas podiam intercalar leituras
            # multi-byte no mesmo barramento e corromper o valor lido.
            with _gyro_lock:
                gx = (_bno_read_word(GYR_DATA_X)     - gyro_offset["X"]) / 16.0
                gy = (_bno_read_word(GYR_DATA_X + 2) - gyro_offset["Y"]) / 16.0
                gz = (_bno_read_word(GYR_DATA_X + 4) - gyro_offset["Z"]) / 16.0
            # Fundo de escala do giroscopio e 2000 dps - qualquer valor acima
            # disso e fisicamente impossivel, e leitura corrompida (ruido
            # eletrico dos motores no barramento I2C, mais comum durante
            # guinada/desvio com motor puxando corrente). Integrar isso
            # jogava o angulo acumulado centenas de graus com 1 amostra so.
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
    """Nome mantido por compatibilidade com quem ja importa/chama
    config.iniciar_mpu (main.py) - por dentro agora inicializa o BNO055."""
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


# ==========================================================
# ANGULO Y ESTAVEL (RAMPA/GANGORRA)
# Usa somente o acelerometro + filtro exponencial
# ==========================================================
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
    # A porta da placa Yahboom ja foi achada por VID:PID (ver
    # _detectar_porta_yahboom) - nunca pode ser tambem a porta do Arduino,
    # senao os dois ficam abrindo o MESMO device concorrentemente (visto ao
    # vivo: PWM do motor falhando com I/O error e o sonar nunca recebendo
    # dado valido, preso reconectando). So acontece quando o caminho fixo e
    # a busca por palavra-chave FTDI abaixo falham e cai no fallback
    # generico - mas vale excluir nos dois pra garantir.
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

"""
Teste de bancada dos motores (placa Yahboom via serial) + guinada por
BNO055. Script isolado, nao importa config.py/resgate2.py - so abre serial
da placa de motor e I2C do giroscopio, sem GPIO/camera/YOLO.

Protocolo e mapeamento de canais copiados de resgate2.py (aplicar_motor/
mover/parar_motores/guinada2) pra bater exatamente com o robo de verdade.
"""

import time

import serial
import smbus2

# ===================== CONFIG =====================
YAHBOOM_PORTA = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
YAHBOOM_BAUDRATE = 115200
ESCALA_PWM_YAHBOOM = 70  # converte -VEL_MAX..VEL_MAX pra PWM Yahboom (-3600..3600)
PWM_FREIO_YAHBOOM = 700
TEMPO_FREIO_YAHBOOM = 0.08

VEL_MAX = 20
VEL_MIN = -VEL_MAX
VELOCIDADE_PADRAO = 15

BNO_ADDR = 0x29
REG_CHIP_ID = 0x00
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL = 0x3B
EUL_DATA_HEADING = 0x1A
MODO_CONFIG = 0x00
MODO_IMUPLUS = 0x08
CHIP_ID_ESPERADO = 0xA0

GRAUS_PADRAO = 90

motor_serial = serial.Serial(YAHBOOM_PORTA, YAHBOOM_BAUDRATE, timeout=0.5)
bus = smbus2.SMBus(1)

giroscopio_disponivel = False
_heading_offset = 0.0


# ===================== MOTORES =====================
def _enviar_pwm_yahboom(m1, m2, m3, m4):
    m1, m2, m3, m4 = (max(-3600, min(3600, int(v))) for v in (m1, m2, m3, m4))
    try:
        motor_serial.write(f"$pwm:{m1},{m2},{m3},{m4}#".encode())
    except serial.SerialException as e:
        print(f"[MOTOR][ERRO] Falha ao mandar PWM pra placa Yahboom: {e}")


_ultima_vel = (0, 0)


def aplicar_motor(velL, velR):
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
        freio_vel = PWM_FREIO_YAHBOOM / ESCALA_PWM_YAHBOOM
        freio_esq = -freio_vel if velL > 0 else (freio_vel if velL < 0 else 0)
        freio_dir = -freio_vel if velR > 0 else (freio_vel if velR < 0 else 0)
        aplicar_motor(freio_esq, freio_dir)
        time.sleep(TEMPO_FREIO_YAHBOOM)
    mover(0, 0)


def forward(speed=VELOCIDADE_PADRAO):
    mover(speed, speed)


def backward(speed=VELOCIDADE_PADRAO):
    mover(-speed, -speed)


def turn_left(speed=VELOCIDADE_PADRAO):
    mover(-speed, speed)


def turn_right(speed=VELOCIDADE_PADRAO):
    mover(speed, -speed)


def testar_canal(nome, m1, m2, m3, m4, velocidade, duracao=1.0):
    """Manda PWM bruto so num canal por vez, pra achar fiacao/motor
    trocado sem depender do mapeamento esquerda/direita."""
    pwm = velocidade * ESCALA_PWM_YAHBOOM
    print(f"[CANAL] {nome} por {duracao:.1f}s...")
    _enviar_pwm_yahboom(m1 * pwm, m2 * pwm, m3 * pwm, m4 * pwm)
    time.sleep(duracao)
    _enviar_pwm_yahboom(0, 0, 0, 0)


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
        except OSError:
            if tentativa == tentativas - 1:
                raise
            time.sleep(0.001)


def _normalizar(ang):
    while ang > 180:
        ang -= 360
    while ang < -180:
        ang += 360
    return ang


def _ler_heading_bruto():
    return _bno_read_word(EUL_DATA_HEADING) / 16.0


def resetar_giroscopio():
    global _heading_offset
    try:
        _heading_offset = _ler_heading_bruto()
    except Exception as e:
        print(f"[BNO055] Falha ao ler heading pro reset ({e})")


def angulo_z():
    try:
        atual = _ler_heading_bruto()
        return int(_normalizar(atual - _heading_offset))
    except Exception as e:
        print(f"[BNO055] Falha ao ler heading ({e})")
        return 0


def iniciar_bno055():
    global giroscopio_disponivel
    try:
        chip_id = bus.read_byte_data(BNO_ADDR, REG_CHIP_ID)
        if chip_id != CHIP_ID_ESPERADO:
            raise RuntimeError(f"CHIP_ID inesperado: 0x{chip_id:02X}")

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
        _bno_write(REG_PWR_MODE, 0x00)
        time.sleep(0.02)
        _bno_write(REG_UNIT_SEL, 0x00)
        time.sleep(0.02)
        _bno_write(REG_OPR_MODE, MODO_IMUPLUS)
        time.sleep(0.7)

        resetar_giroscopio()
        giroscopio_disponivel = True
        print("[BNO055] Pronto")
    except Exception as e:
        giroscopio_disponivel = False
        print(f"[BNO055][AVISO] Giroscopio indisponivel, guinada por tempo fixo vai ser usada no lugar: {e}")


def guinada2(LADO, GRAUS, VELOCIDADE=VELOCIDADE_PADRAO):
    """Mesma logica de resgate2.py: gira ate o heading fundido bater GRAUS,
    com pulso de freio curto ao chegar. Sem giroscopio, cai pra guinada por
    tempo fixo (bem aproximado, so pra nao travar o teste)."""
    if not giroscopio_disponivel:
        _guinada_tempo_fixo(LADO, GRAUS, VELOCIDADE)
        return

    resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    try:
        if LADO == 'E':
            while True:
                mover(-VELOCIDADE, VELOCIDADE)
                if angulo_z() <= -Graus:
                    mover(2, -2)
                    time.sleep(0.09)
                    break
                time.sleep(0.005)
        else:
            while True:
                mover(VELOCIDADE, -VELOCIDADE)
                if angulo_z() >= Graus:
                    mover(-2, 2)
                    time.sleep(0.09)
                    break
                time.sleep(0.005)
    finally:
        parar_motores()
    resetar_giroscopio()


def _guinada_tempo_fixo(LADO, GRAUS, VELOCIDADE):
    # Aproximacao grosseira (sem feedback), so de emergencia se o BNO055
    # nao respondeu no boot. ~0.01s por grau, calibrar na unha se usar.
    duracao = abs(GRAUS) * 0.01
    if LADO == 'E':
        turn_left(VELOCIDADE)
    else:
        turn_right(VELOCIDADE)
    time.sleep(duracao)
    parar_motores()


# ===================== MENU INTERATIVO =====================
AJUDA = """
===================== TESTE DE MOTORES =====================
  w / s          - frente / re (pulso curto)
  a / d          - guinada esquerda / direita (giroscopio, GRAUS_PADRAO)
  q / e          - turn_left / turn_right no lugar (sem giroscopio, pulso curto)
  1 / 2 / 3 / 4  - testa só o canal M1/M2/M3/M4 (achar motor/fiação trocada)
  g<graus>       - guinada direita pelo giroscopio (ex: g45, g-90 vai p/ esquerda)
  v<vel>         - muda velocidade padrao (ex: v20, faixa 1-20)
  t<seg>         - muda duracao do pulso w/s/q/e em segundos (ex: t0.5)
  z              - angulo_z() atual (giroscopio)
  r              - reseta giroscopio (zera angulo_z)
  x              - PARA motores na hora
  h              - mostra essa ajuda
  ESC / Ctrl+C   - sai
===============================================================
"""

velocidade_atual = VELOCIDADE_PADRAO
duracao_pulso = 0.6


def _get_key():
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    antigo = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, antigo)
    return ch


def main():
    global velocidade_atual, duracao_pulso

    print(AJUDA)
    iniciar_bno055()
    print(f"[TESTE] velocidade={velocidade_atual} duracao_pulso={duracao_pulso}s")

    try:
        while True:
            tecla = _get_key()

            if tecla == '\x1b' or tecla == '\x03':  # ESC ou Ctrl+C
                break

            elif tecla == 'h':
                mover(0, 20)
                time.sleep(0.60)
                parar_motores()
                mover(20, 0)
                time.sleep(0.60)
                parar_motores()
                mover(0, -20)
                time.sleep(0.60)
                parar_motores()
                mover(-20, 0)
                time.sleep(0.60)
                parar_motores()


            elif tecla == 'w':
                print("[MOTOR] forward")
                forward(velocidade_atual)
                time.sleep(duracao_pulso)
                parar_motores()

            elif tecla == 's':
                print("[MOTOR] backward")
                backward(velocidade_atual)
                time.sleep(duracao_pulso)
                parar_motores()

            elif tecla == 'q':
                print("[MOTOR] turn_left")
                turn_left(velocidade_atual)
                time.sleep(duracao_pulso)
                parar_motores()

            elif tecla == 'e':
                print("[MOTOR] turn_right")
                turn_right(velocidade_atual)
                time.sleep(duracao_pulso)
                parar_motores()

            elif tecla == 'a':
                print(f"[GUINADA] esquerda {GRAUS_PADRAO} graus")
                guinada2('E', GRAUS_PADRAO, velocidade_atual)
                print(f"[GUINADA] concluida, angulo_z()={angulo_z()}")

            elif tecla == 'd':
                print(f"[GUINADA] direita {GRAUS_PADRAO} graus")
                guinada2('D', GRAUS_PADRAO, velocidade_atual)
                print(f"[GUINADA] concluida, angulo_z()={angulo_z()}")

            elif tecla in ('1', '2', '3', '4'):
                canais = {
                    '1': (1, 0, 0, 0),
                    '2': (0, 1, 0, 0),
                    '3': (0, 0, 1, 0),
                    '4': (0, 0, 0, 1),
                }
                testar_canal(f"M{tecla}", *canais[tecla], velocidade_atual, duracao_pulso)

            elif tecla == 'z':
                if giroscopio_disponivel:
                    print(f"[GIRO] angulo_z()={angulo_z()}")
                else:
                    print("[GIRO] giroscopio indisponivel")

            elif tecla == 'r':
                resetar_giroscopio()
                print("[GIRO] resetado")

            elif tecla == 'x':
                print("[MOTOR] parada de emergencia")
                parar_motores()

            elif tecla in ('g', 'v', 't'):
                # comandos com numero (g45, v15, t0.5) - le o resto da linha
                print(f"{tecla}", end="", flush=True)
                resto = ""
                while True:
                    c = _get_key()
                    if c in ('\r', '\n'):
                        print()
                        break
                    if c == '\x7f':  # backspace
                        resto = resto[:-1]
                        continue
                    resto += c
                    print(c, end="", flush=True)
                try:
                    valor = float(resto)
                except ValueError:
                    print(f"[TESTE] valor invalido: {resto!r}")
                    continue

                if tecla == 'g':
                    lado = 'D' if valor >= 0 else 'E'
                    print(f"[GUINADA] {lado} {abs(valor):.0f} graus")
                    guinada2(lado, valor, velocidade_atual)
                    print(f"[GUINADA] concluida, angulo_z()={angulo_z()}")
                elif tecla == 'v':
                    velocidade_atual = max(1, min(VEL_MAX, int(valor)))
                    print(f"[TESTE] velocidade_atual={velocidade_atual}")
                elif tecla == 't':
                    duracao_pulso = max(0.05, valor)
                    print(f"[TESTE] duracao_pulso={duracao_pulso}s")

    except KeyboardInterrupt:
        pass
    finally:
        parar_motores()
        motor_serial.close()
        print("\n[TESTE] Encerrado")


if __name__ == "__main__":
    main()

"""
Teste de bancada do BNO055 (IMU com fusao Bosch) via I2C direto (smbus2),
sem lib Adafruit - a mesma abordagem usada pro MPU6050 em LINE_FOLLOWER/config.py.
Sensor encontrado em 0x29 no barramento 1 (i2cdetect -y 1).
"""

import time

import smbus2

# =========================================================================
# CONFIG
# =========================================================================
I2C_BUS = 1
BNO055_ADDR = 0x29  # outro endereco possivel e 0x28 (pino ADR em GND)

# Registradores (datasheet BNO055, pagina 0)
REG_CHIP_ID     = 0x00
REG_CALIB_STAT  = 0x35
REG_TEMP        = 0x34
REG_OPR_MODE    = 0x3D
REG_PWR_MODE    = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_UNIT_SEL    = 0x3B
REG_EUL_HEADING = 0x1A  # heading, roll, pitch - 2 bytes cada, nessa ordem
REG_QUA_DATA_W  = 0x20  # w, x, y, z - 2 bytes cada
REG_LIA_DATA_X  = 0x28  # aceleracao linear (sem gravidade) - x, y, z
REG_GYR_DATA_X  = 0x14  # giroscopio - x, y, z

CHIP_ID_ESPERADO = 0xA0

MODO_CONFIG = 0x00
MODO_NDOF   = 0x0C  # fusao completa (acel + giro + magnetometro) - aponta pro norte de verdade
MODO_IMU    = 0x08  # fusao sem magnetometro (so acel + giro) - so da angulo RELATIVO,
                     # mas imune a interferencia magnetica dos motores/ponte H perto do
                     # sensor no robo (era o que fazia o heading "zerar"/pular sozinho
                     # no NDOF quando o robo mexia/os motores ligavam)

ESCALA_EULER = 16.0      # 1 grau = 16 LSB
ESCALA_QUATERNION = 16384.0  # 1 unidade = 2^14 LSB
ESCALA_ACEL = 100.0      # 1 m/s^2 = 100 LSB (unidade padrao apos UNIT_SEL)
ESCALA_GIRO = 16.0       # 1 grau/s = 16 LSB

bus = smbus2.SMBus(I2C_BUS)


# =========================================================================
# LEITURA/ESCRITA BAIXO NIVEL
# =========================================================================
def _write(reg, val):
    bus.write_byte_data(BNO055_ADDR, reg, val)


def _read_byte(reg):
    return bus.read_byte_data(BNO055_ADDR, reg)


def _read_word_signed(reg):
    """Le 2 bytes (LSB, MSB) e devolve com sinal (complemento de 2)."""
    dados = bus.read_i2c_block_data(BNO055_ADDR, reg, 2)
    valor = (dados[1] << 8) | dados[0]
    if valor >= 0x8000:
        valor -= 65536
    return valor


def _read_vetor3(reg_base, escala):
    """Le 3 eixos consecutivos (x, y, z) a partir de reg_base, ja escalados."""
    x = _read_word_signed(reg_base)
    y = _read_word_signed(reg_base + 2)
    z = _read_word_signed(reg_base + 4)
    return (x / escala, y / escala, z / escala)


# =========================================================================
# INICIALIZACAO
# =========================================================================
def iniciar_bno055(modo=MODO_IMU):
    chip_id = _read_byte(REG_CHIP_ID)
    if chip_id != CHIP_ID_ESPERADO:
        raise RuntimeError(
            f"[BNO055] CHIP_ID inesperado: 0x{chip_id:02X} (esperado 0x{CHIP_ID_ESPERADO:02X})"
        )

    _write(REG_OPR_MODE, MODO_CONFIG)
    time.sleep(0.02)

    _write(REG_SYS_TRIGGER, 0x20)  # reset
    time.sleep(0.7)

    # espera o sensor voltar a responder apos o reset
    for _ in range(20):
        try:
            if _read_byte(REG_CHIP_ID) == CHIP_ID_ESPERADO:
                break
        except OSError:
            pass
        time.sleep(0.05)

    _write(REG_PWR_MODE, 0x00)  # normal
    time.sleep(0.02)
    _write(REG_UNIT_SEL, 0x00)  # graus, m/s^2, graus/s (unidades default Windows)
    time.sleep(0.02)
    _write(REG_SYS_TRIGGER, 0x00)  # cristal interno
    time.sleep(0.02)

    _write(REG_OPR_MODE, modo)
    time.sleep(0.7)  # troca de modo demora ate 19ms, mas fusao pede mais pra estabilizar

    nome_modo = "IMU (sem magnetometro)" if modo == MODO_IMU else "NDOF (com magnetometro)"
    print(f"[BNO055] Pronto (CHIP_ID=0x{chip_id:02X}, modo {nome_modo})")


# =========================================================================
# LEITURAS
# =========================================================================
def ler_calibracao():
    """Cada campo vai de 0 (nao calibrado) a 3 (totalmente calibrado)."""
    status = _read_byte(REG_CALIB_STAT)
    sys_cal  = (status >> 6) & 0x03
    giro_cal = (status >> 4) & 0x03
    acel_cal = (status >> 2) & 0x03
    mag_cal  = status & 0x03
    return {"sistema": sys_cal, "giroscopio": giro_cal, "acelerometro": acel_cal, "magnetometro": mag_cal}


def ler_temperatura():
    return _read_byte(REG_TEMP)  # graus Celsius, ja em inteiro


def ler_euler():
    """(heading, roll, pitch) em graus. Heading 0-360, roll/pitch -180..180."""
    heading = _read_word_signed(REG_EUL_HEADING) / ESCALA_EULER
    roll    = _read_word_signed(REG_EUL_HEADING + 2) / ESCALA_EULER
    pitch   = _read_word_signed(REG_EUL_HEADING + 4) / ESCALA_EULER
    return (heading, roll, pitch)


def ler_quaternion():
    w = _read_word_signed(REG_QUA_DATA_W) / ESCALA_QUATERNION
    x = _read_word_signed(REG_QUA_DATA_W + 2) / ESCALA_QUATERNION
    y = _read_word_signed(REG_QUA_DATA_W + 4) / ESCALA_QUATERNION
    z = _read_word_signed(REG_QUA_DATA_W + 6) / ESCALA_QUATERNION
    return (w, x, y, z)


def ler_aceleracao_linear():
    """Aceleracao ja sem a componente da gravidade, em m/s^2."""
    return _read_vetor3(REG_LIA_DATA_X, ESCALA_ACEL)


def ler_giroscopio():
    """Velocidade angular em graus/s."""
    return _read_vetor3(REG_GYR_DATA_X, ESCALA_GIRO)


# =========================================================================
# ZERO DE REFERENCIA / GUINADA RELATIVA
# Mesmo padrao do MPU6050 em LINE_FOLLOWER/config.py: zerar() marca o
# heading atual como referencia, angulo_guinada() devolve o quanto girou
# desde entao (-180..180). Zera antes de cada guinada (igual
# config.resetar_giroscopio() e chamado antes de cada guinada2/guinada_pid).
# =========================================================================
_heading_referencia = 0.0


def zerar_guinada():
    global _heading_referencia
    _heading_referencia = ler_euler()[0]


def angulo_guinada():
    """Graus girados desde o ultimo zerar_guinada(), normalizado -180..180
    (positivo = sentido horario, que e como o BNO055 mede heading)."""
    heading_atual = ler_euler()[0]
    diff = heading_atual - _heading_referencia
    return (diff + 180) % 360 - 180


# =========================================================================
# TESTE
# =========================================================================
if __name__ == "__main__":
    iniciar_bno055()
    zerar_guinada()
    print("[BNO055] Guinada zerada - gire o sensor pra ver o angulo relativo mudar")
    try:
        while True:
            cal = ler_calibracao()
            heading, roll, pitch = ler_euler()
            guinada = angulo_guinada()
            gx, gy, gz = ler_giroscopio()
            ax, ay, az = ler_aceleracao_linear()
            temp = ler_temperatura()

            print(
                f"Guinada={guinada:6.1f} (heading bruto={heading:6.1f}) R={roll:6.1f} P={pitch:6.1f} | "
                f"Giro X={gx:6.1f} Y={gy:6.1f} Z={gz:6.1f} | "
                f"AcelLin X={ax:5.2f} Y={ay:5.2f} Z={az:5.2f} | "
                f"Temp={temp}C | "
                f"Cal sys={cal['sistema']} gy={cal['giroscopio']} "
                f"ac={cal['acelerometro']} mag={cal['magnetometro']}"
            )
            time.sleep(0.0001)
    except KeyboardInterrupt:
        print("\n[BNO055] Teste encerrado")

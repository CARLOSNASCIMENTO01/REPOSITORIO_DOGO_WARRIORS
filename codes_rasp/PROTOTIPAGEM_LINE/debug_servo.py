import lgpio
import time

# Pinos
PINO_SG90 = 5
#PINO_PESADO = 20

ANGULO_INICIAL = 90

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PINO_SG90)
#lgpio.gpio_claim_output(h, PINO_PESADO)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL
}

DELAY_BASE = 0.03






def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass

def angulo(ang):
    ang = max(0, min(180, ang))
    pulso = int(500 + (2000 * ang / 180))
    lgpio.tx_servo(h, PINO_SG90, pulso)
    time.sleep(0.1)
    lgpio.tx_servo(h, PINO_SG90, 0)
try:
    angulo(180)
    time.sleep(5)
    # parar_servo(PINO_PESADO)

finally:
    lgpio.gpiochip_close(h)
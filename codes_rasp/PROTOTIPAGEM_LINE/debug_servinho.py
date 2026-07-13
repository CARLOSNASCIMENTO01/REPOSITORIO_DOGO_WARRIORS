import lgpio
import time

PINO = 20

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PINO)

def angulo(ang):
    ang = max(0, min(180, ang))

    # 500 us -> 0�
    # 2500 us -> 180�
    pulso = int(500 + (2000 * ang / 180))

    print("Pulso:", pulso)

    lgpio.tx_servo(h, PINO, pulso)

try:
    angulo(0)
    time.sleep(2)

    lgpio.tx_servo(h, PINO, 0)

finally:
    lgpio.gpiochip_close(h)
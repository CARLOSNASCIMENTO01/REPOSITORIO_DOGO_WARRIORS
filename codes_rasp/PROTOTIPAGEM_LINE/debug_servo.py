import lgpio
import time

# Pinos
PINO_SG90 = 20
PINO_PESADO = 16

ANGULO_INICIAL = 90

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL,
    PINO_PESADO: ANGULO_INICIAL,
}

DELAY_BASE = 0.03


def mover_servo(pino, angulo, velocidade=100, pulso_min=500, pulso_max=2500):
    """Move suave, passo a passo (usa a velocidade pra controlar o delay)."""
    angulo = max(0, min(180, angulo))
    velocidade = max(1, min(100, velocidade))

    angulo_atual = _angulo_atual.get(pino, ANGULO_INICIAL)
    delay_por_passo = DELAY_BASE * (100 - velocidade) / 100.0

    passo = 1 if angulo >= angulo_atual else -1

    for ang in range(int(angulo_atual), int(angulo) + passo, passo):
        largura_pulso = pulso_min + (ang / 180.0) * (pulso_max - pulso_min)
        lgpio.tx_servo(h, pino, int(largura_pulso))
        if delay_por_passo > 0:
            time.sleep(delay_por_passo)

    _angulo_atual[pino] = angulo
    time.sleep(0.15)


def mover_servo_direto(pino, angulo, pulso_min=500, pulso_max=2500):
    """Manda o pulso direto pro ângulo, sem passar por passos intermediários."""
    angulo = max(0, min(180, angulo))
    largura_pulso = pulso_min + (angulo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    _angulo_atual[pino] = angulo


def oscilar_servo_rapido(pino, angulo_a, angulo_b, repeticoes=1, delay=0.15):
    """
    Alterna o servo rapidamente entre angulo_a e angulo_b, sem suavizar
    (pulso direto em cada ponta). 'delay' é o tempo de espera em cada
    ponta pra dar tempo do servo fisicamente chegar lá antes de inverter.
    """
    for i in range(repeticoes):
        mover_servo_direto(pino, angulo_a)
        time.sleep(delay)
        mover_servo_direto(pino, angulo_b)
        time.sleep(delay)


def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass


try:
    print("Pesado -> 90 graus (vel 20%)")
    mover_servo(PINO_PESADO, 150, velocidade=0)
    print("Pesado -> 90 graus (vel 20%)")
    mover_servo_direto(PINO_SG90, 0)
    mover_servo(PINO_PESADO, 0, velocidade=5)
    print("Pesado -> 90 graus (vel 20%)")
    mover_servo_direto(PINO_SG90, 0)
    print("Pesado -> 90 graus (vel 20%)")
    mover_servo(PINO_PESADO, 90, velocidade=5)
    print("Pesado -> 90 graus (vel 20%)")
    mover_servo_direto(PINO_SG90, 160)
    time.sleep(2.0)
    # Oscilação rápida, sem suavidade, entre 70 e 90 graus
    mover_servo_direto(PINO_SG90, 160)
    oscilar_servo_rapido(PINO_PESADO, angulo_a=30, angulo_b=120, repeticoes=20, delay=0.1)

    parar_servo(PINO_SG90)
    parar_servo(PINO_PESADO)

finally:
    lgpio.gpiochip_close(h)
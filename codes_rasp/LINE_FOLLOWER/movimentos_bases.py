
import time

import config


def guinada2(LADO, GRAUS, VELOCIDADE, vel_m2=None):
    """vel_m2: se passar, a M2 (traseira esquerda) gira nessa velocidade
    (modulo; o sentido segue o lado esquerdo) em vez de VELOCIDADE."""
    config.resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    if LADO == 'E':
        while True:
            config.checar_parada()
            config.mover(-VELOCIDADE, VELOCIDADE, vel_m2=None if vel_m2 is None else -abs(vel_m2))
            if config.angulo_z() <= -Graus:
                config.mover(2, -2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    else:
        while True:
            config.checar_parada()
            config.mover(VELOCIDADE, -VELOCIDADE, vel_m2=None if vel_m2 is None else abs(vel_m2))
            if config.angulo_z() >= Graus:
                config.mover(-2, 2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    config.resetar_giroscopio()


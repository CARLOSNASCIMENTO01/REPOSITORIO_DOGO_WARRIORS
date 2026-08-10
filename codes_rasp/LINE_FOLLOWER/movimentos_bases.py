"""
Guinadas (giro no proprio eixo controlado pelo giroscopio). guinada2 e a
guinada "classica" usada em quase todo canto (verde, desvio); guinada_pid
e a versao fechada em PID sobre o angulo, usada na saida do resgate.
"""

import time

import numpy as np

import config


def guinada2(LADO, GRAUS, VELOCIDADE):
    config.resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    if LADO == 'E':
        while True:
            config.checar_parada()
            config.mover(-VELOCIDADE, VELOCIDADE)
            if config.angulo_z() <= -Graus:
                config.mover(2, -2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    else:
        while True:
            config.checar_parada()
            config.mover(VELOCIDADE, -VELOCIDADE)
            if config.angulo_z() >= Graus:
                config.mover(-2, 2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    # Zera de novo ao terminar - sem isso o angulo_z() ficava preso no valor
    # da guinada (ex: 90) durante todo o segmento reto seguinte, ate a
    # PROXIMA guinada zerar - confundia debug e deixava lixo acumulado
    # exposto pra qualquer outro codigo que va ler angulo_z() nesse meio tempo.
    config.resetar_giroscopio()


def guinada_pid(LADO, GRAUS, VEL_MAX_GIRO=100, VEL_MIN_GIRO=29, ZONA_CHEGADA=10):
    config.parar()
    Kp_g, Ki_g, Kd_g = 0.09, 0.002, 0.001
    config.resetar_giroscopio()
    time.sleep(0.03)

    alvo = abs(GRAUS)
    integral = 0.0
    erro_ant = alvo
    t_ant = time.time()

    while True:
        config.checar_parada()
        atual = abs(config.angulo_z())
        erro = alvo - atual
        agora = time.time()
        dt = max(agora - t_ant, 0.001)

        if erro <= ZONA_CHEGADA:
            if LADO == 'D':
                config.mover(-15, 15)
            else:
                config.mover(15, -15)
            time.sleep(0.09)
            config.parar()
            config.resetar_giroscopio()
            break

        integral += erro * dt
        integral = np.clip(integral, -20, 20)
        derivada = (erro - erro_ant) / dt
        derivada = np.clip(derivada, -300, 300)

        ajuste = Kp_g * erro + Ki_g * integral - Kd_g * derivada
        ajuste = np.clip(ajuste, 0, VEL_MAX_GIRO - VEL_MIN_GIRO)
        vel = int(VEL_MIN_GIRO + ajuste)

        if LADO == 'D':
            config.mover(vel, -vel)
        else:
            config.mover(-vel, vel)

        erro_ant = erro
        t_ant = agora
        time.sleep(0.005)

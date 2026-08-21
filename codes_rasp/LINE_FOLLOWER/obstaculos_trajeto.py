"""
Desvio de obstaculo via sonar frontal - contorna indo pra direita, andando
ao longo da parede pelo sonar esquerdo, e volta pra linha.
"""

import time

import cv2

import config
import movimentos_bases
import seguimento_de_linha

LIMIAR_OBSTACULO_SURPRESA_CM = 15  # desafio surpresa: espera esse obstaculo sumir antes de sair


def esperar_obstaculo_frontal_sumir(limiar_cm=LIMIAR_OBSTACULO_SURPRESA_CM, intervalo=0.05):
    """
    Desafio surpresa: se tiver um obstaculo bem na frente (<= limiar_cm) na
    largada, o robo fica parado esperando ele sumir (o juiz tira o
    obstaculo) antes de comecar a andar. Se nao tiver obstaculo nenhum,
    passa direto sem esperar nada.
    """
    if config.distancia_frente() > limiar_cm:
        return
    print(f"[DESAFIO] Obstaculo a <= {limiar_cm}cm na frente, esperando sumir pra sair...")
    config.parar()
    while config.distancia_frente() <= limiar_cm:
        config.checar_parada()
        time.sleep(intervalo)
    print("[DESAFIO] Obstaculo sumiu, seguindo.")


def desvio():
    # Sonar da frente aponta reto: numa rampa/gangorra inclinada o feixe
    # pode bater no proprio chao e ler "parede" que nao existe. So confia
    # nele com o robo em piso plano (mesma faixa que controle() usa pra
    # saber que nao esta na gangorra).
    if not (-5 < config.angulo_y() < 3):
        return
    if config.distancia_frente() <= 5:
        # O Arduino alterna F/E a cada 60ms, entao cada sonar so atualiza a
        # cada ~120ms. Um sleep menor que isso reconfere o MESMO valor em
        # cache, nao uma leitura nova - por isso 0.15s aqui, nao 0.09s.
        config.mover(-2, -2)
        time.sleep(1.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        if config.distancia_frente() <= 5:
            config.mover(18, 18)
            config.mover(-18, -18)
            time.sleep(0.2)
            config.mover(-2, -2)
            time.sleep(0.1)
            """config.mover(15, -15)
            time.sleep(1.4)
            config.mover(-2, 2)
            time.sleep(0.1)"""
            movimentos_bases.guinada2("D", 96, 14)
            time.sleep(0.2)

            while True:
                config.checar_parada()
                config.mover(18, 18)
                if config.distancia_esquerda() < 15:
                    config.mover(-2, -2)
                    time.sleep(0.8)
                    break
                time.sleep(0.005)

            while True:
                config.checar_parada()
                config.mover(18, 18)
                if config.distancia_esquerda() > 20:
                    config.mover(15, 15)
                    time.sleep(0.30)
                    config.mover(-2, -2)
                    time.sleep(0.2)
                    break
                time.sleep(0.005)
            """config.mover(-15, 15)
            time.sleep(1.3)
            config.mover(-2, 2)
            time.sleep(0.1)"""
            #time.sleep(0.2)
            movimentos_bases.guinada2("E", 92, 15)
            while True:
                config.checar_parada()
                config.mover(18, 18)
                if config.distancia_esquerda() < 20:
                    config.mover(15, 15)
                    time.sleep(0.2)
                    config.mover(-2, -2)
                    time.sleep(0.3)
                    break
                time.sleep(0.005)

            while True:
                config.checar_parada()
                config.mover(18, 18)
                if config.distancia_esquerda() > 17:
                    config.mover(20, 20)
                    time.sleep(0.6)
                    config.mover(-2, -2)
                    time.sleep(0.3)
                    break
                time.sleep(0.005)

            movimentos_bases.guinada2("E", 90, 15)
            TOLERANCIA_CENTRO_LINHA = 30  # pixels de folga pro cx contar como "centralizado"
            AREA_MAX_LINHA_FRAC = 0.5  # linha de verdade e fina, nao deveria cobrir mais que isso da ROI
            FRAMES_CONFIRMACAO = 3  # precisa ver centralizado em frames seguidos, nao so 1
            frames_centralizado_seguidos = 0
            while True:
                config.checar_parada()
                config.mover(12, 12)
                ret, frame = config.cap.read()
                if ret:
                    cx, contours, roi, mask = seguimento_de_linha.visao(frame)
                    
                    area_contorno = cv2.contourArea(max(contours, key=cv2.contourArea)) if contours else 0
                    area_roi = roi.shape[0] * roi.shape[1]
                    contorno_valido = (
                        cx is not None
                        and not seguimento_de_linha.gap_debug(contours)
                        and area_contorno < area_roi * AREA_MAX_LINHA_FRAC
                    )
                    centralizado = contorno_valido and abs(cx - roi.shape[1] // 2) < TOLERANCIA_CENTRO_LINHA
                   
                    frames_centralizado_seguidos = frames_centralizado_seguidos + 1 if centralizado else 0
                    print(f"[DESVIO-CENTRALIZA] cx={cx} area={area_contorno:.0f}/{area_roi} centralizado={centralizado} seguidos={frames_centralizado_seguidos}")
                    if frames_centralizado_seguidos >= FRAMES_CONFIRMACAO:
                        config.mover(14, 14)
                        time.sleep(0.1)
                        config.mover(-2, -2)
                        time.sleep(0.9)
                        break
                time.sleep(0.005)
            """config.mover(15, -15)
            time.sleep(1.3)
            config.mover(-2, 2)
            time.sleep(0.1)"""
            movimentos_bases.guinada2("D", 90, 18)
            config.mover(-13, -13)
            time.sleep(0.9)
            config.mover(2, 2)
            time.sleep(0.9)
            config.mover(0, 0)
            time.sleep(0.8)

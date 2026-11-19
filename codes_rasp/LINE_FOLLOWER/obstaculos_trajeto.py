
import time

import cv2

import config
import movimentos_bases
import seguimento_de_linha


def desvio():
    # Seletor: "quadrado" = desvio de sempre (contorna pela direita
    # seguindo a parede no sonar esquerdo) | "outro" = o novo, la embaixo.
    TIPO_OBSTACULO = "outro"  # "quadrado"

    if not (-5 < config.angulo_y() < 3):
        return
    if config.distancia_frente() <= 5:
    
        config.mover(-2, -2)
        time.sleep(1.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        if config.distancia_frente() <= 5:
            if TIPO_OBSTACULO == "quadrado":
            

                config.mover(18, 18)
                config.mover(-18, -18)
                time.sleep(0.2)
                config.mover(-2, -2)
                time.sleep(0.1)
            
                movimentos_bases.guinada2("D", 92, 14)
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
                TOLERANCIA_CENTRO_LINHA = 30
                AREA_MAX_LINHA_FRAC = 0.5
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
                movimentos_bases.guinada2("D", 90, 18)
                config.mover(-15, -15)
                time.sleep(0.9)
                config.mover(2, 2)
                time.sleep(0.9)
                config.mover(0, 0)
                time.sleep(0.8)

            else:
                # ===== OBSTACULO "OUTRO" =====
                config.mover(15, 15)
                time.sleep(1.5)
                config.mover(-15, -15)
                time.sleep(0.5)
                config.mover(-2, -2)
                movimentos_bases.guinada2("D", 90, 15)
                config.mover(15, 15)
                time.sleep(0.6)
                config.mover(-2, -2)
                time.sleep(0.1)
                while True:  # refaz o ciclo ate achar preto
                    config.checar_parada()
                    pixels_pretos = 0
                    girado_esquerda = 0.0  # soma real (giroscopio) dos giros pra esquerda
                    heading_ant = config._ler_heading_bruto()
                    while pixels_pretos <= 29000 and girado_esquerda < 180:  # repete ate ver preto ou chegar a 180 graus
                        movimentos_bases.guinada2("E", 22, 15)
                        heading = config._ler_heading_bruto()
                        girado_esquerda -= config._normalizar(heading - heading_ant)
                        heading_ant = heading

                        inicio_passo = time.time()
                        while pixels_pretos <= 29000 and time.time() - inicio_passo <= 0.6:
                            config.checar_parada()
                            config.mover(15, 15)
                            ret, frame = config.cap.read()
                            if ret:
                                pixels_pretos = cv2.countNonZero(seguimento_de_linha.visao(frame)[3])
                    config.mover(-2, -2)

                    if pixels_pretos > 29000:
                        # ===== achou a linha preta =====
                        print(f"[OBSTACULO-OUTRO] Achou preto ({pixels_pretos}) depois de girar {girado_esquerda:.0f} graus")
                        config.mover(15, 15)
                        time.sleep(0.2)
                        movimentos_bases.guinada2("E", 90, 15)
                        config.mover(15, 15)
                        time.sleep(1.9)
                        config.mover(-15, -15)
                        time.sleep(1.2)
                        movimentos_bases.guinada2("E", 179, 15)
                        config.mover(-15, -15)
                        time.sleep(0.7)
                        config.mover(-2, -2)
                        time.sleep(0.1)
                        break

                    else:
                        # ===== chegou a 180 graus sem achar preto =====
                        print(f"[OBSTACULO-OUTRO] Chegou a 180 graus ({girado_esquerda:.0f}) sem achar preto - parado")
                        config.mover(-2, -2)
                        config.mover(-16, -16)
                        time.sleep(0.2)         
                        movimentos_bases.guinada2("E", 90, 15)
                        config.mover(17, 17)
                        time.sleep(1.9)
                        config.mover(-15, -15)
                        time.sleep(0.5)
                        movimentos_bases.guinada2("D", 90, 15)
                        config.mover(-15, -15)
                        time.sleep(0.3)
                        while True:
                            config.checar_parada()
                            config.mover(18, 18)
                            if config.distancia_esquerda() < 15:
                                config.mover(-2, -2)
                                time.sleep(0.8)
                                break
                            time.sleep(0.005)
                        config.mover(17, 17)
                        time.sleep(0.4)
                        # sem break: volta pro comeco do while True e refaz o ciclo de girar + ir pra frente

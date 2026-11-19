"""
Saida da sala de resgate, rodando como processo proprio.

resgate.py chama este script (os.execv) depois das entregas. Achou a saida:
volta pro seguidor ja andando (main.py --auto). Botao no meio: volta pro
seguidor parado, esperando o botao.

Ao comecar, cria config.ARQUIVO_SAIDA_INICIADA - enquanto ele existir, quando
o seguidor achar a sala de resgate de novo ele vem direto pra ca, sem
refazer o resgate. O main.py apaga o arquivo quando e' iniciado do zero.

Mesma logica da antiga acionar_saida() do resgate.py, com os mesmos numeros.
"""

import os
import sys
import threading
import time

import cv2
import lgpio
import numpy as np

import config
import movimentos_bases

# Mesmo teto de velocidade do resgate.py, onde a saida foi calibrada.
config.VEL_MAX = 20
config.VEL_MIN = -20

LIMIAR_LINHA = 80
AREA_MIN_LINHA = 11000
AREA_MAX_LINHA_FRAC = 0.5
kernel_linha = np.ones((3, 3), np.uint8)

MAIN_PY = "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py"

# ---- ir ate o canto vermelho (camera da frente) - mesmos numeros do resgate.py ----
CROP_W, CROP_H = 320, 240
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30
red_min1 = np.array([0, 69, 0])
red_max1 = np.array([8, 255, 255])
red_min2 = np.array([157, 123, 0])
red_max2 = np.array([179, 255, 255])
kernel_tri = np.ones((3, 3), np.uint8)
AREA_MIN_TRI = 2500
AREA_MAX_TRI = 1000000
STOP_AREA_RATIO_TRIANGULO = 0.80
VOLTAS_TRIANGULO_NORMAL = 3
VOLTAS_TRIANGULO_LENTO = 1
VEL_GIRO_TRIANGULO = 16
VEL_GIRO_TRIANGULO_LENTO = 12
TEMPO_MAX_VOLTA_TRIANGULO_LENTO = 40.0
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_CENTER_MAX_TENTATIVAS = 15
TRIANGULO_CENTER_LOST_FRAMES = 15
CENTER_TURN_SPEED_MIN = 15
CENTER_TURN_SPEED_MAX = 25
APPROACH_STEP_TIME = 0.18

cap_frente = None


def _mask_linha_baixo(frame):
    h_ = frame.shape[0]
    roi = frame[int(h_ * 0.1):h_, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, LIMIAR_LINHA, 255, cv2.THRESH_BINARY_INV)[1]
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)


def _pixels_pretos():
    ret, frame = config.cap.read()
    if not ret:
        return 0
    return cv2.countNonZero(_mask_linha_baixo(frame))


def _detectar_linha_embaixo(frame):
    mask = _mask_linha_baixo(frame)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return False
    area = cv2.contourArea(max(contours, key=cv2.contourArea))
    return AREA_MIN_LINHA <= area < mask.shape[0] * mask.shape[1] * AREA_MAX_LINHA_FRAC


def alinhar_na_linha():
    FRAMES_CONFIRMACAO = 3
    ALINHAR_VEL = 13
    seguidos = 0
    while True:
        config.checar_parada()
        config.mover(ALINHAR_VEL, -ALINHAR_VEL)
        ret, frame = config.cap.read()
        detectado = ret and _detectar_linha_embaixo(frame)
        seguidos = seguidos + 1 if detectado else 0
        print(f"[SAIDA-ALINHA] detectado={detectado} seguidos={seguidos}")
        if seguidos >= FRAMES_CONFIRMACAO:
            print("[SAIDA] Contorno da linha encontrado de novo, alinhado")
            break
        time.sleep(0.005)
    config.parar()
    movimentos_bases.guinada2('D', 10, 20)


def abrir_camera_frente():
    global cap_frente
    tentativas = 0
    c = config.abrir_camera_generica(config.CAMERA_VITIMA_PATH)
    while c is None:
        tentativas += 1
        print(f"[Camera] Frente ainda nao apareceu, tentando de novo... ({tentativas})")
        config.checar_parada()
        time.sleep(1)
        if tentativas % 3 == 0:
            config.resetar_usb_camera(config.CAMERA_VITIMA_PATH)
        c = config.abrir_camera_generica(config.CAMERA_VITIMA_PATH)
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap_frente = c


def capturar_crop():
    for _ in range(6):
        cap_frente.read()
    ret, frame = cap_frente.read()
    if not ret:
        return None
    hh, ww = frame.shape[:2]
    x1 = max(0, ww // 2 - CROP_W // 2)
    y1 = max(0, hh // 2 - CROP_H // 2)
    return frame[y1:y1 + CROP_H, x1:x1 + CROP_W].copy()


def _eh_triangulo(contorno, encostando_borda=False):
    area = cv2.contourArea(contorno)
    if area < AREA_MIN_TRI or area > AREA_MAX_TRI:
        return False
    aproximado = cv2.approxPolyDP(contorno, 0.06 * cv2.arcLength(contorno, True), True)
    min_vertices = 2 if encostando_borda else 3
    if len(aproximado) < min_vertices or len(aproximado) > 4:
        return False
    if not encostando_borda and not cv2.isContourConvex(aproximado):
        return False
    return cv2.moments(contorno)["m00"] != 0


def detectar_vermelho(crop):
    """(cx, area) do maior triangulo vermelho no crop, ou (None, None)."""
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(cv2.inRange(hsv, red_min1, red_max1), cv2.inRange(hsv, red_min2, red_max2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_tri, iterations=2)
    mask = cv2.dilate(mask, kernel_tri, iterations=2)

    melhor = None
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        x, y, w, h_bb = cv2.boundingRect(c)
        borda = x <= 2 or y <= 2 or x + w >= crop.shape[1] - 2 or y + h_bb >= crop.shape[0] - 2
        if _eh_triangulo(c, encostando_borda=borda):
            area = cv2.contourArea(c)
            if melhor is None or area > melhor[1]:
                M = cv2.moments(c)
                melhor = (int(M["m10"] / M["m00"]), area)
    return melhor if melhor else (None, None)


def centralizar_triangulo():
    perdida = 0
    for _ in range(TRIANGULO_CENTER_MAX_TENTATIVAS):
        config.checar_parada()
        crop = capturar_crop()
        if crop is None:
            config.parar()
            return False
        cx, _ = detectar_vermelho(crop)
        if cx is None:
            perdida += 1
            if perdida >= TRIANGULO_CENTER_LOST_FRAMES:
                print("Perdeu o triangulo durante centralizacao")
                config.parar()
                return False
            continue
        perdida = 0
        erro = cx - IMG_CENTER_X
        if abs(erro) <= TOLERANCE:
            print(f"Centralizado no triangulo (cx={cx})")
            config.parar()
            return True
        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN)
        if erro < 0:
            config.mover(-speed, speed)
        else:
            config.mover(speed, -speed)
        time.sleep(0.05 + erro_norm * 0.10)
        config.parar()
        time.sleep(0.05)
    print("Estourou tentativas de centralizacao no triangulo")
    config.parar()
    return False


def aproximar_triangulo():
    perdida = 0
    for passo in range(TRIANGULO_MAX_APPROACH_STEPS):
        config.checar_parada()
        crop = capturar_crop()
        if crop is None:
            config.parar()
            return False
        cx, area = detectar_vermelho(crop)
        if cx is None:
            config.parar()
            perdida += 1
            print(f"Triangulo sumiu durante aproximacao ({perdida}/{TRIANGULO_LOST_FRAMES})")
            if perdida >= TRIANGULO_LOST_FRAMES:
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue
        perdida = 0
        ratio = area / (CROP_W * CROP_H)
        print(f"[canto] passo={passo} area_ratio={ratio:.2f} cx={cx}")
        if ratio >= STOP_AREA_RATIO_TRIANGULO:
            print("Perto o suficiente do triangulo, parando")
            config.parar()
            return True
        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("Saiu do centro durante aproximacao, recentralizando")
            if not centralizar_triangulo():
                return False
            continue
        config.mover(20, 20)
        time.sleep(0.05)
    print("Estourou o limite de passos na aproximacao")
    config.parar()
    return False


def _heading():
    try:
        return config._ler_heading_bruto()
    except Exception as e:
        print(f"[SAIDA] Falha ao ler heading ({e})")
        return None


def _volta_procurando_triangulo(velocidade, tempo_max=None):
    """Uma volta de 360 (giroscopio) girando intermitente; True se achou."""
    girado = 0.0
    heading_anterior = _heading()
    inicio = time.time()
    while girado < 360:
        if tempo_max is not None and time.time() - inicio >= tempo_max:
            print(f"[BUSCAR_TRIANGULO] Volta estourou {tempo_max:.0f}s com so {girado:.0f} graus, seguindo")
            return False
        config.checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, area = detectar_vermelho(crop)
            if cx is not None:
                print(f"[BUSCAR_TRIANGULO] Detectado vermelho area={area} cx={cx}")
                return True
        config.mover(velocidade, -velocidade)
        time.sleep(0.3)
        heading_atual = _heading()
        if heading_atual is None or heading_anterior is None:
            girado += 4.0
        else:
            girado += config._normalizar(heading_atual - heading_anterior)
        heading_anterior = heading_atual
    return False


def buscar_triangulo():
    for volta in range(VOLTAS_TRIANGULO_NORMAL):
        print(f"[BUSCAR_TRIANGULO] Volta {volta + 1}/{VOLTAS_TRIANGULO_NORMAL} (vel {VEL_GIRO_TRIANGULO})")
        if _volta_procurando_triangulo(VEL_GIRO_TRIANGULO):
            return True
    for volta in range(VOLTAS_TRIANGULO_LENTO):
        print(f"[BUSCAR_TRIANGULO] Volta lenta {volta + 1}/{VOLTAS_TRIANGULO_LENTO} (vel {VEL_GIRO_TRIANGULO_LENTO})")
        if _volta_procurando_triangulo(VEL_GIRO_TRIANGULO_LENTO, TEMPO_MAX_VOLTA_TRIANGULO_LENTO):
            return True
    print("[BUSCAR_TRIANGULO] Nao achou nas voltas, reposicionando...")
    return False


def reposicionar_canto(tentativa, vel_recuo=25):
    if tentativa <= 2:
        config.mover(-vel_recuo, -vel_recuo)
        time.sleep(0.4)
        config.mover(-30, 0)
        time.sleep(0.8)
    else:
        config.mover(20, 20)
        time.sleep(0.7)
        config.mover(-20, 0)
        time.sleep(0.7)
    config.parar()


def ir_ate_canto_vermelho():
    """Mesmo caminho da entrega da bola preta no resgate.py (acha, centraliza
    e chega no triangulo vermelho), sem nenhum movimento de garra."""
    global cap_frente
    abrir_camera_frente()
    print("[SAIDA] Indo ate o canto vermelho...")
    tentativas = 0
    while not (buscar_triangulo() and centralizar_triangulo() and aproximar_triangulo()):
        print("[SAIDA] Nao chegou no canto vermelho, reposicionando...")
        tentativas += 1
        reposicionar_canto(tentativas)
    config.parar()
    cap_frente.release()
    cap_frente = None
    print("[SAIDA] No canto vermelho")


def voltar_pro_seguidor(andando):
    """andando=True: saiu da sala, volta pro seguidor ja rodando (--auto).
    False: botao, volta parado esperando o botao."""
    config.parar()
    if cap_frente is not None:
        cap_frente.release()
    config.cap.release()
    lgpio.gpiochip_close(config.h)
    args = [sys.executable, MAIN_PY]
    if andando:
        args.append("--auto")
    args.append(f"--entregas={config.ENTREGAS_HERDADAS}")
    os.execv(sys.executable, args)


def _saiu():
    print("saiiiiiii")
    config.mover(15, 15)
    time.sleep(0.9)
    movimentos_bases.guinada2('E', 20, 15)
    alinhar_na_linha()
    time.sleep(2.0)
    voltar_pro_seguidor(andando=True)


def manobra_parede():
    """Empurra pra frente contra a parede/canto e ve pelo giroscopio se o
    robo foi desviado (canto inclinado) - escolhe pra onde virar a partir disso."""
    config.resetar_giroscopio()
    angulozero = config.angulo_z()
    print("paredeeee")
    config.mover(-2, -2)
    time.sleep(0.3)
    
    config.mover(30, 30)
    time.sleep(1.5)
    if (config.angulo_z() - angulozero) < -19:
        config.mover(-20, -20)
        time.sleep(0.5)  # 0.2 - re antes da guinada (sequencia da de 40)
        movimentos_bases.guinada2('D', 86, 15)
        config.mover(19, 19)
        time.sleep(0.9)
        movimentos_bases.guinada2('D', 44, 18)  # 15
    else:
        config.mover(-15, -15)
        time.sleep(0.7)
        movimentos_bases.guinada2('D', 84, 15, vel_m2=30)  # M2 (traseira esq) mais forte



def saida():
    config.mover_servo(config.PINO_PESADO, 130)

    # Antes de comecar a saida: re por 2 s.
    config.mover(-20, -20)
    fim_re = time.time() + 2.0
    while time.time() < fim_re:
        config.checar_parada()
        time.sleep(0.02)
    config.parar()

    # Vai ate o canto vermelho, faz a manobra de parede ali e so depois entra
    # no loop da saida.
    ir_ate_canto_vermelho()
    config.mover(20, 20)
    time.sleep(2.0)
    config.mover(-20, -20)
    time.sleep(0.5)  # 0.15 - re antes da guinada (sequencia da de 40)
    
    movimentos_bases.guinada2('D', 86, 15)
    config.mover(19, 19)
    time.sleep(0.9)
    movimentos_bases.guinada2('D', 40, 15)
    
   

    while True:
        config.checar_parada()

        if not config.sonar_valido():
            time.sleep(0.02)
            continue

        frente = config.distancia_frente()
        esquerda = config.distancia_esquerda()

        pixels_pretos = _pixels_pretos()
        if pixels_pretos > 20000:
            print("muito pixel preto")
        print(f"pixels_pretos={pixels_pretos}")

        config.mover(16, 16)
        print(config.angulo_z())
        if frente < 9:
            manobra_parede()
        print("preto quanitidade",pixels_pretos)
        if 15000 < pixels_pretos < 28000:
            print("saida frente")
            config.mover(-15, -15)
            time.sleep(0.2)
            inicio_frente = time.time()
            while pixels_pretos <= 28000 and time.time() - inicio_frente <= 0.9:
                config.checar_parada()
                config.mover(15, 15)
                pixels_pretos = _pixels_pretos()
                time.sleep(0.02)
            config.parar()
            print("pixels_pretos=", pixels_pretos)
            if pixels_pretos >= 28000:
                _saiu()
            else:
                config.mover(-15, -15)
                time.sleep(2.0)
                movimentos_bases.guinada2('D', 86, 15)
                config.mover(18, 18)
                time.sleep(1.2)

        if esquerda > 40:
            print("buracooooo")
            config.mover(-2, -2)
            time.sleep(0.4)
            config.mover(15, 15)
            time.sleep(0.33)
            movimentos_bases.guinada2('E', 87, 15)

            inicio_frente = time.time()
            while pixels_pretos <= 30000 and time.time() - inicio_frente <= 2.1:
                config.checar_parada()
                config.mover(15, 15)
                pixels_pretos = _pixels_pretos()
                time.sleep(0.02)
            config.parar()
            print("pixels_pretos=", pixels_pretos)
            if pixels_pretos > 29000:
                _saiu()
            else:
                config.mover(-15, -15)
                time.sleep(1.6)
                movimentos_bases.guinada2('D', 86, 15)
                if config.distancia_frente() < 15:
                    config.mover(-15, -15)
                    time.sleep(0.3)
                    movimentos_bases.guinada2('D', 86, 15)
                config.mover(18, 18)
                time.sleep(1.2)


if __name__ == "__main__":
    open(config.ARQUIVO_SAIDA_INICIADA, "w").close()
    print("[SAIDA] Iniciando a saida da sala de resgate")

    config.handshake_serial()
    threading.Thread(target=config.serial_thread, daemon=True).start()
    config.iniciar_mpu()
    config.iniciar_camera()

    config.robo_ativo.set()
    threading.Thread(target=config.botao_thread, daemon=True).start()

    try:
        saida()
    except config.RoboParado:
        print("[BOTAO] Parado no meio da saida, voltando pro seguidor")
        voltar_pro_seguidor(andando=False)
    except KeyboardInterrupt:
        print("\nEncerrado (Ctrl+C)")
    finally:
        config.parar()

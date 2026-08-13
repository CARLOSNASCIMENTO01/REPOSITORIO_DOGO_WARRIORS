"""
Tudo relacionado a seguir a linha: visao crua (threshold + contorno preto),
deteccao de gap/travamento/fita vermelha, o PID que calcula velL/velR
(controle) e a deteccao/reacao aos marcadores verdes de curva.
"""

import time

import cv2
import numpy as np

import config
import movimentos_bases

# =========================================================================
# VISAO DA LINHA
# =========================================================================
LIMIAR   = 80
AREA_MIN = 10000 # 11000
kernal = np.ones((3, 3), np.uint8)

_trav_frame_ref = None
_trav_tempo_ref = time.time()


def visao(frame):
    h_, w_ = frame.shape[:2]
    roi = frame[int(h_ * 0.1):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None
    if contours:
        main = max(contours, key=cv2.contourArea)
        M = cv2.moments(main)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])

    return cx, contours, roi, mask


def gap_debug(contours):
    if len(contours) == 0:
        return True
    maior = max(contours, key=cv2.contourArea)
    area_ = cv2.contourArea(maior)
    if area_ < AREA_MIN:
        return True
    return False


def detectar_travamento(roi):
    global _trav_frame_ref, _trav_tempo_ref
    LIMIAR_DIFF = 8
    TEMPO_TRAV = 4.0
    INTERVALO_REF = 2.0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    agora = time.time()

    if _trav_frame_ref is None:
        _trav_frame_ref = gray.copy()
        _trav_tempo_ref = agora
        return False

    movimento = np.mean(cv2.absdiff(gray, _trav_frame_ref))

    if agora - _trav_tempo_ref >= INTERVALO_REF:
        if movimento >= LIMIAR_DIFF:
            _trav_frame_ref = gray.copy()
            _trav_tempo_ref = agora
            return False
        else:
            tempo_parado = agora - _trav_tempo_ref
            print(f"[TRAV] diff={movimento:.1f} | parado ha {tempo_parado:.1f}s")
            if tempo_parado >= TEMPO_TRAV:
                print("[TRAVAMENTO] Robo travado!")
                _trav_frame_ref = None
                _trav_tempo_ref = agora
                return True
    return False


def detectar_fita_vermelha(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
    )
    h_, w_ = mask.shape
    for y in range(0, h_, 5):
        if cv2.countNonZero(mask[y, :]) > w_ * 0.6:
            print("[VERMELHO] Fita detectada - pausando 6s")
            config.parar()
            time.sleep(6)
            limpar_camera(10)
            return True
    return False


def limpar_camera(n=5):
    for _ in range(n):
        config.cap.read()


# =========================================================================
# CONTROLE (segue linha) - PID
# =========================================================================
Kp = 0.9 #0.9
Kd = 0.25 # 0.3
VEL_BASE = 18 #15
DEADZONE = 0 # 8, 0

contador_gangorra = 0
ultimo_erro = 0
ultimo_tempo = time.time()


def controle(cx, contours, roi):
    global ultimo_erro, ultimo_tempo, contador_gangorra

    w = roi.shape[1]
    tempo_atual = time.time()
    dt = tempo_atual - ultimo_tempo
    if dt <= 0:
        dt = 0.0001

    if cx is not None:
        centro = w // 2
        erro = (cx - centro) / centro * 100
    else:
        erro = ultimo_erro

    if gap_debug(contours):
        erro = 0

    derivada = (erro - ultimo_erro) / dt
    derivada = np.clip(derivada, -300, 300)

    # Deadzone so no ajuste final, nao no erro em si - zerar o erro ANTES de
    # calcular a derivada corrompia o termo D (pico artificial de -ultimo_erro/dt
    # toda vez que entrava na deadzone vindo de um erro grande), causando as
    # guinadas bruscas que a deadzone deveria estar evitando.
    if abs(erro) < DEADZONE:
        ajuste = 0
    else:
        ajuste = (Kp * erro) + (Kd * derivada)

    print("angulo y ", config.angulo_y(), " | angulo z ", config.angulo_z())
    if -5 < config.angulo_y() < 3 and contador_gangorra != 2:
        contador_gangorra = 0

    # Muito verde na ROI (perto de um marcador de curva) - diminui a
    # velocidade pra dar mais estabilidade na hora de decidir a curva em
    # vez de passar correndo por cima do marcador.
    hsv_ctrl = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_verde_ctrl = cv2.inRange(hsv_ctrl, green_min, green_max)
    pixels_verde_ctrl = cv2.countNonZero(mask_verde_ctrl)

    if config.angulo_y() <= -10:
        contador_gangorra = 2
        velocidade_base = 26
        velL = np.clip(velocidade_base + ajuste, config.VEL_MIN, config.VEL_MAX)
        velR = np.clip(velocidade_base - ajuste, config.VEL_MIN, config.VEL_MAX)
    elif pixels_verde_ctrl > 10000:
        print(f"[VERDE-CONTROLE] pixels_verde={pixels_verde_ctrl} - freando")
        
        velocidade_base = 12
        velL = np.clip(velocidade_base + ajuste, -13, 13)
        velR = np.clip(velocidade_base - ajuste, -13, 13)
        if pixels_verde_ctrl > 24000:
            # So vira se o verde nao estiver mais "colado" numa linha preta
            # continuando embaixo dele - reusa o check_black() (o mesmo
            # calculo que ja faz um marcador unico ser ignorado quando
            # ainda esta apos uma linha preta, ainda nao chegou) em cima
            # de so UM dos contornos verdes achados aqui. Nao precisa dos
            # dois marcadores nem de nenhuma deteccao nova - so o sinal
            # "baixo" (preto colado embaixo do verde = linha ainda
            # conectada = nao chegou de verdade) que ja existe.
            hsv_verde_ref = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            green_mask_ref = cv2.inRange(hsv_verde_ref, green_min, green_max)
            green_mask_ref = cv2.dilate(green_mask_ref, kernal, iterations=3)
            green_mask_ref = cv2.erode(green_mask_ref, kernal, iterations=2)
            contornos_verde_ref, _ = cv2.findContours(green_mask_ref, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            contornos_verde_ref = [c for c in contornos_verde_ref if cv2.contourArea(c) > MIN_GREEN_AREA]

            apos_linha_preta = False
            if contornos_verde_ref:
                roi_h_ref, _ = roi.shape[:2]
                corte_top_ref = int(roi_h_ref * 0.4)
                black_ref = cv2.inRange(roi, black_min, black_max_bot)
                black_ref[:corte_top_ref, :] = cv2.inRange(roi[:corte_top_ref, :], black_min, black_max_top)
                black_ref = cv2.dilate(black_ref, kernal, iterations=3)
                black_ref = cv2.erode(black_ref, kernal, iterations=2)

                green_box_ref = cv2.boxPoints(cv2.minAreaRect(contornos_verde_ref[0]))
                row_ref = check_black(np.zeros((1, 5), dtype=np.int16), 0, green_box_ref, black_ref)
                apos_linha_preta = row_ref[0, 0] == 1  # baixo=1 -> linha preta ainda conectada embaixo
                print(f"[VERDE-REF] contornos_verde={len(contornos_verde_ref)} box={green_box_ref.tolist()} "
                      f"voto(baixo,cima,esq,dir)={row_ref[0, :4].tolist()} apos_linha_preta={apos_linha_preta}")
            else:
                print("[VERDE-REF] nenhum contorno verde grande achado pra referencia")

            if apos_linha_preta:
                print(f"[VERDE-CONTROLE] pixels_verde={pixels_verde_ctrl} - verde apos linha preta, ignorando giro")
            else:
                print("gira 180")
                config.mover(-2, -2)
                time.sleep(0.3)
                movimentos_bases.guinada2("D", 170, 20)
                config.mover(15, 15)
                time.sleep(0.22)
                config.mover(-2, -2)
                time.sleep(0.2)
                config.parar()
                limpar_camera(10)
        """if pixels_verde_ctrl > 22000:
            print(f"[VERDE-CONTROLE] pixels_verde={pixels_verde_ctrl} - muito perto/parado, recuando")
            inicio_caso_verde = time.time()
            while time.time() - inicio_caso_verde < 2.0:
                config.checar_parada()
                ret_cv, frame_cv = config.cap.read()
                if not ret_cv:
                    continue
                _, _, roi_cv, _ = visao(frame_cv)
                hsv_cv = cv2.cvtColor(roi_cv, cv2.COLOR_BGR2HSV)
                mask_cv = cv2.inRange(hsv_cv, green_min, green_max)
                contornos_cv, _ = cv2.findContours(mask_cv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                contornos_grandes_cv = [c for c in contornos_cv if cv2.contourArea(c) > MIN_GREEN_AREA]

                if len(contornos_grandes_cv) >= 2:
                    caso_verde_cv = "verde verde"
                elif contador_180 > 0:
                    caso_verde_cv = "180"
                elif contador_direita > contador_esquerda:
                    caso_verde_cv = "direita"
                elif contador_esquerda > contador_direita:
                    caso_verde_cv = "esquerda"
                else:
                    caso_verde_cv = "reto"

                print(f"[VERDE-CONTROLE] processando caso: {caso_verde_cv}")
                if caso_verde_cv != "reto":
                    verde(roi_cv)
                    break"""
    else:
        velocidade_base = VEL_BASE
        velL = np.clip(velocidade_base + ajuste, config.VEL_MIN, config.VEL_MAX)
        velR = np.clip(velocidade_base - ajuste, config.VEL_MIN, config.VEL_MAX)
    if config.angulo_y() >= -3 and contador_gangorra == 2:
        print("descendo gangorra ")
        contador_gangorra = 1
        config.mover(-10, -10)
        time.sleep(1.0)
        config.mover(-40, -40)
        time.sleep(0.1)
        config.mover(-10, -10)
        time.sleep(1.0)
        velocidade_base = 20
        velL = velocidade_base + ajuste
        velR = velocidade_base - ajuste
    
    config.mover(velL, velR)
    ultimo_erro = erro

    ultimo_tempo = tempo_atual
    return ajuste, contours


# =========================================================================
# DETECCAO DE MARCADOR VERDE
# =========================================================================
green_min = np.array([17, 48, 39]) #40, 50, 45
green_max = np.array([89, 255, 255]) #85, 255, 255
black_min     = np.array([0, 0, 0])
black_max_top = np.array([78, 78, 78]) #90, 90, 90
black_max_bot = np.array([91, 91, 91]) #135, 135, 135
MIN_GREEN_AREA = 1500 #2500
LIMIAR_PIXELS_VERDE_OUTRO_LADO = 1500  # verde do lado oposto acima disso = pode ser o 2o marcador do 180 comecando a aparecer

DEBUG_VERDE = True
DEBUG_SEM_ACAO = False
PREVIEW_VERDE_EVERY_N = 3

contador_direita = 0
contador_esquerda = 0
contador_180 = 0
_preview_verde_contador = 0


def resetar_estado():
    """Reseta todo o estado deste modulo (travamento, PID e voto de verde) -
    chamado quando o robo reinicia pelo botao."""
    global _trav_frame_ref, _trav_tempo_ref
    global contador_gangorra, ultimo_erro, ultimo_tempo
    global contador_direita, contador_esquerda, contador_180, _preview_verde_contador

    _trav_frame_ref = None
    _trav_tempo_ref = time.time()

    contador_gangorra = 0
    ultimo_erro = 0
    ultimo_tempo = time.time()

    contador_direita = 0
    contador_esquerda = 0
    contador_180 = 0
    _preview_verde_contador = 0


def check_black(black_around_sign, i, green_box, black_image):
    gb_y = green_box[green_box[:, 1].argsort()]
    marker_height = int(gb_y[-1][1] - gb_y[0][1])
    sign_y = int(gb_y[2][1])
    black_around_sign[i, 4] = sign_y

    y1, y2 = sign_y, min(sign_y + int(marker_height * 0.8), config.camera_y)
    x1, x2 = int(min(gb_y[2][0], gb_y[3][0])), int(max(gb_y[2][0], gb_y[3][0]))
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 0] = 1

    y1, y2 = max(int(gb_y[1][1]) - int(marker_height * 0.8), 0), int(gb_y[1][1])
    x1, x2 = max(int(min(gb_y[0][0], gb_y[1][0])), 0), max(int(max(gb_y[0][0], gb_y[1][0])), 0)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 1] = 1

    gb_x = green_box[green_box[:, 0].argsort()]

    y1, y2 = int(min(gb_x[0][1], gb_x[1][1])), int(max(gb_x[0][1], gb_x[1][1]))
    x1, x2 = max(int(gb_x[1][0]) - int(marker_height * 0.8), 0), int(gb_x[1][0])
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 2] = 1

    y1, y2 = int(min(gb_x[2][1], gb_x[3][1])), int(max(gb_x[2][1], gb_x[3][1]))
    x1, x2 = int(gb_x[2][0]), min(int(gb_x[2][0]) + int(marker_height * 0.8), config.camera_x)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        black_around_sign[i, 3] = 1

    return black_around_sign


def determine_turn_direction(black_around_sign, green_centers):
    votes_left = 0
    votes_right = 0
    mid_x = (max(green_centers) + min(green_centers)) / 2 if len(green_centers) >= 2 else config.camera_x / 2

    for i, row in enumerate(black_around_sign):
        if row[1] == 1 and row[2] == 1 and row[0] == 0 and row[3] == 0:
            votes_right += 1
        elif row[1] == 1 and row[3] == 1 and row[0] == 0 and row[2] == 0:
            votes_left += 1

    if votes_left >= 1 and votes_right >= 1:
        return False, False, True
    return votes_left > 0, votes_right > 0, False


def check_green(contours_grn, black_image, green_image):
    global contador_esquerda, contador_direita, contador_180
    black_around_sign = np.zeros((len(contours_grn), 5), dtype=np.int16)
    green_centers = []

    for i, contour in enumerate(contours_grn):
        if cv2.contourArea(contour) <= MIN_GREEN_AREA:
            green_centers.append(config.camera_x / 2)
            continue
        green_box = cv2.boxPoints(cv2.minAreaRect(contour))
        green_centers.append(float(np.mean(green_box[:, 0])))
        black_around_sign = check_black(black_around_sign, i, green_box, black_image)

    turn_left, turn_right, turn_180 = determine_turn_direction(black_around_sign, green_centers)
    pixels_verde = cv2.countNonZero(green_image)
    print("pixel verde", pixels_verde)

    contador_180 = contador_180 + 1 if turn_180 else 0
    contador_esquerda = contador_esquerda + 1 if turn_left else 0
    contador_direita = contador_direita + 1 if turn_right else 0
    turn_left, turn_right, turn_180 = determine_turn_direction(black_around_sign, green_centers)
    if turn_180:
        caso = "GIRAR_180"
    elif turn_left and pixels_verde < 22000 and contador_esquerda > 2 and contador_direita < 1:  #15500
        caso = "ESQUERDA"
    elif turn_right and pixels_verde < 22000 and contador_direita > 2 and contador_esquerda < 1:  #15500
        caso = "DIREITA"
    else:
        caso = "RETO"
    return caso
    
    """if turn_180:
        return "GIRAR_180"
    elif turn_left :
        return "ESQUERDA"
    elif turn_right :
        return "DIREITA"
    else:
        return "RETO"""


def detectar_verde(roi):
    roi_h, roi_w = roi.shape[:2]
    corte_top = int(roi_h * 0.4)

    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, black_max_top)
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image, kernal, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    contours_grn, _ = cv2.findContours(green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_grn) == 0:
        return None

    acao = check_green(contours_grn, black_image, green_image)
    return acao if acao != "RETO" else None


def mostrar_preview_verde(roi):
    global _preview_verde_contador
    if not DEBUG_VERDE:
        return
    _preview_verde_contador += 1
    if _preview_verde_contador % PREVIEW_VERDE_EVERY_N != 0:
        return

    roi_h, roi_w = roi.shape[:2]
    corte_top = int(roi_h * 0.4)

    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, black_max_top)
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image, kernal, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    contours_grn, _ = cv2.findContours(green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    debug_img = roi.copy()
    for c in contours_grn:
        area_c = cv2.contourArea(c)
        cor = (0, 255, 0) if area_c > MIN_GREEN_AREA else (0, 165, 255)
        box = cv2.boxPoints(cv2.minAreaRect(c)).astype(int)
        cv2.drawContours(debug_img, [box], 0, cor, 1)
        cv2.putText(debug_img, f"{int(area_c)}", tuple(box[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.3, cor, 1)

    escala = 3
    debug_big = cv2.resize(debug_img, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
    black_big = cv2.resize(black_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
    green_big = cv2.resize(green_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)

    cv2.imshow("Verde - ROI (contornos)", debug_big)
    cv2.imshow("Verde - mask preto", black_big)
    cv2.imshow("Verde - mask verde", green_big)
    cv2.waitKey(1)


def verde(roi):

    acao_verde = detectar_verde(roi)
    if not acao_verde:
        return

   
    
    config.parar()

    if acao_verde == "ESQUERDA":
        print("verde_esquerda")
        config.mover(-2, -2)
        time.sleep(0.2)
        config.mover(15, 15)
        time.sleep(0.3)
        config.parar()
        movimentos_bases.guinada2('E', 85, 20)
        config.mover(15, 15)
        time.sleep(0.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        inicio = time.time()
        while time.time() - inicio < 1.0:
            config.checar_parada()
            ret, frame = config.cap.read()
            if not ret:
                continue
            cx, contours, roi_loop, mask = visao(frame)
            controle(cx, contours, roi_loop)

    elif acao_verde == "DIREITA":
        print("verde_direita")
        limpar_camera(10)
        config.mover(-2, -2)
        time.sleep(0.2)
        config.mover(15, 15)
        time.sleep(0.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        config.parar()
        movimentos_bases.guinada2('D', 85, 20)
        config.mover(15, 15)
        time.sleep(0.3)
        inicio = time.time()
        while time.time() - inicio < 1.0:
            config.checar_parada()
            ret, frame = config.cap.read()
            if not ret:
                continue
            cx, contours, roi_loop, mask = visao(frame)
            controle(cx, contours, roi_loop)

    elif acao_verde == "GIRAR_180":
        print("gira 180")
        config.mover(-2, -2)
        time.sleep(0.3)
        movimentos_bases.guinada2("D", 170, 20)
        config.mover(15, 15)
        time.sleep(0.22)
        config.mover(-2, -2)
        time.sleep(0.2)
        config.parar()
        limpar_camera(10)

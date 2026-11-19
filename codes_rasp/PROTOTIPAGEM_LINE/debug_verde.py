"""
debug_verde.py - diagnostico puro do marcador verde (so imprime, nao move
motor nenhum). Mesma logica geometrica do check_black/determine_turn_direction
do line_overenginer_base.py (olha o preto ENCOSTADO em cada lado do box do
verde pra decidir a curva), sem os contadores/votos e sem o limiar de
countNonZero(verde) que seguimento_de_linha.py usa pra estabilizar - aqui e
a decisao geometrica crua, pra calibrar HSV/mascara antes de voltar pra
producao.

Nao importa config.py (sem GPIO, sem motor) - so precisa da camera. O
robo.service (main.py/supertans.py) ja abre a mesma camera, entao pare ele
antes: "sudo systemctl stop robo.service".

Com DISPLAY disponivel abre janelas com trackbar pra calibrar HSV/preto/area
em tempo real. Sem DISPLAY, roda so no terminal com os valores fixos do
CONFIG abaixo (mesmo ponto de partida: valores ja calibrados em
seguimento_de_linha.py) e ainda salva snapshot em disco pra inspecionar
depois (SALVAR_SNAPSHOT_A_CADA).

Teclas (com janela aberta):
  q - sair
  s - imprimir os valores calibrados agora (pra copiar pro CONFIG)
"""

import os
import time

import cv2
import numpy as np

# ===== CONFIG =====
CAMERA_PATH = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_INDEX_FALLBACK = 0
LARGURA, ALTURA = 640, 480  # bem maior que o 160x120 do seguidor - nossa lente tem FOV pequeno, precisa do detalhe

CORTE_TOPO_FRAC = 0.1  # mesmo corte de seguimento_de_linha.visao() - tira o fundo, fica so a pista

GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN = 17, 48, 39   # ponto de partida = seguimento_de_linha.py
GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX = 89, 255, 255
BLACK_MAX_TOP = 78   # 0-255 por canal BGR, regiao de cima da ROI (mais sombra)
BLACK_MAX_BOT = 91   # regiao de baixo da ROI (mais perto da camera, luz melhor)
AREA_MIN_VERDE = 1500
KERNEL = np.ones((3, 3), np.uint8)

# Exige opt-in explicito (nao so checar $DISPLAY) - uma sessao remota (SSH)
# pode ter DISPLAY setado sem acesso real ao X11, e cv2.namedWindow/imshow
# trava sem avisar nesse caso. Pra abrir as janelas: DEBUG_VERDE_GUI=1
MOSTRAR_JANELAS = os.environ.get("DEBUG_VERDE_GUI") == "1"
SALVAR_SNAPSHOT_A_CADA = 30  # frames - sobrescreve sempre os mesmos arquivos
SNAPSHOT_DIR = "/tmp/debug_verde_snapshots"


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_PATH, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX_FALLBACK, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            "Nao conseguiu abrir a camera de linha - ela ja esta aberta por "
            "outro processo? Pare o robo.service: sudo systemctl stop robo.service"
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def criar_trackbars():
    cv2.namedWindow("Calibrar", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H min", "Calibrar", GREEN_H_MIN, 179, lambda v: None)
    cv2.createTrackbar("H max", "Calibrar", GREEN_H_MAX, 179, lambda v: None)
    cv2.createTrackbar("S min", "Calibrar", GREEN_S_MIN, 255, lambda v: None)
    cv2.createTrackbar("S max", "Calibrar", GREEN_S_MAX, 255, lambda v: None)
    cv2.createTrackbar("V min", "Calibrar", GREEN_V_MIN, 255, lambda v: None)
    cv2.createTrackbar("V max", "Calibrar", GREEN_V_MAX, 255, lambda v: None)
    cv2.createTrackbar("Preto max topo", "Calibrar", BLACK_MAX_TOP, 255, lambda v: None)
    cv2.createTrackbar("Preto max baixo", "Calibrar", BLACK_MAX_BOT, 255, lambda v: None)
    cv2.createTrackbar("Area min verde", "Calibrar", AREA_MIN_VERDE, 10000, lambda v: None)


def ler_trackbars():
    g = lambda nome: cv2.getTrackbarPos(nome, "Calibrar")
    green_min = np.array([g("H min"), g("S min"), g("V min")])
    green_max = np.array([g("H max"), g("S max"), g("V max")])
    return green_min, green_max, g("Preto max topo"), g("Preto max baixo"), g("Area min verde")


def mascaras(roi, green_min, green_max, preto_top, preto_bot):
    roi_h, _ = roi.shape[:2]
    corte_top = int(roi_h * CORTE_TOPO_FRAC)

    black_min = np.array([0, 0, 0])
    black_mask = cv2.inRange(roi, black_min, np.array([preto_bot] * 3))
    black_mask[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, np.array([preto_top] * 3))
    black_mask = cv2.dilate(black_mask, KERNEL, iterations=3)
    black_mask = cv2.erode(black_mask, KERNEL, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(hsv, green_min, green_max)
    green_mask = cv2.dilate(green_mask, KERNEL, iterations=3)
    green_mask = cv2.erode(green_mask, KERNEL, iterations=2)

    return black_mask, green_mask


def check_black(sinais, i, box, black_mask, roi_w, roi_h):
    by = box[box[:, 1].argsort()]
    altura_marcador = by[-1][1] - by[0][1]
    sinais[i, 4] = int(by[2][1])

    y1, y2 = int(by[2][1]), min(int(by[2][1] + altura_marcador * 0.8), roi_h)
    x1, x2 = int(min(by[2][0], by[3][0])), int(max(by[2][0], by[3][0]))
    r = black_mask[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        sinais[i, 0] = 1  # preto embaixo do verde

    y1, y2 = max(int(by[1][1] - altura_marcador * 0.8), 0), int(by[1][1])
    x1, x2 = max(int(min(by[0][0], by[1][0])), 0), max(int(max(by[0][0], by[1][0])), 0)
    r = black_mask[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        sinais[i, 1] = 1  # preto em cima do verde

    bx = box[box[:, 0].argsort()]

    y1, y2 = int(min(bx[0][1], bx[1][1])), int(max(bx[0][1], bx[1][1]))
    x1, x2 = max(int(bx[1][0] - altura_marcador * 0.8), 0), int(bx[1][0])
    r = black_mask[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        sinais[i, 2] = 1  # preto a esquerda do verde

    y1, y2 = int(min(bx[2][1], bx[3][1])), int(max(bx[2][1], bx[3][1]))
    x1, x2 = int(bx[2][0]), min(int(bx[2][0] + altura_marcador * 0.8), roi_w)
    r = black_mask[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * 0.5:
        sinais[i, 3] = 1  # preto a direita do verde

    return sinais


def determine_turn_direction(sinais, roi_h):
    """So preto em 2 lados ADJACENTES do verde sinaliza curva - topo+esquerda
    = vira direita, topo+direita = vira esquerda (cruz do marcador). A linha
    4 (posicao em y) > 95% da altura da ROI = marcador ja quase saindo por
    baixo do quadro = ja passou da fita, ignora (tarde demais pra reagir)."""
    vira_esquerda = vira_direita = ja_passou_esquerda = ja_passou_direita = False
    for row in sinais:
        if row[1] == 1 and row[2] == 1 and row[0] == 0 and row[3] == 0:
            vira_direita = True
            if row[4] > roi_h * 0.95:
                ja_passou_direita = True
        elif row[1] == 1 and row[3] == 1 and row[0] == 0 and row[2] == 0:
            vira_esquerda = True
            if row[4] > roi_h * 0.95:
                ja_passou_esquerda = True
    return vira_esquerda, vira_direita, ja_passou_esquerda, ja_passou_direita


def classificar_verde(contours_grn, black_mask, roi_w, roi_h, area_min):
    sinais = np.zeros((len(contours_grn), 5), dtype=np.int32)
    boxes = []
    idxs = []

    for i, c in enumerate(contours_grn):
        if cv2.contourArea(c) <= area_min:
            continue
        box = cv2.boxPoints(cv2.minAreaRect(c))
        boxes.append(box)
        idxs.append(i)
        sinais = check_black(sinais, i, box, black_mask, roi_w, roi_h)

    n = len(idxs)
    if n == 0:
        return {"n_verdes": 0, "pos": None, "acao": "NADA", "boxes": []}

    linhas = sinais[idxs]
    vira_e, vira_d, passou_e, passou_d = determine_turn_direction(linhas, roi_h)
    centros_x = [float(np.mean(b[:, 0])) for b in boxes]

    if n >= 2:
        # dois marcadores verdes = sinal de retorno (180). So vira se pelo
        # menos um dos dois ainda nao passou da fita.
        if vira_e and vira_d:
            acao = "IGNORAR_JA_PASSOU" if (passou_e and passou_d) else "GIRAR_180"
        else:
            acao = "RETO"
        pos = None
    else:
        pos = "ESQUERDA" if centros_x[0] < roi_w / 2 else "DIREITA"
        if vira_e and not vira_d:
            acao = "IGNORAR_JA_PASSOU" if passou_e else "VIRA_ESQUERDA"
        elif vira_d and not vira_e:
            acao = "IGNORAR_JA_PASSOU" if passou_d else "VIRA_DIREITA"
        else:
            acao = "RETO"

    return {"n_verdes": n, "pos": pos, "acao": acao, "centros_x": centros_x, "boxes": boxes}


def desenhar_debug(roi, contours_grn, area_min, resultado):
    img = roi.copy()
    for c in contours_grn:
        area = cv2.contourArea(c)
        cor = (0, 255, 0) if area > area_min else (0, 165, 255)
        box = cv2.boxPoints(cv2.minAreaRect(c)).astype(int)
        cv2.drawContours(img, [box], 0, cor, 1)
        cv2.putText(img, f"{int(area)}", tuple(box[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.35, cor, 1)
    cv2.putText(img, f"{resultado['acao']} {resultado.get('pos') or ''}", (5, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def main():
    cap = abrir_camera()
    if MOSTRAR_JANELAS:
        criar_trackbars()
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)

    green_min = np.array([GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN])
    green_max = np.array([GREEN_H_MAX, GREEN_S_MAX, GREEN_V_MAX])
    preto_top, preto_bot, area_min = BLACK_MAX_TOP, BLACK_MAX_BOT, AREA_MIN_VERDE

    contador = 0
    contador_fps = 0
    inicio_fps = time.time()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[CAMERA] frame perdido")
                continue

            roi = frame[int(frame.shape[0] * CORTE_TOPO_FRAC):, :]
            roi_h, roi_w = roi.shape[:2]

            if MOSTRAR_JANELAS:
                green_min, green_max, preto_top, preto_bot, area_min = ler_trackbars()

            black_mask, green_mask = mascaras(roi, green_min, green_max, preto_top, preto_bot)
            contours_grn, _ = cv2.findContours(green_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            resultado = classificar_verde(contours_grn, black_mask, roi_w, roi_h, area_min)

            contador += 1
            contador_fps += 1
            fps = None
            if time.time() - inicio_fps >= 1.0:
                fps = contador_fps / (time.time() - inicio_fps)
                contador_fps = 0
                inicio_fps = time.time()

            msg = f"[VERDE] n={resultado['n_verdes']} pos={resultado['pos']} acao={resultado['acao']}"
            if fps:
                msg += f" fps={fps:.0f}"
            print(msg)

            debug_img = desenhar_debug(roi, contours_grn, area_min, resultado)

            if contador % SALVAR_SNAPSHOT_A_CADA == 0:
                cv2.imwrite(os.path.join(SNAPSHOT_DIR, "roi.png"), roi)
                cv2.imwrite(os.path.join(SNAPSHOT_DIR, "overlay.png"), debug_img)
                cv2.imwrite(os.path.join(SNAPSHOT_DIR, "mask_verde.png"), green_mask)
                cv2.imwrite(os.path.join(SNAPSHOT_DIR, "mask_preto.png"), black_mask)

            if MOSTRAR_JANELAS:
                cv2.imshow("Calibrar", debug_img)
                cv2.imshow("Mascara verde", green_mask)
                cv2.imshow("Mascara preto", black_mask)
                tecla = cv2.waitKey(1) & 0xFF
                if tecla == ord('q'):
                    break
                if tecla == ord('s'):
                    print(f"green_min = np.array([{green_min[0]}, {green_min[1]}, {green_min[2]}])")
                    print(f"green_max = np.array([{green_max[0]}, {green_max[1]}, {green_max[2]}])")
                    print(f"BLACK_MAX_TOP = {preto_top}")
                    print(f"BLACK_MAX_BOT = {preto_bot}")
                    print(f"AREA_MIN_VERDE = {area_min}")
    finally:
        cap.release()
        if MOSTRAR_JANELAS:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

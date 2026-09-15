"""
Calibracao ao vivo das mascaras de cor (verde/vermelho) usadas pra achar o
triangulo/canto de entrega em resgate.py (green_min/green_max, red_min1/
red_max1, red_min2/red_max2, AREA_MIN_TRI, AREA_MAX_TRI - mesma logica de
_mascara_cor_tri() + _eh_triangulo()).

Sliders na janela "Sliders" ajustam H/S/V min/max de cada mascara e as
areas minima/maxima. As janelas "Mascara verde"/"Mascara vermelho" mostram
o resultado ao vivo, e "Crop + contornos" mostra o contorno aceito (colorido)
ou rejeitado (cinza) por cima da imagem crua.

'b' liga/desliga um modo que carrega o modelo YOLO de bola (model.onnx,
mesmo do resgate.py) e desenha a caixa da bola detectada tanto no crop
quanto em cima das duas mascaras - serve pra conferir se a cor da bola
(prata/preta) esta vazando pra dentro da mascara do triangulo.

's' pega os valores atuais dos sliders e RE-ESCREVE o bloco de constantes
no topo deste proprio arquivo (mesmo nome/formato usado em resgate.py -
green_min = np.array([...]) etc), pra so copiar e colar direto de la.
Tambem imprime o bloco no terminal.

NAO importa resgate.py de proposito (evitaria abrir GPIO/motor/MPU/sonar a
toa so pra calibrar cor). Motores nunca sao tocados. O modelo YOLO so
carrega se 'b' for apertado.

Precisa de display (janela grafica) - via SSH sem X11/VNC o cv2.namedWindow()
falha. 'q'/ESC na janela ou Ctrl+C no terminal pra sair.
"""

import os
import threading
import time

import cv2
import numpy as np

CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CAMERA_INDEX_FALLBACK = 2
CROP_W, CROP_H = 320, 240

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.onnx")
CONF_THRES = 0.35

# ===================== CONSTANTES (copiar pro resgate.py) =====================
# Bloco reescrito automaticamente ao apertar 's' - mesmo nome/formato usado
# em resgate.py, pra dar pra copiar e colar direto.
green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])
red_min1 = np.array([0, 69, 0])
red_max1 = np.array([8, 255, 255])
red_min2 = np.array([157, 123, 0])
red_max2 = np.array([179, 255, 255])
kernel_tri = np.ones((3, 3), np.uint8)

PROPORCAO_ESPERADA = 30 / 6
TOLERANCIA_PROPORCAO = 0.8
AREA_MIN_TRI = 2500
AREA_MAX_TRI = 200000
# ===================== FIM CONSTANTES =====================

CAMINHO_PROPRIO = os.path.abspath(__file__)
MARCADOR_INICIO = "# ===================== CONSTANTES (copiar pro resgate.py) ====================="
MARCADOR_FIM = "# ===================== FIM CONSTANTES ====================="

modelo_bola = None
carregando_modelo_bola = False
mostrar_bolas = False


def _nada(_valor):
    pass


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(CAMERA_INDEX_FALLBACK, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    return cap


def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()


# ===================== SLIDERS =====================
def criar_sliders():
    cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("G_H_min", "Sliders", int(green_min[0]), 179, _nada)
    cv2.createTrackbar("G_S_min", "Sliders", int(green_min[1]), 255, _nada)
    cv2.createTrackbar("G_V_min", "Sliders", int(green_min[2]), 255, _nada)
    cv2.createTrackbar("G_H_max", "Sliders", int(green_max[0]), 179, _nada)
    cv2.createTrackbar("G_S_max", "Sliders", int(green_max[1]), 255, _nada)
    cv2.createTrackbar("G_V_max", "Sliders", int(green_max[2]), 255, _nada)
    cv2.createTrackbar("R1_H_min", "Sliders", int(red_min1[0]), 179, _nada)
    cv2.createTrackbar("R1_S_min", "Sliders", int(red_min1[1]), 255, _nada)
    cv2.createTrackbar("R1_V_min", "Sliders", int(red_min1[2]), 255, _nada)
    cv2.createTrackbar("R1_H_max", "Sliders", int(red_max1[0]), 179, _nada)
    cv2.createTrackbar("R1_S_max", "Sliders", int(red_max1[1]), 255, _nada)
    cv2.createTrackbar("R1_V_max", "Sliders", int(red_max1[2]), 255, _nada)
    cv2.createTrackbar("R2_H_min", "Sliders", int(red_min2[0]), 179, _nada)
    cv2.createTrackbar("R2_S_min", "Sliders", int(red_min2[1]), 255, _nada)
    cv2.createTrackbar("R2_V_min", "Sliders", int(red_min2[2]), 255, _nada)
    cv2.createTrackbar("R2_H_max", "Sliders", int(red_max2[0]), 179, _nada)
    cv2.createTrackbar("R2_S_max", "Sliders", int(red_max2[1]), 255, _nada)
    cv2.createTrackbar("R2_V_max", "Sliders", int(red_max2[2]), 255, _nada)
    # AREA_MAX_TRI real (resgate.py) vai ate 1000000 - slider limitado a
    # 200000 pra nao ficar minusculo/dificil de ajustar; se precisar de mais,
    # sobe na mao depois de colar no resgate.py.
    cv2.createTrackbar("AREA_MIN_TRI", "Sliders", int(AREA_MIN_TRI), 20000, _nada)
    cv2.createTrackbar("AREA_MAX_TRI", "Sliders", min(int(AREA_MAX_TRI), 200000), 200000, _nada)


def ler_sliders():
    g = lambda nome: cv2.getTrackbarPos(nome, "Sliders")
    gmin = np.array([g("G_H_min"), g("G_S_min"), g("G_V_min")])
    gmax = np.array([g("G_H_max"), g("G_S_max"), g("G_V_max")])
    r1min = np.array([g("R1_H_min"), g("R1_S_min"), g("R1_V_min")])
    r1max = np.array([g("R1_H_max"), g("R1_S_max"), g("R1_V_max")])
    r2min = np.array([g("R2_H_min"), g("R2_S_min"), g("R2_V_min")])
    r2max = np.array([g("R2_H_max"), g("R2_S_max"), g("R2_V_max")])
    area_min = g("AREA_MIN_TRI")
    area_max = g("AREA_MAX_TRI")
    return gmin, gmax, r1min, r1max, r2min, r2max, area_min, area_max


# ===================== MASCARA / TRIANGULO (mesma logica do resgate.py) =====================
def mascara_cor(hsv, cor, gmin, gmax, r1min, r1max, r2min, r2max):
    if cor == "verde":
        mask = cv2.inRange(hsv, gmin, gmax)
    else:
        m1 = cv2.inRange(hsv, r1min, r1max)
        m2 = cv2.inRange(hsv, r2min, r2max)
        mask = cv2.bitwise_or(m1, m2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_tri, iterations=2)
    mask = cv2.dilate(mask, kernel_tri, iterations=2)
    return mask


def avaliar_triangulo(contorno, crop_w, crop_h, area_min_tri, area_max_tri):
    """Mesma logica de _eh_triangulo() do resgate.py - devolve um dict com
    cada criterio checado, pra mostrar tudo no print."""
    area = cv2.contourArea(contorno)
    resultado = {"area": area, "passou_area": area_min_tri <= area <= area_max_tri}
    if not resultado["passou_area"]:
        resultado["vertices"] = None
        resultado["passou_vertices"] = False
        resultado["passou_convex"] = False
        resultado["aceito"] = False
        return resultado

    x, y, w, h_bb = cv2.boundingRect(contorno)
    margem = 2
    encostando_borda = (
        x <= margem or y <= margem or
        x + w >= crop_w - margem or
        y + h_bb >= crop_h - margem
    )

    perimetro = cv2.arcLength(contorno, True)
    aproximado = cv2.approxPolyDP(contorno, 0.06 * perimetro, True)
    resultado["vertices"] = len(aproximado)
    resultado["encostando_borda"] = encostando_borda

    min_vertices = 2 if encostando_borda else 3
    resultado["passou_vertices"] = min_vertices <= len(aproximado) <= 4

    if not resultado["passou_vertices"]:
        resultado["passou_convex"] = False
        resultado["aceito"] = False
        return resultado

    resultado["passou_convex"] = encostando_borda or cv2.isContourConvex(aproximado)
    resultado["aceito"] = resultado["passou_area"] and resultado["passou_vertices"] and resultado["passou_convex"]
    resultado["aproximado"] = aproximado
    return resultado


# ===================== MODO BOLA (YOLO, opcional, tecla 'b') =====================
def _carregar_modelo_bola_thread():
    global modelo_bola, carregando_modelo_bola
    try:
        from ultralytics import YOLO
        print("[BOLA] Carregando modelo YOLO...")
        modelo_bola = YOLO(MODEL_PATH)
        print("[BOLA] Modelo pronto:", modelo_bola.names)
    except Exception as e:
        print(f"[BOLA][ERRO] Falha ao carregar modelo: {e}")
    finally:
        carregando_modelo_bola = False


def alternar_modo_bola():
    global mostrar_bolas, carregando_modelo_bola
    mostrar_bolas = not mostrar_bolas
    if mostrar_bolas and modelo_bola is None and not carregando_modelo_bola:
        carregando_modelo_bola = True
        threading.Thread(target=_carregar_modelo_bola_thread, daemon=True).start()
    print(f"[BOLA] Modo bola {'ligado' if mostrar_bolas else 'desligado'}")


def detectar_bola(crop):
    if modelo_bola is None:
        return None
    results = modelo_bola.predict(crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False)
    boxes = getattr(results[0], "boxes", None)
    if boxes is None or len(boxes) == 0:
        return None
    x1, y1, x2, y2 = map(int, boxes.xyxy[0].tolist())
    cls_id = int(boxes.cls[0].item()) if getattr(boxes, "cls", None) is not None else None
    nome = results[0].names.get(cls_id, str(cls_id)) if cls_id is not None else "?"
    return x1, y1, x2, y2, nome


# ===================== SALVAR (tecla 's') =====================
def _fmt_array(v):
    return f"np.array([{int(v[0])}, {int(v[1])}, {int(v[2])}])"


def formatar_bloco_constantes(gmin, gmax, r1min, r1max, r2min, r2max, area_min, area_max):
    linhas = [
        MARCADOR_INICIO,
        "# Bloco reescrito automaticamente ao apertar 's' - mesmo nome/formato usado",
        "# em resgate.py, pra dar pra copiar e colar direto.",
        f"green_min = {_fmt_array(gmin)}",
        f"green_max = {_fmt_array(gmax)}",
        f"red_min1 = {_fmt_array(r1min)}",
        f"red_max1 = {_fmt_array(r1max)}",
        f"red_min2 = {_fmt_array(r2min)}",
        f"red_max2 = {_fmt_array(r2max)}",
        "kernel_tri = np.ones((3, 3), np.uint8)",
        "",
        "PROPORCAO_ESPERADA = 30 / 6",
        "TOLERANCIA_PROPORCAO = 0.8",
        f"AREA_MIN_TRI = {int(area_min)}",
        f"AREA_MAX_TRI = {int(area_max)}",
        MARCADOR_FIM,
    ]
    return "\n".join(linhas)


def salvar_constantes(bloco_novo):
    try:
        with open(CAMINHO_PROPRIO, "r", encoding="utf-8") as f:
            conteudo = f.read()
        inicio = conteudo.index(MARCADOR_INICIO)
        fim = conteudo.index(MARCADOR_FIM) + len(MARCADOR_FIM)
        conteudo_novo = conteudo[:inicio] + bloco_novo + conteudo[fim:]
        with open(CAMINHO_PROPRIO, "w", encoding="utf-8") as f:
            f.write(conteudo_novo)
        print("[SALVO] Constantes atualizadas no topo deste arquivo - copia pro resgate.py:")
    except Exception as e:
        print(f"[SALVO][ERRO] Nao consegui gravar no arquivo ({e}), mas aqui esta o bloco pra copiar na mao:")
    print(bloco_novo)


# ===================== LOOP PRINCIPAL =====================
def main():
    cap = abrir_camera()
    if not cap.isOpened():
        print("Nao consegui abrir a camera de resgate")
        return

    criar_sliders()

    print("Calibrando - mexe nos sliders na janela 'Sliders' e olha as mascaras/print.")
    print("'s' salva os valores atuais no topo deste arquivo (copia pro resgate.py).")
    print("'b' liga/desliga o modo bola (mostra deteccao YOLO por cima das mascaras).")
    print("'q'/ESC na janela ou Ctrl+C no terminal pra sair.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            crop = center_crop(frame, CROP_W, CROP_H)
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

            gmin, gmax, r1min, r1max, r2min, r2max, area_min, area_max = ler_sliders()

            debug_img = crop.copy()
            linhas_print = []
            mask_verde = mask_vermelho = None

            for cor, cor_bgr in (("verde", (0, 255, 0)), ("vermelho", (0, 0, 255))):
                mask = mascara_cor(hsv, cor, gmin, gmax, r1min, r1max, r2min, r2max)
                if cor == "verde":
                    mask_verde = mask
                else:
                    mask_vermelho = mask

                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if not contornos:
                    linhas_print.append(f"{cor}: nenhum contorno")
                    continue

                maior = max(contornos, key=cv2.contourArea)
                r = avaliar_triangulo(maior, CROP_W, CROP_H, area_min, area_max)

                cor_desenho = cor_bgr if r["aceito"] else (128, 128, 128)
                cv2.drawContours(debug_img, [maior], -1, cor_desenho, 2)
                if r["aceito"]:
                    cv2.drawContours(debug_img, [r["aproximado"]], -1, (0, 255, 255), 2)

                linhas_print.append(
                    f"{cor}: area={r['area']:7.0f} passou_area={r['passou_area']!s:5} "
                    f"vertices={r.get('vertices')} passou_vert={r['passou_vertices']!s:5} "
                    f"passou_convex={r['passou_convex']!s:5}  =>  ACEITO={r['aceito']}"
                )

            mask_verde_bgr = cv2.cvtColor(mask_verde, cv2.COLOR_GRAY2BGR)
            mask_vermelho_bgr = cv2.cvtColor(mask_vermelho, cv2.COLOR_GRAY2BGR)

            info_bola = ""
            if mostrar_bolas:
                if modelo_bola is None:
                    info_bola = "carregando..." if carregando_modelo_bola else "sem modelo"
                else:
                    bbox = detectar_bola(crop)
                    if bbox is not None:
                        x1, y1, x2, y2, nome = bbox
                        info_bola = f"{nome} ({x1},{y1})-({x2},{y2})"
                        for img in (debug_img, mask_verde_bgr, mask_vermelho_bgr):
                            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                        cv2.putText(debug_img, nome, (x1, max(y1 - 5, 0)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    else:
                        info_bola = "nenhuma bola"

            print(
                f"AREA_MIN_TRI={area_min} AREA_MAX_TRI={area_max}  |  " + " || ".join(linhas_print) +
                (f"  |  BOLA: {info_bola}" if mostrar_bolas else "")
            )

            hud = "[s] salvar  [b] modo bola: " + ("ON" if mostrar_bolas else "off")
            cv2.putText(debug_img, hud, (4, CROP_H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            escala = 2
            crop_grande = cv2.resize(debug_img, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            verde_grande = cv2.resize(mask_verde_bgr, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            vermelho_grande = cv2.resize(mask_vermelho_bgr, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Crop + contornos", crop_grande)
            cv2.imshow("Mascara verde", verde_grande)
            cv2.imshow("Mascara vermelho", vermelho_grande)

            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (ord("q"), 27):
                break
            elif tecla == ord("s"):
                bloco = formatar_bloco_constantes(gmin, gmax, r1min, r1max, r2min, r2max, area_min, area_max)
                salvar_constantes(bloco)
            elif tecla == ord("b"):
                alternar_modo_bola()
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

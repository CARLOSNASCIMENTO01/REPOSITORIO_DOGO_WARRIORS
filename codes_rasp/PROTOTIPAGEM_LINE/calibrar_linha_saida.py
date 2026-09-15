"""
Calibracao ao vivo do contorno de linha preta na camera de baixo (a mesma
usada em resgate.py:alinhar_na_linha()/acionar_saida() - _maior_contorno_linha()
e _maior_contorno_linha_h(), variantes vertical e horizontal).

Sliders na janela "Sliders" ajustam limiar/area/tolerancia de cada variante.
"ROI + contornos" mostra os dois eixos (fitLine) achados por cima da imagem
crua - amarelo/ciano = passou a tolerancia de angulo, vermelho/magenta =
nao passou. "Mascara vertical"/"Mascara horizontal" mostram cada mascara
binaria separada (limiares sao diferentes um do outro).

's' pega os valores atuais dos sliders e RE-ESCREVE o bloco de constantes
no topo deste proprio arquivo (mesmo nome/formato usado em resgate.py -
LIMIAR_LINHA = 80 etc), pra so copiar e colar direto de la. Tambem imprime
o bloco no terminal.

NAO importa resgate.py de proposito (evitaria abrir GPIO/motor/MPU/sonar/
YOLO a toa so pra calibrar limiar). Motores nunca sao tocados.

Precisa de display (janela grafica) - via SSH sem X11/VNC o cv2.namedWindow()
falha. 'q'/ESC na janela ou Ctrl+C no terminal pra sair.
"""

import math
import os
import time

import cv2
import numpy as np

CAMERA_INDEX_LINHA = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK = 0
LINHA_W, LINHA_H = 160, 120

# ===================== CONSTANTES (copiar pro resgate.py) =====================
# Bloco reescrito automaticamente ao apertar 's' - mesmo nome/formato usado
# em resgate.py, pra dar pra copiar e colar direto.
LIMIAR_LINHA = 80
AREA_MIN_LINHA = 11000
kernel_linha = np.ones((3, 3), np.uint8)
AREA_MAX_LINHA_FRAC = 0.5
TOLERANCIA_ANGULO_VERTICAL_LINHA = 30

LIMIAR_LINHA_H = 123
AREA_MIN_LINHA_H = 20000
AREA_MAX_LINHA_FRAC_H = 0.81
TOLERANCIA_ANGULO_HORIZONTAL_LINHA = 58
# ===================== FIM CONSTANTES =====================

CAMINHO_PROPRIO = os.path.abspath(__file__)
MARCADOR_INICIO = "# ===================== CONSTANTES (copiar pro resgate.py) ====================="
MARCADOR_FIM = "# ===================== FIM CONSTANTES ====================="


def _nada(_valor):
    pass


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_INDEX_LINHA, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(CAMERA_LINHA_INDEX_FALLBACK, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(3, LINHA_W)
        cap.set(4, LINHA_H)
    return cap


# ===================== SLIDERS =====================
def criar_sliders():
    cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("LIMIAR_LINHA", "Sliders", int(LIMIAR_LINHA), 255, _nada)
    cv2.createTrackbar("AREA_MIN_LINHA", "Sliders", int(AREA_MIN_LINHA), 20000, _nada)
    cv2.createTrackbar("AREA_MAX_FRAC_%", "Sliders", int(AREA_MAX_LINHA_FRAC * 100), 100, _nada)
    cv2.createTrackbar("TOL_VERTICAL_GRAUS", "Sliders", int(TOLERANCIA_ANGULO_VERTICAL_LINHA), 90, _nada)
    cv2.createTrackbar("LIMIAR_LINHA_H", "Sliders", int(LIMIAR_LINHA_H), 255, _nada)
    cv2.createTrackbar("AREA_MIN_LINHA_H", "Sliders", int(AREA_MIN_LINHA_H), 40000, _nada)
    cv2.createTrackbar("AREA_MAX_FRAC_H_%", "Sliders", int(AREA_MAX_LINHA_FRAC_H * 100), 100, _nada)
    cv2.createTrackbar("TOL_HORIZONTAL_GRAUS", "Sliders", int(TOLERANCIA_ANGULO_HORIZONTAL_LINHA), 90, _nada)


def ler_sliders():
    g = lambda nome: cv2.getTrackbarPos(nome, "Sliders")
    limiar = g("LIMIAR_LINHA")
    area_min = g("AREA_MIN_LINHA")
    area_max_frac = g("AREA_MAX_FRAC_%") / 100.0
    tol_vertical = g("TOL_VERTICAL_GRAUS")
    limiar_h = g("LIMIAR_LINHA_H")
    area_min_h = g("AREA_MIN_LINHA_H")
    area_max_frac_h = g("AREA_MAX_FRAC_H_%") / 100.0
    tol_horizontal = g("TOL_HORIZONTAL_GRAUS")
    return limiar, area_min, area_max_frac, tol_vertical, limiar_h, area_min_h, area_max_frac_h, tol_horizontal


# ===================== CONTORNO (mesma logica do resgate.py) =====================
def maior_contorno_linha(roi, limiar, area_min, area_max_frac):
    """Mesma logica de _maior_contorno_linha()/_maior_contorno_linha_h() do
    resgate.py - so parametrizada pra servir pras duas variantes."""
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    mask = cv2.threshold(gray, limiar, 255, cv2.THRESH_BINARY_INV)[1]
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, mask

    maior = max(contours, key=cv2.contourArea)
    area_contorno = cv2.contourArea(maior)
    area_roi = roi.shape[0] * roi.shape[1]

    if area_contorno < area_min or area_contorno >= area_roi * area_max_frac:
        return None, area_contorno, mask
    return maior, area_contorno, mask


def desvio_do_eixo(contorno):
    vx, vy, x0, y0 = cv2.fitLine(contorno, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    desvio_vertical = math.degrees(math.atan2(abs(float(vx)), abs(float(vy))))
    return desvio_vertical, (float(vx), float(vy), float(x0), float(y0))


# ===================== SALVAR (tecla 's') =====================
def formatar_bloco_constantes(limiar, area_min, area_max_frac, tol_vertical,
                               limiar_h, area_min_h, area_max_frac_h, tol_horizontal):
    linhas = [
        MARCADOR_INICIO,
        "# Bloco reescrito automaticamente ao apertar 's' - mesmo nome/formato usado",
        "# em resgate.py, pra dar pra copiar e colar direto.",
        f"LIMIAR_LINHA = {int(limiar)}",
        f"AREA_MIN_LINHA = {int(area_min)}",
        "kernel_linha = np.ones((3, 3), np.uint8)",
        f"AREA_MAX_LINHA_FRAC = {area_max_frac:.2f}",
        f"TOLERANCIA_ANGULO_VERTICAL_LINHA = {int(tol_vertical)}",
        "",
        f"LIMIAR_LINHA_H = {int(limiar_h)}",
        f"AREA_MIN_LINHA_H = {int(area_min_h)}",
        f"AREA_MAX_LINHA_FRAC_H = {area_max_frac_h:.2f}",
        f"TOLERANCIA_ANGULO_HORIZONTAL_LINHA = {int(tol_horizontal)}",
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
        print("Nao consegui abrir a camera de baixo")
        return

    criar_sliders()

    print("Calibrando - mexe nos sliders na janela 'Sliders' e olha as mascaras/print.")
    print("'s' salva os valores atuais no topo deste arquivo (copia pro resgate.py).")
    print("'q'/ESC na janela ou Ctrl+C no terminal pra sair.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            (limiar, area_min, area_max_frac, tol_vertical,
             limiar_h, area_min_h, area_max_frac_h, tol_horizontal) = ler_sliders()

            # mesmo recorte de ROI que _maior_contorno_linha()/_maior_contorno_linha_h() usam
            h_, w_ = frame.shape[:2]
            roi = frame[int(h_ * 0.1):h_, :]
            roi_h, roi_w = roi.shape[:2]

            contorno_v, area_v, mask_v = maior_contorno_linha(roi, limiar, area_min, area_max_frac)
            contorno_h, area_h, mask_h = maior_contorno_linha(roi, limiar_h, area_min_h, area_max_frac_h)

            debug_img = roi.copy()
            comprimento = 60

            desvio_v_txt = "n/a"
            passou_v = False
            if contorno_v is not None:
                cv2.drawContours(debug_img, [contorno_v], -1, (0, 255, 0), 2)
                desvio_vertical, (vx, vy, x0, y0) = desvio_do_eixo(contorno_v)
                passou_v = desvio_vertical <= tol_vertical
                desvio_v_txt = f"{desvio_vertical:5.1f}"
                p1 = (int(x0 - vx * comprimento), int(y0 - vy * comprimento))
                p2 = (int(x0 + vx * comprimento), int(y0 + vy * comprimento))
                cv2.line(debug_img, p1, p2, (0, 255, 255) if passou_v else (0, 0, 255), 2)

            desvio_h_txt = "n/a"
            passou_h = False
            if contorno_h is not None:
                cv2.drawContours(debug_img, [contorno_h], -1, (255, 0, 0), 1)
                desvio_vertical_h, (vx2, vy2, x02, y02) = desvio_do_eixo(contorno_h)
                desvio_horizontal = 90 - desvio_vertical_h
                passou_h = desvio_horizontal <= tol_horizontal
                desvio_h_txt = f"{desvio_horizontal:5.1f}"
                p1h = (int(x02 - vx2 * comprimento), int(y02 - vy2 * comprimento))
                p2h = (int(x02 + vx2 * comprimento), int(y02 + vy2 * comprimento))
                cv2.line(debug_img, p1h, p2h, (255, 255, 0) if passou_h else (255, 0, 255), 2)

            print(
                f"[V] LIMIAR={limiar:3d} AREA_MIN={area_min:6d} FRAC={area_max_frac:.2f} "
                f"TOL={tol_vertical:2d} area={area_v:7.0f} desvio={desvio_v_txt} ACEITO={passou_v}  |  "
                f"[H] LIMIAR={limiar_h:3d} AREA_MIN={area_min_h:6d} FRAC={area_max_frac_h:.2f} "
                f"TOL={tol_horizontal:2d} area={area_h:7.0f} desvio={desvio_h_txt} ACEITO={passou_h}"
            )

            escala = 3
            roi_grande = cv2.resize(debug_img, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            cv2.putText(roi_grande, f"V(amarelo)={passou_v}  H(ciano)={passou_h}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            mask_v_grande = cv2.resize(mask_v, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            mask_h_grande = cv2.resize(mask_h, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("ROI + contornos", roi_grande)
            cv2.imshow("Mascara vertical", mask_v_grande)
            cv2.imshow("Mascara horizontal", mask_h_grande)

            tecla = cv2.waitKey(1) & 0xFF
            if tecla in (ord("q"), 27):
                break
            elif tecla == ord("s"):
                bloco = formatar_bloco_constantes(limiar, area_min, area_max_frac, tol_vertical,
                                                   limiar_h, area_min_h, area_max_frac_h, tol_horizontal)
                salvar_constantes(bloco)
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

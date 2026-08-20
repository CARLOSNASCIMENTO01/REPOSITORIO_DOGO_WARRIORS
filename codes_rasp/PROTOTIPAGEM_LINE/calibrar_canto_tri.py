"""
Ferramenta standalone de calibracao da identificacao de cantos (triangulos)
verde/vermelho usada em resgate.py (green_min/green_max, red_min1/red_max1,
red_min2/red_max2, AREA_MIN_TRI, AREA_MAX_TRI - a mesma logica de
_mascara_cor_tri() + _eh_triangulo()).

Abre a camera de resgate (a mesma usada pra bola/canto, nao a de baixo) +
uma janela com sliders (trackbars) - mexe nos sliders olhando a janela
"Debug" e o print no terminal, aponta a camera pro canto de verdade, e
ajusta ate os numeros ficarem bons. Quando achar o valor certo, copia
manualmente pro topo de resgate.py (esse script nao mexe nele).

NAO importa resgate.py de proposito (evitaria abrir GPIO/motor/MPU/sonar/
YOLO a toa so pra calibrar cor). Motores nunca sao tocados.

Precisa de um display (janela grafica) pra funcionar - se rodar via SSH
sem X11/VNC, o cv2.namedWindow() vai falhar. 'q' na janela ou Ctrl+C no
terminal pra sair.
"""

import math
import time

import cv2
import numpy as np

CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CAMERA_INDEX_FALLBACK = 2
CROP_W, CROP_H = 320, 240

# ===== valores iniciais dos sliders - mesmos de resgate.py =====
GREEN_MIN_INICIAL = (40, 50, 45)
GREEN_MAX_INICIAL = (85, 255, 255)
RED_MIN1_INICIAL = (0, 100, 100)
RED_MAX1_INICIAL = (10, 255, 255)
RED_MIN2_INICIAL = (170, 100, 100)
RED_MAX2_INICIAL = (180, 255, 255)
AREA_MIN_TRI_INICIAL = 2500
AREA_MAX_TRI_INICIAL = 100000  # slider limitado a 100000 (real vai ate 1000000, ajusta na mao se precisar de mais)

kernel_tri = np.ones((3, 3), np.uint8)


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


def criar_sliders():
    cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("G_H_min", "Sliders", GREEN_MIN_INICIAL[0], 179, _nada)
    cv2.createTrackbar("G_S_min", "Sliders", GREEN_MIN_INICIAL[1], 255, _nada)
    cv2.createTrackbar("G_V_min", "Sliders", GREEN_MIN_INICIAL[2], 255, _nada)
    cv2.createTrackbar("G_H_max", "Sliders", GREEN_MAX_INICIAL[0], 179, _nada)
    cv2.createTrackbar("G_S_max", "Sliders", GREEN_MAX_INICIAL[1], 255, _nada)
    cv2.createTrackbar("G_V_max", "Sliders", GREEN_MAX_INICIAL[2], 255, _nada)
    cv2.createTrackbar("R1_H_min", "Sliders", RED_MIN1_INICIAL[0], 179, _nada)
    cv2.createTrackbar("R1_S_min", "Sliders", RED_MIN1_INICIAL[1], 255, _nada)
    cv2.createTrackbar("R1_V_min", "Sliders", RED_MIN1_INICIAL[2], 255, _nada)
    cv2.createTrackbar("R1_H_max", "Sliders", RED_MAX1_INICIAL[0], 179, _nada)
    cv2.createTrackbar("R1_S_max", "Sliders", RED_MAX1_INICIAL[1], 255, _nada)
    cv2.createTrackbar("R1_V_max", "Sliders", RED_MAX1_INICIAL[2], 255, _nada)
    cv2.createTrackbar("R2_H_min", "Sliders", RED_MIN2_INICIAL[0], 179, _nada)
    cv2.createTrackbar("R2_S_min", "Sliders", RED_MIN2_INICIAL[1], 255, _nada)
    cv2.createTrackbar("R2_V_min", "Sliders", RED_MIN2_INICIAL[2], 255, _nada)
    cv2.createTrackbar("R2_H_max", "Sliders", RED_MAX2_INICIAL[0], 179, _nada)
    cv2.createTrackbar("R2_S_max", "Sliders", RED_MAX2_INICIAL[1], 255, _nada)
    cv2.createTrackbar("R2_V_max", "Sliders", RED_MAX2_INICIAL[2], 255, _nada)
    cv2.createTrackbar("AREA_MIN_TRI", "Sliders", AREA_MIN_TRI_INICIAL, 20000, _nada)
    cv2.createTrackbar("AREA_MAX_TRI", "Sliders", AREA_MAX_TRI_INICIAL, 200000, _nada)


def ler_sliders():
    g = lambda nome: cv2.getTrackbarPos(nome, "Sliders")
    green_min = np.array([g("G_H_min"), g("G_S_min"), g("G_V_min")])
    green_max = np.array([g("G_H_max"), g("G_S_max"), g("G_V_max")])
    red_min1 = np.array([g("R1_H_min"), g("R1_S_min"), g("R1_V_min")])
    red_max1 = np.array([g("R1_H_max"), g("R1_S_max"), g("R1_V_max")])
    red_min2 = np.array([g("R2_H_min"), g("R2_S_min"), g("R2_V_min")])
    red_max2 = np.array([g("R2_H_max"), g("R2_S_max"), g("R2_V_max")])
    area_min_tri = g("AREA_MIN_TRI")
    area_max_tri = g("AREA_MAX_TRI")
    return green_min, green_max, red_min1, red_max1, red_min2, red_max2, area_min_tri, area_max_tri


def mascara_cor(hsv, cor, green_min, green_max, red_min1, red_max1, red_min2, red_max2):
    if cor == "verde":
        mask = cv2.inRange(hsv, green_min, green_max)
    else:
        m1 = cv2.inRange(hsv, red_min1, red_max1)
        m2 = cv2.inRange(hsv, red_min2, red_max2)
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


def main():
    cap = abrir_camera()
    if not cap.isOpened():
        print("Nao consegui abrir a camera de resgate")
        return

    criar_sliders()

    print("Calibrando - mexe nos sliders na janela 'Sliders' e olha o print/janelas.")
    print("'q' na janela ou Ctrl+C no terminal pra sair.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            crop = center_crop(frame, CROP_W, CROP_H)
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

            (green_min, green_max, red_min1, red_max1, red_min2, red_max2,
             area_min_tri, area_max_tri) = ler_sliders()

            debug_img = crop.copy()
            linhas_print = []

            for cor, cor_bgr in (("verde", (0, 255, 0)), ("vermelho", (0, 0, 255))):
                mask = mascara_cor(hsv, cor, green_min, green_max, red_min1, red_max1, red_min2, red_max2)
                contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if cor == "verde":
                    mask_verde = mask
                else:
                    mask_vermelho = mask

                if not contornos:
                    linhas_print.append(f"{cor}: nenhum contorno")
                    continue

                maior = max(contornos, key=cv2.contourArea)
                r = avaliar_triangulo(maior, CROP_W, CROP_H, area_min_tri, area_max_tri)

                cor_desenho = cor_bgr if r["aceito"] else (128, 128, 128)
                cv2.drawContours(debug_img, [maior], -1, cor_desenho, 2)
                if r["aceito"]:
                    cv2.drawContours(debug_img, [r["aproximado"]], -1, (0, 255, 255), 2)

                linhas_print.append(
                    f"{cor}: area={r['area']:7.0f} passou_area={r['passou_area']!s:5} "
                    f"vertices={r.get('vertices')} passou_vert={r['passou_vertices']!s:5} "
                    f"passou_convex={r['passou_convex']!s:5}  =>  ACEITO={r['aceito']}"
                )

            print(
                f"AREA_MIN_TRI={area_min_tri} AREA_MAX_TRI={area_max_tri}  |  " + " || ".join(linhas_print)
            )

            escala = 2
            crop_grande = cv2.resize(debug_img, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            verde_grande = cv2.resize(mask_verde, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            vermelho_grande = cv2.resize(mask_vermelho, (CROP_W * escala, CROP_H * escala), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("Crop + contornos aceitos", crop_grande)
            cv2.imshow("Mascara verde", verde_grande)
            cv2.imshow("Mascara vermelho", vermelho_grande)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

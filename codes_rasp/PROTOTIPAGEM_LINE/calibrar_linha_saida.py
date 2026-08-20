"""
Ferramenta standalone de calibracao da deteccao de linha preta usada em
resgate.py:acionar_saida() (LIMIAR_LINHA, AREA_MIN_LINHA, AREA_MAX_LINHA_FRAC,
TOLERANCIA_ANGULO_VERTICAL_LINHA - a mesma logica de _maior_contorno_linha()
+ detectar_linha_vertical_embaixo()).

Abre a camera de baixo + uma janela com sliders (trackbars) - mexe nos
sliders olhando a janela "Debug" e o print no terminal, aponta a camera
pra linha/pro chao, e ajusta ate os numeros ficarem bons. Quando achar o
valor certo, copia manualmente pro topo de resgate.py (esse script nao
mexe nele).

NAO importa resgate.py de proposito (evitaria abrir GPIO/motor/MPU/sonar/
YOLO a toa so pra calibrar limiar). Motores nunca sao tocados.

Precisa de um display (janela grafica) pra funcionar - se rodar via SSH
sem X11/VNC, o cv2.namedWindow() vai falhar. 'q' na janela ou Ctrl+C no
terminal pra sair.
"""

import math
import time

import cv2
import numpy as np

CAMERA_LINHA_PATH = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK = 0
camera_x, camera_y = 160, 120

# ===== valores iniciais dos sliders - mesmos de resgate.py =====
LIMIAR_LINHA_INICIAL = 80
AREA_MIN_LINHA_INICIAL = 11000
AREA_MAX_LINHA_FRAC_INICIAL = 50  # % da ROI - slider em inteiro, vira 0.50
TOLERANCIA_ANGULO_INICIAL = 30  # graus de desvio do vertical

kernel_linha = np.ones((3, 3), np.uint8)


def _nada(_valor):
    pass


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_LINHA_PATH, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(CAMERA_LINHA_INDEX_FALLBACK, cv2.CAP_V4L2)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cap.set(3, camera_x)
        cap.set(4, camera_y)
    return cap


def criar_sliders():
    cv2.namedWindow("Sliders", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("LIMIAR_LINHA", "Sliders", LIMIAR_LINHA_INICIAL, 255, _nada)
    cv2.createTrackbar("AREA_MIN_LINHA", "Sliders", AREA_MIN_LINHA_INICIAL, 20000, _nada)
    cv2.createTrackbar("AREA_MAX_FRAC_%", "Sliders", AREA_MAX_LINHA_FRAC_INICIAL, 100, _nada)
    cv2.createTrackbar("TOLERANCIA_GRAUS", "Sliders", TOLERANCIA_ANGULO_INICIAL, 90, _nada)


def ler_sliders():
    g = lambda nome: cv2.getTrackbarPos(nome, "Sliders")
    limiar = g("LIMIAR_LINHA")
    area_min = g("AREA_MIN_LINHA")
    area_max_frac = g("AREA_MAX_FRAC_%") / 100.0
    tolerancia_graus = g("TOLERANCIA_GRAUS")
    return limiar, area_min, area_max_frac, tolerancia_graus


def main():
    cap = abrir_camera()
    if not cap.isOpened():
        print("Nao consegui abrir a camera de linha")
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

            limiar, area_min, area_max_frac, tolerancia_graus = ler_sliders()

            # mesmo recorte de ROI que _maior_contorno_linha() usa
            h_, w_ = frame.shape[:2]
            roi = frame[int(h_ * 0.1):h_, :]
            roi_h, roi_w = roi.shape[:2]
            area_roi = roi_h * roi_w

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            mask = cv2.threshold(gray, limiar, 255, cv2.THRESH_BINARY_INV)[1]
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_linha)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            debug_img = roi.copy()
            area_contorno = 0.0
            desvio_vertical = None
            passou_area = False
            passou_vertical = False
            aceito = False

            if contours:
                maior = max(contours, key=cv2.contourArea)
                area_contorno = cv2.contourArea(maior)
                cv2.drawContours(debug_img, [maior], -1, (0, 255, 0), 2)

                passou_area = area_min <= area_contorno < area_roi * area_max_frac

                if passou_area:
                    vx, vy, x0, y0 = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
                    desvio_vertical = math.degrees(math.atan2(abs(float(vx)), abs(float(vy))))
                    passou_vertical = desvio_vertical <= tolerancia_graus

                    # desenha o eixo achado pelo fitLine, pra ver visualmente
                    # o quao "vertical" ele julgou o contorno
                    comprimento = 60
                    p1 = (int(x0 - vx * comprimento), int(y0 - vy * comprimento))
                    p2 = (int(x0 + vx * comprimento), int(y0 + vy * comprimento))
                    cor_eixo = (0, 255, 255) if passou_vertical else (0, 0, 255)
                    cv2.line(debug_img, p1, p2, cor_eixo, 2)

                aceito = passou_area and passou_vertical

            desvio_txt = f"{desvio_vertical:5.1f}" if desvio_vertical is not None else "  n/a"
            print(
                f"LIMIAR={limiar:3d} AREA_MIN={area_min:6d} AREA_MAX_FRAC={area_max_frac:.2f} "
                f"TOL_GRAUS={tolerancia_graus:2d}  |  "
                f"contornos={len(contours):2d} maior_area={area_contorno:7.0f}/{area_roi} "
                f"passou_area={passou_area!s:5} desvio_vert={desvio_txt} "
                f"passou_vertical={passou_vertical!s:5}  =>  ACEITO={aceito}"
            )

            escala = 3
            roi_grande = cv2.resize(debug_img, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            mask_grande = cv2.resize(mask, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            cv2.putText(roi_grande, f"ACEITO={aceito}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if aceito else (0, 0, 255), 2)
            cv2.imshow("ROI + contorno + eixo", roi_grande)
            cv2.imshow("Mascara preta", mask_grande)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

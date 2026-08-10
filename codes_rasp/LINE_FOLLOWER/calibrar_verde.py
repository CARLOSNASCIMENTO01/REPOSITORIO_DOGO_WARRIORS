"""
Ferramenta standalone de calibracao dos limiares de cor do marcador verde
(green_min/green_max/black_min/black_max_top/black_max_bot/MIN_GREEN_AREA).

Abre a camera de linha + uma janela com sliders (trackbars) - mexe nos
sliders olhando a janela "Mascaras" e o print no terminal, aponta a camera
pro marcador de verdade, e ajusta ate os numeros ficarem bons. Quando
achar o valor certo, copia manualmente pro topo de seguimento_de_linha.py
(esse script nao mexe nele).

NAO importa config.py de proposito (evitaria abrir GPIO/motor/MPU/sonar a
toa so pra calibrar cor) e NAO chama nenhuma funcao do seguidor. Motores
nunca sao tocados.

Precisa de um display (janela grafica) pra funcionar - se rodar via SSH
sem X11/VNC, o cv2.namedWindow() vai falhar. 'q' na janela ou Ctrl+C no
terminal pra sair.
"""

import time

import cv2
import numpy as np

CAMERA_LINHA_PATH = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK = 0
camera_x, camera_y = 160, 120

# ===== valores iniciais dos sliders - mesmos de seguimento_de_linha.py =====
GREEN_MIN_INICIAL = (17, 48, 39)
GREEN_MAX_INICIAL = (89, 255, 255)
PRETO_MAX_TOP_INICIAL = 78
PRETO_MAX_BOT_INICIAL = 91
MIN_GREEN_AREA_INICIAL = 2000

kernal = np.ones((3, 3), np.uint8)


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
    cv2.createTrackbar("G_H_min", "Sliders", GREEN_MIN_INICIAL[0], 179, _nada)
    cv2.createTrackbar("G_S_min", "Sliders", GREEN_MIN_INICIAL[1], 255, _nada)
    cv2.createTrackbar("G_V_min", "Sliders", GREEN_MIN_INICIAL[2], 255, _nada)
    cv2.createTrackbar("G_H_max", "Sliders", GREEN_MAX_INICIAL[0], 179, _nada)
    cv2.createTrackbar("G_S_max", "Sliders", GREEN_MAX_INICIAL[1], 255, _nada)
    cv2.createTrackbar("G_V_max", "Sliders", GREEN_MAX_INICIAL[2], 255, _nada)
    cv2.createTrackbar("Preto_max_top", "Sliders", PRETO_MAX_TOP_INICIAL, 255, _nada)
    cv2.createTrackbar("Preto_max_bot", "Sliders", PRETO_MAX_BOT_INICIAL, 255, _nada)
    cv2.createTrackbar("MIN_GREEN_AREA", "Sliders", MIN_GREEN_AREA_INICIAL, 10000, _nada)


def ler_sliders():
    g = lambda nome: cv2.getTrackbarPos(nome, "Sliders")
    green_min = np.array([g("G_H_min"), g("G_S_min"), g("G_V_min")])
    green_max = np.array([g("G_H_max"), g("G_S_max"), g("G_V_max")])
    preto_max_top = g("Preto_max_top")
    preto_max_bot = g("Preto_max_bot")
    min_green_area = g("MIN_GREEN_AREA")
    return green_min, green_max, preto_max_top, preto_max_bot, min_green_area


def main():
    cap = abrir_camera()
    if not cap.isOpened():
        print("Nao consegui abrir a camera de linha")
        return

    criar_sliders()
    black_min = np.array([0, 0, 0])

    print("Calibrando - mexe nos sliders na janela 'Sliders' e olha o print.")
    print("'q' na janela ou Ctrl+C no terminal pra sair.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            green_min, green_max, preto_max_top, preto_max_bot, min_green_area = ler_sliders()
            black_max_top = np.array([preto_max_top] * 3)
            black_max_bot = np.array([preto_max_bot] * 3)

            # mesmo recorte de ROI que visao()/detectar_verde() usam
            h_, w_ = frame.shape[:2]
            roi = frame[int(h_ * 0.1):h_, :]
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
            maior_area = max((cv2.contourArea(c) for c in contours_grn), default=0)
            pixels_verde = cv2.countNonZero(green_image)
            pixels_preto = cv2.countNonZero(black_image)
            passou_area_minima = maior_area > min_green_area

            print(
                f"green_min={green_min.tolist()} green_max={green_max.tolist()} "
                f"preto_top={preto_max_top} preto_bot={preto_max_bot} "
                f"MIN_GREEN_AREA={min_green_area}  |  "
                f"verde_px={pixels_verde:5d} preto_px={pixels_preto:5d} "
                f"contornos={len(contours_grn)} maior_contorno={maior_area:7.0f} "
                f"passou={passou_area_minima}"
            )

            escala = 3
            roi_grande = cv2.resize(roi, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            verde_grande = cv2.resize(green_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            preto_grande = cv2.resize(black_image, (roi_w * escala, roi_h * escala), interpolation=cv2.INTER_NEAREST)
            cv2.imshow("ROI (camera)", roi_grande)
            cv2.imshow("Mascara verde", verde_grande)
            cv2.imshow("Mascara preta", preto_grande)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

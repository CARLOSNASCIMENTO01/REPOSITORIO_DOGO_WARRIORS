"""
Mostra a imagem ao vivo da camera de linha, sem mascara nem processamento
nenhum - so pra ver o que a camera esta enxergando de verdade.

NAO importa config.py/resgate.py de proposito. Precisa de display (janela
grafica) pra funcionar. 'q' na janela ou Ctrl+C no terminal pra sair.
"""

import cv2

CAMERA_LINHA_PATH = "/dev/v4l/by-id/usb-Jieli_Technology_USB_Composite_Device-video-index0"
CAMERA_LINHA_INDEX_FALLBACK = 0


def abrir_camera():
    cap = cv2.VideoCapture(CAMERA_LINHA_PATH, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(CAMERA_LINHA_INDEX_FALLBACK, cv2.CAP_V4L2)
    return cap


def main():
    cap = abrir_camera()
    if not cap.isOpened():
        print("Nao consegui abrir a camera de linha")
        return

    print("Mostrando ao vivo - 'q' na janela ou Ctrl+C no terminal pra sair.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            cv2.imshow("Camera de linha - ao vivo", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        print("\nEncerrado.")
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

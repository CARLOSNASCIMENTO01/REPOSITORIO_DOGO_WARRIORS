"""
Identificacao de vitimas (YOLO ONNX numa 2a camera, so aberta durante o
resgate) e o handoff pro resgate.py quando o seguidor acha a sala (gap
continuo por tempo demais).

OBS: identificar_vitimas() nao e chamada por area() hoje - a docstring
promete isso mas o codigo nunca fez essa chamada. Preservado exatamente
como estava.
"""

import os
import sys
import time

import cv2

import config

# "from ultralytics import YOLO" NAO fica aqui no topo - sozinho ja custa
# varios segundos (importa o torch inteiro) e travava o boot do seguidor
# antes de qualquer print, mesmo o modelo de vitima nunca sendo usado nessa
# execucao. O import so acontece dentro de carregar_modelo_vitima(), que so
# roda se identificar_vitimas() for chamada de verdade.

CAMERA_VITIMA_PATH = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CAMERA_VITIMA_INDEX_FALLBACK = 2

VICTIM_MODEL_PATH   = "modelon.onnx"
VICTIM_CROP_W, VICTIM_CROP_H = 320, 240
VICTIM_CONF_THRES   = 0.35
VICTIM_FRAMES_WARMUP = 3
VICTIM_FRAMES_ANALISE = 30
DEBUG_VITIMA_PREVIEW = True

modelo_vitima = None


def carregar_modelo_vitima():
    global modelo_vitima
    try:
        from ultralytics import YOLO
    except ImportError:
        print("[VITIMA][AVISO] ultralytics nao instalado, deteccao de vitima desativada")
        return
    try:
        print("[VITIMA] Carregando modelo YOLO...")
        modelo_vitima = YOLO(VICTIM_MODEL_PATH)
        print("[VITIMA] Modelo pronto")
    except Exception as e:
        print("[VITIMA][ERRO] Falha ao carregar modelo:", e)
        modelo_vitima = None


def _abrir_camera_vitima():
    fonte_principal = CAMERA_VITIMA_PATH or CAMERA_VITIMA_INDEX_FALLBACK
    c = config.abrir_camera_generica(fonte_principal)
    if c is not None:
        return c

    print("[VITIMA][AVISO] Camera de vitima nao respondeu, tentando resetar USB...")
    config.resetar_usb_camera(fonte_principal)

    c = config.abrir_camera_generica(fonte_principal)
    if c is not None:
        return c

    if CAMERA_VITIMA_PATH:
        return config.abrir_camera_generica(CAMERA_VITIMA_INDEX_FALLBACK)
    return None


def identificar_vitimas():
    """Abre a 2a camera, roda a inferencia YOLO em alguns frames e fecha.
    Retorna lista de deteccoes: [{"classe": str, "conf": float}, ...]
    Chamada dentro de area() assim que o robo entra na area de resgate."""
    if modelo_vitima is None:
        carregar_modelo_vitima()  # carregamento preguicoso, so na primeira chamada

    if modelo_vitima is None:
        print("[VITIMA] Modelo indisponivel, pulando deteccao")
        return []

    cap_vitima = _abrir_camera_vitima()
    if cap_vitima is None:
        print("[VITIMA][ERRO] Nao foi possivel abrir a camera de vitima")
        return []

    deteccoes = {}
    try:
        for _ in range(VICTIM_FRAMES_WARMUP):
            cap_vitima.read()

        for _ in range(VICTIM_FRAMES_ANALISE):
            config.checar_parada()
            ret, frame = cap_vitima.read()
            if not ret or frame is None:
                continue

            fh, fw = frame.shape[:2]
            start_x = max(fw // 2 - VICTIM_CROP_W // 2, 0)
            start_y = max(fh // 2 - VICTIM_CROP_H // 2, 0)
            cropped = frame[start_y:start_y + VICTIM_CROP_H, start_x:start_x + VICTIM_CROP_W]

            inicio = time.perf_counter()

            results = modelo_vitima.predict(
                cropped,
                imgsz=(VICTIM_CROP_W, VICTIM_CROP_H),
                conf=VICTIM_CONF_THRES,
                verbose=False
            )

            tempo = time.perf_counter() - inicio
            fps_modelo = 1.0 / tempo

            results = modelo_vitima.predict(
                cropped, imgsz=(VICTIM_CROP_W, VICTIM_CROP_H),
                conf=VICTIM_CONF_THRES, verbose=False
            )

            for box in results[0].boxes:
                classe = results[0].names[int(box.cls[0])]
                conf = float(box.conf[0])
                if classe not in deteccoes or conf > deteccoes[classe]:
                    deteccoes[classe] = conf

            if DEBUG_VITIMA_PREVIEW:
                annotated = results[0].plot()

                cv2.putText(
                    annotated,
                    f"YOLO FPS: {fps_modelo:.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )
                cv2.imshow("Deteccao Vitima", annotated)
                cv2.waitKey(1)

    finally:
        cap_vitima.release()
        if DEBUG_VITIMA_PREVIEW:
            cv2.destroyWindow("Deteccao Vitima")

    lista = [{"classe": c, "conf": v} for c, v in deteccoes.items()]
    if lista:
        print(f"[VITIMA] Detectado: {lista}")
    else:
        print("[VITIMA] Nenhuma vitima detectada")
    return lista


def area(contours):
    print("estou na area")
    os.execv(sys.executable, [
        sys.executable,
        "/home/pi/ROBO_DOGO_2026/codes_rasp/PROTOTIPAGEM_LINE/resgate.py",
        f"--entregas={config.ENTREGAS_HERDADAS}",
    ])

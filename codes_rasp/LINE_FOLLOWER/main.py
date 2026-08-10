"""
ROBO_DOGO_2026 - Robo seguidor de linha (OBR)
Ponto de entrada: so boot + o loop principal (while True que chama as
funcoes importadas dos modulos por assunto). A logica de cada assunto mora
no modulo dele:

  config.py              - hardware: estado/parada, GPIO, motores Yahboom,
                            servos, botao, giroscopio (BNO055), sonar
                            (serial do Arduino) e camera de linha (cap)
  movimentos_bases.py    - guinada2 / guinada_pid
  seguimento_de_linha.py - visao(), controle() (PID), deteccao de verde
  obstaculos_trajeto.py  - desvio() por sonar
  resgate_de_vitimas.py  - identificar_vitimas() + area() (handoff pro
                            resgate.py)

Mudancas de comportamento pedidas (nao sao bugfix, sao features novas):
  1) Botao vira TOGGLE: aperta -> para tudo imediatamente (mesmo dentro de
     guinada/desvio/resgate). Aperta de novo -> reseta estado e reroda o
     loop principal do zero.
  2) Parada imediata em qualquer ponto do codigo via excecao RoboParado,
     checada a cada iteracao dos while True internos.
  3) Boot mais rapido: camera, serial e calibracao do BNO055 rodam em paralelo.
  4) Se a porta serial nao aparecer na hora do boot (arduino ainda enumerando
     na USB), fica tentando em vez de matar o processo.
  5) Erros inesperados no loop principal nao derrubam o programa: sao
     logados e o robo tenta rodar de novo (retry). Isso completa o
     Restart=always do systemd (ver robo_dogo.service).
  6) Reset de USB via software (ioctl USBDEVFS_RESET) quando uma camera
     falha ao abrir ou trava - nao precisa tirar/colocar o cabo na mao.
"""

import sys
import time
from threading import Thread

import cv2
import lgpio

import config
import obstaculos_trajeto
import resgate_de_vitimas
import seguimento_de_linha
from seguimento_de_linha import controle, verde

# =========================================================================
# CONFIG especifico do loop principal (nao pertence a nenhum modulo de
# assunto - so o "cimento" que liga eles)
# =========================================================================
VEL_GAP = 15
TEMPO_GAP_SALA_RESGATE = 2.0  # segundos em gap continuo pra considerar sala de resgate
FALHAS_CAMERA_ANTES_DE_RESETAR = 15  # ~ alguns segundos de frames perdidos seguidos

# Desafio surpresa (obstaculo na largada) - True liga a espera, False desliga
# sem precisar apagar o codigo. So espera de verdade se realmente tiver algo
# na frente do sonar (ver obstaculos_trajeto.esperar_obstaculo_frontal_sumir).
AGUARDAR_OBSTACULO_SURPRESA_NO_BOOT = True

_serial_thread_iniciada = [False]


# =========================================================================
# LOOP PRINCIPAL
# =========================================================================
def loop_principal():
    seguimento_de_linha.resetar_estado()
    config.resetar_giroscopio()
    config.parar()
    if not _serial_thread_iniciada[0]:
        Thread(target=config.serial_thread, daemon=True).start()
    _serial_thread_iniciada[0] = True

    falhas_camera_seguidas = 0

    while True:
        config.checar_parada()

        ret, frame = config.cap.read()
        if not ret:
            falhas_camera_seguidas += 1
            if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_RESETAR:
                print("[Camera] Linha de camera parou de responder, resetando USB...")
                try:
                    config.cap.release()
                except Exception:
                    pass
                config.resetar_usb_camera(
                    config.CAMERA_LINHA_PATH or config.CAMERA_LINHA_INDEX_FALLBACK
                )
                config.iniciar_camera()
                falhas_camera_seguidas = 0
            continue
        falhas_camera_seguidas = 0

        if seguimento_de_linha.detectar_fita_vermelha(frame):
            continue

        cx, contours, roi, mask = seguimento_de_linha.visao(frame)

        if seguimento_de_linha.detectar_travamento(roi):
            print("[TRAVAMENTO] Tentando se soltar...")
            while True:
                config.checar_parada()
                config.mover(100, 100)
                time.sleep(0.3)
                ret, frame_trav = config.cap.read()
                if ret:
                    _, _, roi_trav, _ = seguimento_de_linha.visao(frame_trav)
                    if not seguimento_de_linha.detectar_travamento(roi_trav):
                        print("[TRAVAMENTO] Solto!")
                        config.parar()
                        seguimento_de_linha.ultimo_erro = 0
                        seguimento_de_linha.ultimo_tempo = time.time()
                break

        verde(roi)
        ajuste, contours = controle(cx, contours, roi)
        obstaculos_trajeto.desvio()

        if seguimento_de_linha.gap_debug(contours):
            inicio_gap = time.time()
            while True:
                config.checar_parada()
                ret, frame = config.cap.read()
                if not ret:
                    continue
                cx, contours, roi, mask = seguimento_de_linha.visao(frame)
                # Sem isso, um marcador verde que tambem derruba o contorno
                # preto principal (gap_debug=True) ficava sem nenhuma chance
                # de ser detectado/reagido enquanto esse loop conta o tempo
                # de gap - o verde() precisa continuar rodando junto.
                verde(roi)
                if not seguimento_de_linha.gap_debug(contours):
                    print("[GAP] Saiu do gap, voltando a linha")
                    break
                if time.time() - inicio_gap > TEMPO_GAP_SALA_RESGATE:
                    config.parar()
                    print(f"[SALA DE RESGATE] Gap continuo por mais de {TEMPO_GAP_SALA_RESGATE:.0f}s, entrando no resgate")
                    time.sleep(3.0)
                    resgate_de_vitimas.area(contours)
                    break
                config.mover(VEL_GAP, VEL_GAP)

        if cv2.waitKey(1) == 27:
            break


# =========================================================================
# BOOT
# =========================================================================
def iniciar_hardware():
    t_mpu = Thread(target=config.iniciar_mpu)
    t_mpu.start()
    # modelo de vitima NAO carrega no boot do seguidor - so quando
    # identificar_vitimas() for chamada de verdade (carregamento preguicoso)

    config.handshake_serial()   # serial primeiro, igual no codigo original
    if not _serial_thread_iniciada[0]:
        # Precisa estar rodando ANTES do botao ser apertado, senao
        # distancia_frente()/esperar_obstaculo_frontal_sumir() leem o valor
        # padrao (9999, "sem obstaculo") em vez do sonar de verdade - o
        # obstaculo surpresa da largada nunca seria detectado.
        Thread(target=config.serial_thread, daemon=True).start()
        _serial_thread_iniciada[0] = True
    config.iniciar_camera()     # so a camera de LINHA e aberta no boot

    t_mpu.join()
    print("[BOOT] Hardware pronto")


if __name__ == "__main__":
    t0 = time.time()
    iniciar_hardware()
    print(f"[BOOT] Tempo de inicializacao: {time.time() - t0:.1f}s")

    # Servos so se mexem AQUI, depois do giroscopio calibrado (dentro de
    # iniciar_hardware() -> t_mpu.join()) - se mexessem antes, a propria
    # vibracao do servo durante a calibracao (_calibrar() em config.py)
    # podia jogar o offset de "parado" pra um valor errado.
    config.robo_ativo.set()
    config.angulo(150)
    config.angulo(0)
    if "--auto" in sys.argv:
        # ja comeca ativo (vindo do resgate) - servo pesado levantado.
        # mover_servo_direto() aqui de proposito, nao mover_servo(): logo no
        # boot o codigo nao faz ideia de onde o servo pesado fisicamente esta
        # (fica onde foi deixado na execucao anterior), mas mover_servo() monta
        # uma rampa suave assumindo que ele comeca em ANGULO_INICIAL=90 - se a
        # posicao real for outra (o normal), o servo da um salto/correcao logo
        # no inicio da rampa em vez de simplesmente ir direto pro alvo.
        config.mover_servo_direto(config.PINO_PESADO, config.SERVO_PESADO_ANGULO_ATIVO)
    else:
        # esperando o botao pra comecar - servo pesado abaixado, indicando parado
        config.mover_servo_direto(config.PINO_PESADO, config.SERVO_PESADO_ANGULO_PARADO)
        config.robo_ativo.clear()

    Thread(target=config.botao_thread, daemon=True).start()

    obstaculo_surpresa_checado = False

    try:
        while True:
            try:
                config.robo_ativo.wait()
                if AGUARDAR_OBSTACULO_SURPRESA_NO_BOOT and not obstaculo_surpresa_checado:
                    obstaculos_trajeto.esperar_obstaculo_frontal_sumir()
                    obstaculo_surpresa_checado = True
                loop_principal()

            except KeyboardInterrupt:
                print("Finalizado")
                break

            except OSError as e:
                print("[MPU] Erro I2C:", repr(e))
                time.sleep(0.01)
                continue

            except Exception as e:
                print("[ERRO] loop principal:", repr(e))
                config.parar()

    finally:
        config.parar()
        lgpio.gpiochip_close(config.h)
        try:
            config.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

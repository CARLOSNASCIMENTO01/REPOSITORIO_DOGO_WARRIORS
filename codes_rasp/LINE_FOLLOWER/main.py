
import os as _os, sys as _sys
_VENV_PY = "/home/pi/ROBO_DOGO_2026/env_dogo/bin/python3"
if "env_dogo" not in _sys.prefix and not _os.environ.get("ROBO_REEXEC") and _os.path.exists(_VENV_PY):
    _os.environ["ROBO_REEXEC"] = "1"
    _os.execv(_VENV_PY, [_VENV_PY] + _sys.argv)

import os
import sys
import time
from threading import Thread

import cv2
import lgpio

import config
import movimentos_bases
import obstaculos_trajeto
import resgate_de_vitimas
import seguimento_de_linha
import video_debug
from seguimento_de_linha import controle, verde

# =========================================================================
# CONFIG especifico do loop principal
# =========================================================================
VIDEO_DEBUG = True  # False - mostra a camera da linha no navegador (porta 8081), so codifica se tiver pagina aberta
VEL_GAP = 15 #20
TEMPO_GAP_SALA_RESGATE = 2.0
TEMPO_RECUO_EXTRA_GAP = 0.2 #0.3 - depois de ver linha de novo recuando, anda mais esse tanto (vista mais limpa da ponta)
TIMEOUT_RECUO_GAP = 2.0
FRAMES_CONFIRMACAO_SAIU_GAP = 3
TEMPO_MIN_ENTRE_CORRECOES_GAP = 2.5 #1.2 - segundos minimos entre uma correcao de gap e a proxima, mesmo ja tendo visto linha de novo
MAX_CORRECOES_POR_GAP = 2 #3 - quantas vezes no maximo corrige dentro do mesmo gap antes de parar de tentar
VEL_GIRO_CORRECAO_GAP = 12 #16, 10 - correcao fina do angulo ao recuar pra fora do gap
FALHAS_CAMERA_ANTES_DE_RESETAR = 15

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
    correcao_gap_feita = False
    tempo_ultima_correcao_gap = 0.0
    tentativas_correcao_gap = 0

    while True:
        config.checar_parada()

        ret, frame = config.cap.read()
        if not ret:
            falhas_camera_seguidas += 1
            if falhas_camera_seguidas >= FALHAS_CAMERA_ANTES_DE_RESETAR:
                print("[Camera] Linha de camera parou de responder, reiniciando processo...")
                config.reiniciar_processo()
            continue
        falhas_camera_seguidas = 0

        if seguimento_de_linha.detectar_fita_vermelha(frame):
            continue

        cx, contours, roi, mask_video = seguimento_de_linha.visao(frame)
        if VIDEO_DEBUG:
            video_debug.publicar(roi, mask_video, cx)

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
            if seguimento_de_linha.DEBUG_ANGULO_GAP:
                config.parar()
                while True:
                    config.checar_parada()
                    ret, frame = config.cap.read()
                    if not ret:
                        continue
                    _, contours_dbg, _, _ = seguimento_de_linha.visao(frame)
                    angulo_gap = seguimento_de_linha.angulo_linha_gap(contours_dbg)
                    print(f"[DEBUG GAP] angulo={angulo_gap}")
                    time.sleep(0.2)

            # DESATIVADO
            '''
            # Ao entrar no gap, recua ate ver a linha de novo (em vez de so
            # esperar/andar pra frente) - anda mais um pouco pra ter uma
            # vista mais limpa da ponta da linha, calcula o angulo dela e
            # corrige com uma guinada bem lenta antes de seguir: angulo > 0
            # gira esquerda, angulo < 0 gira direita (ver
            # seguimento_de_linha.angulo_linha_gap). So corrige de novo se
            # (a) ja tiver saido do gap e visto linha de novo desde a ultima
            # vez (correcao_gap_feita==False), (b) ja tiver passado
            # TEMPO_MIN_ENTRE_CORRECOES_GAP desde a ultima correcao, e (c)
            # ainda nao tiver batido MAX_CORRECOES_POR_GAP tentativas nesse
            # mesmo gap (tentativas_correcao_gap reseta so quando sai do gap
            # de verdade, junto com correcao_gap_feita).
            pode_corrigir_gap = (
                not correcao_gap_feita
                and (time.time() - tempo_ultima_correcao_gap) > TEMPO_MIN_ENTRE_CORRECOES_GAP
                and tentativas_correcao_gap < MAX_CORRECOES_POR_GAP
            )
            if pode_corrigir_gap:
                correcao_gap_feita = True
                tempo_ultima_correcao_gap = time.time()
                tentativas_correcao_gap += 1
                print(f"[GAP] Recuando ate ver a linha de novo... (tentativa {tentativas_correcao_gap}/{MAX_CORRECOES_POR_GAP})")
                inicio_recuo = time.time()
                achou_recuando = False
                while time.time() - inicio_recuo < TIMEOUT_RECUO_GAP:
                    config.checar_parada()
                    config.mover(-VEL_GAP, -VEL_GAP)
                    ret, frame = config.cap.read()
                    if not ret:
                        continue
                    _, contours_recuo, _, _ = seguimento_de_linha.visao(frame)
                    if not seguimento_de_linha.gap_debug(contours_recuo):
                        achou_recuando = True
                        break
                config.parar()

                if not achou_recuando:
                    # Nao achou a linha recuando dentro do tempo limite - esse
                    # gap pode ser de verdade (entrada do resgate, sem linha
                    # nenhuma pra tras) - desiste da correcao e cai no
                    # comportamento antigo (crawl pra frente / timeout).
                    print(f"[GAP] Nao achou a linha recuando em {TIMEOUT_RECUO_GAP:.1f}s, desistindo da correcao")
                else:
                    config.mover(-VEL_GAP, -VEL_GAP)
                    time.sleep(TEMPO_RECUO_EXTRA_GAP)
                    config.parar()

                    ret, frame = config.cap.read()
                    if ret:
                        _, contours_ang, _, _ = seguimento_de_linha.visao(frame)
                        angulo_gap = seguimento_de_linha.angulo_linha_gap(contours_ang)
                        if angulo_gap is not None:
                            print(f"[GAP] Corrigindo angulo={angulo_gap:.1f} graus")
                            if angulo_gap > 0:
                                movimentos_bases.guinada2('E', abs(angulo_gap), VEL_GIRO_CORRECAO_GAP)
                            elif angulo_gap < 0:
                                movimentos_bases.guinada2('D', abs(angulo_gap), VEL_GIRO_CORRECAO_GAP)
                        else:
                            print("[GAP] Angulo nao confiavel (contorno pouco alongado), sem corrigir")

                        # Compensa o recuo extra (TEMPO_RECUO_EXTRA_GAP) que
                        # deu antes de medir o angulo - volta pra frente o
                        # DOBRO desse tanto (pedido explicito, senao o robo
                        # ficava mais pra tras do que quando entrou no gap).
                        config.mover(VEL_GAP, VEL_GAP)
                        time.sleep(TEMPO_RECUO_EXTRA_GAP * 2)
                        config.parar()
            '''

            # Caiu no gap subindo rampa/gangorra = topo da gangorra (perdeu a linha).
            if seguimento_de_linha.contador_gangorra == 2:
                seguimento_de_linha.teve_gap_na_subida = True
            inicio_gap = time.time()
            frames_fora_gap_seguidos = 0
            while True:
                config.checar_parada()
                ret, frame = config.cap.read()
                if not ret:
                    continue
                cx, contours, roi, _ = seguimento_de_linha.visao(frame)
                verde(roi)
                if not seguimento_de_linha.gap_debug(contours):
                    frames_fora_gap_seguidos += 1
                    if frames_fora_gap_seguidos >= FRAMES_CONFIRMACAO_SAIU_GAP:
                        print("[GAP] Saiu do gap, voltando a linha")
                        correcao_gap_feita = False
                        tentativas_correcao_gap = 0
                        break
                    continue
                frames_fora_gap_seguidos = 0
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

    config.handshake_serial()
    if not _serial_thread_iniciada[0]:
        Thread(target=config.serial_thread, daemon=True).start()
        _serial_thread_iniciada[0] = True
    config.iniciar_camera()     # so a camera de LINHA e aberta no boot

    t_mpu.join()
    print("[BOOT] Hardware pronto")


if __name__ == "__main__":
    # Iniciado do zero (sem vir do resgate/saida) = rodada nova: esquece que a
    # saida ja tinha comecado, senao ele pularia o resgate.
    if not any(a == "--auto" or a.startswith("--entregas=") for a in sys.argv[1:]):
        try:
            os.remove(config.ARQUIVO_SAIDA_INICIADA)
        except FileNotFoundError:
            pass

    t0 = time.time()
    iniciar_hardware()
    print(f"[BOOT] Tempo de inicializacao: {time.time() - t0:.1f}s")

    # Servos so se mexem AQUI
    config.robo_ativo.set()
    if "--auto" in sys.argv:
        config.mover_servo_direto(config.PINO_PESADO, config.SERVO_PESADO_ANGULO_ATIVO)
    else:
        config.mover_servo_direto(config.PINO_PESADO, config.SERVO_PESADO_ANGULO_PARADO)
        config.robo_ativo.clear()

    Thread(target=config.botao_thread, daemon=True).start()
    if VIDEO_DEBUG:
        video_debug.iniciar()

    try:
        while True:
            try:
                config.robo_ativo.wait()
                loop_principal()

            except KeyboardInterrupt:
                print("Finalizado")
                break

            except OSError as e:
                print("[MPU] Erro I2C:", repr(e))
                time.sleep(0.01)
                continue

            except config.RoboParado:
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

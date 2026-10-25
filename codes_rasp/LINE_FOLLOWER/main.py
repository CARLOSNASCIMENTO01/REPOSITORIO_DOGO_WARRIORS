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
import movimentos_bases
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
TEMPO_RECUO_EXTRA_GAP = 0.2 #0.3 - depois de ver linha de novo recuando, anda mais esse tanto (vista mais limpa da ponta)
TIMEOUT_RECUO_GAP = 2.0  # se recuar isso tudo sem achar a linha, desiste da correcao (gap pode ser de verdade, sem linha pra tras)
FRAMES_CONFIRMACAO_SAIU_GAP = 3  # frames seguidos vendo linha antes de considerar "saiu do gap de verdade" (libera corrigir de novo)
TEMPO_MIN_ENTRE_CORRECOES_GAP = 2.5 #1.2 - segundos minimos entre uma correcao de gap e a proxima, mesmo ja tendo visto linha de novo
MAX_CORRECOES_POR_GAP = 2 #3 - quantas vezes no maximo corrige dentro do mesmo gap antes de parar de tentar
VEL_GIRO_CORRECAO_GAP = 12 #16, 10 - correcao fina do angulo ao recuar pra fora do gap
FALHAS_CAMERA_ANTES_DE_RESETAR = 15  # ~ alguns segundos de frames perdidos seguidos

# Grava o percurso inteiro com a camera da frente/resgate, desde o boot, em
# video_rounds/ - True liga, False desliga sem precisar apagar codigo. So
# cobre o trecho do SEGUIDOR - resgate.py tem a mesma chave (GRAVAR_PERCURSO)
# no topo dele proprio, independente (ele nao importa main.py/config.py).
GRAVAR_PERCURSO = False

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
            if seguimento_de_linha.DEBUG_ANGULO_GAP:
                # So debug (DEBUG_ANGULO_GAP=True no seguimento_de_linha.py) -
                # pausa TUDO (motores parados, nao entra no resgate) e so
                # fica printando o angulo da ponta da linha no gap, pra
                # calibrar olhando ao vivo. Sai so pelo botao (RoboParado).
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

            # DESATIVADO (alinhador de gap: recuava ate ver a linha, media o
            # angulo da ponta e corrigia com guinada antes de seguir) - so
            # comentado, nao apagado, pra poder reativar depois se precisar.
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

            inicio_gap = time.time()
            frames_fora_gap_seguidos = 0
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
                    frames_fora_gap_seguidos += 1
                    # So conta como "saiu do gap de verdade" (e libera
                    # correcao_gap_feita pra poder alinhar de novo no
                    # proximo) depois de FRAMES_CONFIRMACAO_SAIU_GAP frames
                    # seguidos vendo linha - 1 frame so podia ser um flash/
                    # ruido, e resetava a flag cedo demais, deixando a
                    # correcao repetir varias vezes no mesmo gap.
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
                    # Fecha o clipe do trecho do seguidor antes do execv pro
                    # resgate.py - ele abre o proprio clipe (mesmo round,
                    # sufixo "resgate") por cima da camera que ja usa pra bola/
                    # triangulo, sem precisar de uma segunda captura aqui.
                    config.parar_gravacao_percurso()
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
        # distancia_frente() le o valor padrao (9999, "sem obstaculo") em
        # vez do sonar de verdade assim que o robo comecar a andar.
        Thread(target=config.serial_thread, daemon=True).start()
        _serial_thread_iniciada[0] = True
    config.iniciar_camera()     # so a camera de LINHA e aberta no boot
    config.iniciar_gravacao_percurso(GRAVAR_PERCURSO, sufixo="seguidor")

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
    # Garra (SG90/PINO_SG90) NAO e mais mexida por aqui (nem por nenhum outro
    # modulo alem de resgate.py) - esquema fisico do copo mudou (0 agora e
    # 180, "frouxo" e 130 a velocidade fixa 20) e so resgate.py conhece os
    # angulos novos. Mandar os angulos antigos (150/90/0) daqui quebraria o
    # mecanismo novo.
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

    norte_calibrado = False

    try:
        while True:
            try:
                config.robo_ativo.wait()
                if not norte_calibrado:
                    # So fixa o "norte" (0 graus) da guinada absoluta AQUI,
                    # depois do robo_ativo.wait() liberar - ou seja, so
                    # quando o robo de fato vai comecar a andar (botao ja
                    # apertado, ou --auto ja ativo vindo do resgate). Fazer
                    # isso logo no boot (antes do botao) calibrava com o
                    # robo ainda sendo posicionado/alinhado pelo operador -
                    # se ficasse torto nesse instante, TODAS as guinadas
                    # absolutas do resto da sessao (mesmo depois do resgate,
                    # via /tmp) saiam erradas. guinada_absoluta()/
                    # guinada_absoluta_alinhada() (movimentos_bases.py) usam
                    # essa referencia fixa pra girar em relacao ao campo.
                    movimentos_bases.calibrar_norte_absoluto()
                    norte_calibrado = True
                loop_principal()

            except KeyboardInterrupt:
                print("Finalizado")
                break

            except OSError as e:
                print("[MPU] Erro I2C:", repr(e))
                time.sleep(0.01)
                continue

            except config.RoboParado:
                # Botao pausou o robo - nao e erro. So espera aqui
                # (robo_ativo.wait() la em cima) ate apertar de novo, e
                # loop_principal() recomeca do zero quando retomar.
                continue

            except Exception as e:
                print("[ERRO] loop principal:", repr(e))
                config.parar()

    finally:
        config.parar()
        config.parar_gravacao_percurso()
        lgpio.gpiochip_close(config.h)
        try:
            config.cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()

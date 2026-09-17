"""
Pagina em branco pra programar a saida nova, dentro do proprio seguidor de
linha (nao mexe no resgate.py). Ver REFERENCIA_HARDWARE.py pro que da pra
usar de config.py/movimentos_bases.py/seguimento_de_linha.py/
obstaculos_trajeto.py.

Pra ligar isso no robo de verdade: chama minha_saida() de dentro do
loop_principal() do main.py, no ponto que fizer sentido pro seu fluxo
(mesmo jeito que obstaculos_trajeto.desvio() ja e chamado hoje).
"""

import threading
import time

import cv2

import config
import movimentos_bases
import seguimento_de_linha


def minha_saida():
    config.mover_servo(config.PINO_PESADO, 130)

    # try/finally - a placa Yahboom mantem o ultimo PWM mandado ate alguem
    # avisar o contrario. Sem isso, um Ctrl+C (ou RoboParado, ou qualquer
    # erro) no meio do while deixa o robo andando sozinho com o ultimo
    # comando, porque config.parar() nunca seria chamado.
    try:
        while True:
            config.checar_parada()

            if not config.sonar_valido():
                time.sleep(0.02)
                continue

            frente = config.distancia_frente()
            esquerda = config.distancia_esquerda()

            

            ret, frame_baixo = config.cap.read()
            if ret:
                _, _, _, mask = seguimento_de_linha.visao(frame_baixo)
                pixels_pretos = cv2.countNonZero(mask)
                if pixels_pretos > 20000:
                    print("muito pixel preto")
            else:
                pixels_pretos = 0
            print(f"pixels_pretos={pixels_pretos}")

            """if frente < 9 :
                config.mover(19, -19)
                time.sleep(1.0)
            else:
                config.mover(15, 20)
            time.sleep(0.02)
            if frente > 50 and esquerda > 50:
                if pixels_pretos > 20000:
                    print("muito pixel preto")
                else :
                    config.mover(19, -19)
                    time.sleep(1.5)
            else:
                config.mover(15, 20)
            time.sleep(0.02)"""
            config.mover(16, 16)
            print(config.angulo_z())
            if frente < 9 :
                config.resetar_giroscopio()
                angulozero = config.angulo_z()
                config.mover(30, 30)
                time.sleep(1.5)
                if (config.angulo_z() - angulozero) < -20:
                    movimentos_bases.guinada2('D', 86, 15)
                    config.mover(15, 15)
                    time.sleep(0.9)
                    movimentos_bases.guinada2('D', 45, 15)
                else:
                    config.mover(-15, -15)
                    time.sleep(0.7)
                    movimentos_bases.guinada2('D', 84, 15)
            
            if 18000 < pixels_pretos < 26000:
                config.mover(-15,-15)
                time.sleep(0.2)
                inicio_frente = time.time()
                while pixels_pretos <= 28000 and time.time() - inicio_frente <= 0.9:
                    config.checar_parada()
                    config.mover(15, 15)
                    ret2, frame2 = config.cap.read()
                    if ret2:
                        _, _, _, mask2 = seguimento_de_linha.visao(frame2)
                        pixels_pretos = cv2.countNonZero(mask2)
                    time.sleep(0.02)
                config.parar()
                print("pixels_pretos=", pixels_pretos)
                if pixels_pretos > 27000:
                    print("saiiiiiii")
                    time.sleep(2.0)
                    return 0
                else:
                    config.mover(-15,-15)
                    time.sleep(2.5)
                    movimentos_bases.guinada2('D', 86, 15)
            if esquerda > 40:
                config.mover(-2,-2)
                time.sleep(0.4)
                config.mover(15,15)
                time.sleep(0.4)
                movimentos_bases.guinada2('E', 86, 15)
                
                inicio_frente = time.time()
                while pixels_pretos <= 28000 and time.time() - inicio_frente <= 1.9:
                    config.checar_parada()
                    config.mover(15, 15)
                    ret2, frame2 = config.cap.read()
                    if ret2:
                        _, _, _, mask2 = seguimento_de_linha.visao(frame2)
                        pixels_pretos = cv2.countNonZero(mask2)
                    time.sleep(0.02)
                config.parar()
                print("pixels_pretos=", pixels_pretos)
                if pixels_pretos > 29000:
                    print("saiiiiiii")
                    time.sleep(2.0)
                    return 0
                else:
                    
                    config.mover(-15,-15)
                    time.sleep(1.4)
                    movimentos_bases.guinada2('D', 86, 15)
                    if config.distancia_frente() < 15:
                        config.mover(-15,-15)
                        time.sleep(0.5)
                        movimentos_bases.guinada2('D', 86, 15)
                        
                    config.mover(15,15)
                    time.sleep(1.3)
                
    finally:
        config.parar()


if __name__ == "__main__":
    # Rodando esse arquivo sozinho (sem passar pelo main.py), ninguem liga a
    # serial do Arduino - sem isso config.sonar_valido() fica True so por uma
    # fracao de segundo (o valor inicial do "ultimo_recebimento" la no import
    # de config.py) e depois vira False pra sempre, travando o loop no
    # "if not sonar_valido(): continue" sem nunca mais chamar mover().
    config.handshake_serial()
    threading.Thread(target=config.serial_thread, daemon=True).start()

    # Mesma coisa pro giroscopio (BNO055) - iniciar_mpu() reseta o chip e
    # coloca ele no modo de fusao certo. Sem isso, angulo_z()/guinada2()
    # (que ja e usada la embaixo) nao tem garantia de funcionar direito.
    config.iniciar_mpu()

    # Mesma coisa pra camera de linha (config.cap) - so existe depois de
    # iniciar_camera(), senao config.cap fica None e o .read() quebra.
    config.iniciar_camera()

    # Ninguem fica lendo o botao fisico por conta propria tambem - precisa
    # subir a botao_thread() aqui, igual main.py ja faz.
    threading.Thread(target=config.botao_thread, daemon=True).start()

    # E ninguem liga config.robo_ativo sozinho (isso normalmente e o botao
    # quem faz, na primeira ativacao) - sem isso a primeira checar_parada()
    # ja derruba com RoboParado. Aqui e teste de bancada, entao liga direto;
    # o primeiro toque no botao a partir daqui vai PAUSAR (ja que comeca
    # ativo), igual no robo de verdade.
    config.robo_ativo.set()

    try:
        while True:
            config.robo_ativo.wait()
            try:
                minha_saida()
            except config.RoboParado:
                print("[BOTAO] Pausado - aperte de novo pra retomar.")
                continue
    except KeyboardInterrupt:
        print("\nEncerrado (Ctrl+C) - motores parados.")
        config.parar()

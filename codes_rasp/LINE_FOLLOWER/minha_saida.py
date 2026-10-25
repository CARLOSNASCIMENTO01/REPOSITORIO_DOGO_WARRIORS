
import os
import sys
import threading
import time

import cv2
import lgpio

import config
import movimentos_bases
import seguimento_de_linha


def minha_saida():
    config.mover_servo(config.PINO_PESADO, 130)

    try:
        #movimentos_bases.guinada2('D', 80, 15)
        while True:
            config.mover(-15,15)
            if config.distancia_frente() < 80:

                break
                
        config.mover(28,30)
        time.sleep(4.0)
        config.mover(-15,-15)
        time.sleep(0.5)
        movimentos_bases.guinada2('D', 85, 15)
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

            
            config.mover(16, 16)
            print(config.angulo_z())
            if frente < 9 :
                print("paredeeee")
                config.resetar_giroscopio()
                angulozero = config.angulo_z()
                config.mover(30, 30)
                time.sleep(1.5)
                if (config.angulo_z() - angulozero) < -20:
                    movimentos_bases.guinada2('D', 86, 15)
                    config.mover(19, 19)
                    time.sleep(0.9)
                    movimentos_bases.guinada2('D', 40, 15)
                else:
                    config.mover(-15, -15)
                    time.sleep(0.7)
                    movimentos_bases.guinada2('D', 84, 15)
            
            if 24000 < pixels_pretos < 28000:
                print("saida frente")
                config.mover(-15,-15)
                time.sleep(0.2)
                inicio_frente = time.time()
                while pixels_pretos <= 29000 and time.time() - inicio_frente <= 0.9:
                    config.checar_parada()
                    config.mover(15, 15)
                    ret2, frame2 = config.cap.read()
                    if ret2:
                        _, _, _, mask2 = seguimento_de_linha.visao(frame2)
                        pixels_pretos = cv2.countNonZero(mask2)
                    time.sleep(0.02)
                config.parar()
                print("pixels_pretos=", pixels_pretos)
                if pixels_pretos >= 29000:
                    print("saiiiiiii")
                    config.mover(15,15)
                    time.sleep(0.9)
                    movimentos_bases.guinada2('E', 20, 15)
                    time.sleep(2.0)
                    config.parar()
                    config.cap.release()
                    lgpio.gpiochip_close(config.h)
                    os.execv(sys.executable, [
                        sys.executable,
                        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                        "--auto",
                        f"--entregas={config.ENTREGAS_HERDADAS}",
                    ])

                else:
                    config.mover(-15,-15)
                    time.sleep(2.0)
                    movimentos_bases.guinada2('D', 86, 15)
                    config.mover(18,18)
                    time.sleep(1.2)
            if esquerda > 40:
                print("buracooooo")
                config.mover(-2,-2)
                time.sleep(0.4)
                config.mover(15,15)
                time.sleep(0.4)
                movimentos_bases.guinada2('E', 86, 15)
                
                inicio_frente = time.time()
                while pixels_pretos <= 30000 and time.time() - inicio_frente <= 2.1:
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
                    config.mover(15,15)
                    time.sleep(0.9)
                    movimentos_bases.guinada2('E', 20, 15)
                    time.sleep(2.0)
                    config.parar()
                    config.cap.release()
                    lgpio.gpiochip_close(config.h)
                    os.execv(sys.executable, [
                        sys.executable,
                        "/home/pi/ROBO_DOGO_2026/codes_rasp/LINE_FOLLOWER/main.py",
                        "--auto",
                        f"--entregas={config.ENTREGAS_HERDADAS}",
                    ])

                else:

                    config.mover(-15,-15)
                    time.sleep(1.4)
                    movimentos_bases.guinada2('D', 86, 15)
                    if config.distancia_frente() < 15:
                        config.mover(-15,-15)
                        time.sleep(0.3)
                        movimentos_bases.guinada2('D', 86, 15)
                        
                    config.mover(18,18)
                    time.sleep(1.2)
                
    finally:
        config.parar()


if __name__ == "__main__":
    
    config.handshake_serial()
    threading.Thread(target=config.serial_thread, daemon=True).start()


    config.iniciar_mpu()

    config.iniciar_camera()

    threading.Thread(target=config.botao_thread, daemon=True).start()

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

"""
teste_velocidade_reta.py - robo anda reto com velocidade fixa 100 por tempo
indeterminado, so isso - sem guinada, sem visao, sem camera, nada mais.
Standalone (importa config.py direto), nao faz parte do robo.service.

RODAR SOZINHO: pare o robo.service antes (mesmo GPIO) -
"sudo systemctl stop robo.service". Precisa apertar o botao pra comecar
(nao sai andando sozinho) e de novo pra pausar. Ctrl+C encerra.
"""

import threading

import config

# VELOCIDADE acima de 0: as quatro rodas andam nela, iguais.
# VELOCIDADE = 0: cada roda anda na sua velocidade abaixo.
VELOCIDADE = 0

VEL_DIANT_ESQ = -30
VEL_TRAS_ESQ = -30
VEL_DIANT_DIR = -30
VEL_TRAS_DIR = -30


def mover_4_rodas(diant_esq, tras_esq, diant_dir, tras_dir):
    """Cada roda com sua velocidade. Mesmo mapeamento e sinal do
    config.aplicar_motor(): M1=diant dir, M2=tras esq, M3=tras dir,
    M4=diant esq, sinal invertido. Sem o corte do VEL_MAX (so o limite da
    placa, +-3600 de PWM ~ 51 de velocidade)."""
    e = config.ESCALA_PWM_YAHBOOM
    config._enviar_pwm_yahboom(-diant_dir * e, -tras_esq * e, -tras_dir * e, -diant_esq * e)


def main():
    threading.Thread(target=config.botao_thread, daemon=True).start()
    if VELOCIDADE > 0:
        rodas = (VELOCIDADE, VELOCIDADE, VELOCIDADE, VELOCIDADE)
    else:
        rodas = (VEL_DIANT_ESQ, VEL_TRAS_ESQ, VEL_DIANT_DIR, VEL_TRAS_DIR)
    print(f"[RODAS] diant_esq={rodas[0]} tras_esq={rodas[1]} diant_dir={rodas[2]} tras_dir={rodas[3]}")
    print("[BOTAO] Aperte o botao pra comecar.")

    while True:
        config.robo_ativo.wait()
        try:
            while True:
                config.checar_parada()
                mover_4_rodas(*rodas)
        except config.RoboParado:
            print("[BOTAO] Pausado - aperte de novo pra retomar.")
            continue


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nEncerrado (Ctrl+C).")
    finally:
        config.parar()

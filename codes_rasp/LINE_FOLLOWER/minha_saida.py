"""
Pagina em branco pra programar a saida nova, dentro do proprio seguidor de
linha (nao mexe no resgate.py). Ver REFERENCIA_HARDWARE.py pro que da pra
usar de config.py/movimentos_bases.py/seguimento_de_linha.py/
obstaculos_trajeto.py.

Pra ligar isso no robo de verdade: chama minha_saida() de dentro do
loop_principal() do main.py, no ponto que fizer sentido pro seu fluxo
(mesmo jeito que obstaculos_trajeto.desvio() ja e chamado hoje).
"""

import time

import config
import movimentos_bases
import seguimento_de_linha


def minha_saida():
    while True:
        config.checar_parada()

        if not config.sonar_valido():
            time.sleep(0.02)
            continue

        frente = config.distancia_frente()
        esquerda = config.distancia_esquerda()

        config.mover(15,18)
        if frente < 15:
            config.mover(15,-15)
        time.sleep(0.02)


if __name__ == "__main__":
    minha_saida()

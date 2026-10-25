"""
Guinadas (giro no proprio eixo controlado pelo giroscopio). guinada2 e a
guinada "classica" usada em quase todo canto (verde, desvio); guinada_pid
e a versao fechada em PID sobre o angulo, usada na saida do resgate.
"""

import os
import time

import numpy as np

import config


def guinada2(LADO, GRAUS, VELOCIDADE):
    config.resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    if LADO == 'E':
        while True:
            config.checar_parada()
            config.mover(-VELOCIDADE, VELOCIDADE)
            if config.angulo_z() <= -Graus:
                config.mover(2, -2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    else:
        while True:
            config.checar_parada()
            config.mover(VELOCIDADE, -VELOCIDADE)
            if config.angulo_z() >= Graus:
                config.mover(-2, 2)
                time.sleep(0.09)
                break
            time.sleep(0.005)
    # Zera de novo ao terminar - sem isso o angulo_z() ficava preso no valor
    # da guinada (ex: 90) durante todo o segmento reto seguinte, ate a
    # PROXIMA guinada zerar - confundia debug e deixava lixo acumulado
    # exposto pra qualquer outro codigo que va ler angulo_z() nesse meio tempo.
    config.resetar_giroscopio()


def guinada_desacelerada(LADO, GRAUS, VELOCIDADE_MAX=20, VELOCIDADE_MIN=8):
    """Guinada relativa (igual guinada2) mas com velocidade variavel - comeca
    rapido (VELOCIDADE_MAX) e vai desacelerando linearmente conforme o
    angulo restante diminui, terminando bem mais devagar (VELOCIDADE_MIN)
    perto do alvo. Sem PID - so proporcional ao quanto ainda falta girar,
    pra chegar mais suave no ponto certo sem passar por inercia."""
    config.resetar_giroscopio()
    time.sleep(0.1)
    Graus = abs(GRAUS)
    while True:
        config.checar_parada()
        atual = abs(config.angulo_z())
        restante = max(Graus - atual, 0)
        proporcao = min(restante / Graus, 1.0) if Graus > 0 else 0.0
        velocidade = int(VELOCIDADE_MIN + (VELOCIDADE_MAX - VELOCIDADE_MIN) * proporcao)

        if LADO == 'E':
            config.mover(-velocidade, velocidade)
            if config.angulo_z() <= -Graus:
                config.mover(2, -2)
                time.sleep(0.09)
                break
        else:
            config.mover(velocidade, -velocidade)
            if config.angulo_z() >= Graus:
                config.mover(-2, 2)
                time.sleep(0.09)
                break
        time.sleep(0.005)
    config.resetar_giroscopio()


def guinada_pid(LADO, GRAUS, VEL_MAX_GIRO=100, VEL_MIN_GIRO=29, ZONA_CHEGADA=10):
    config.parar()
    Kp_g, Ki_g, Kd_g = 0.09, 0.002, 0.001
    config.resetar_giroscopio()
    time.sleep(0.03)

    alvo = abs(GRAUS)
    integral = 0.0
    erro_ant = alvo
    t_ant = time.time()

    while True:
        config.checar_parada()
        atual = abs(config.angulo_z())
        erro = alvo - atual
        agora = time.time()
        dt = max(agora - t_ant, 0.001)

        if erro <= ZONA_CHEGADA:
            if LADO == 'D':
                config.mover(-15, 15)
            else:
                config.mover(15, -15)
            time.sleep(0.09)
            config.parar()
            config.resetar_giroscopio()
            break

        integral += erro * dt
        integral = np.clip(integral, -20, 20)
        derivada = (erro - erro_ant) / dt
        derivada = np.clip(derivada, -300, 300)

        ajuste = Kp_g * erro + Ki_g * integral - Kd_g * derivada
        ajuste = np.clip(ajuste, 0, VEL_MAX_GIRO - VEL_MIN_GIRO)
        vel = int(VEL_MIN_GIRO + ajuste)

        if LADO == 'D':
            config.mover(vel, -vel)
        else:
            config.mover(-vel, vel)

        erro_ant = erro
        t_ant = agora
        time.sleep(0.005)


# ===================== GUINADA ABSOLUTA (independente do resetar_giroscopio) =====================
# guinada2()/guinada_pid() acima giram um tanto RELATIVO a onde o robo esta
# apontando agora (zeram a referencia deles a cada chamada via
# config.resetar_giroscopio()). guinada_absoluta() e diferente: o "zero"/
# norte e fixado UMA VEZ SO, antes do robo comecar a andar, e nunca muda
# depois disso - mesmo que guinada2()/config.resetar_giroscopio() fiquem
# zerando a referencia RELATIVA deles o tempo todo. Sao dois contadores
# separados (um le so o heading bruto com offset proprio, o outro usa
# config._heading_offset) - um nunca mexe no outro. O angulo aqui e
# absoluto (0-360 desde esse norte fixo, da a volta e reinicia em 0 quando
# passa de 360), nao "quanto girar a partir de agora".
_norte_absoluto = None
_ultimo_bruto_absoluto = [0.0]

# O robo troca de processo o tempo todo via os.execv() (main.py <-> resgate.py
# <-> minha_saida.py) - isso apaga qualquer variavel em memoria, entao sem
# salvar em arquivo o norte seria recalibrado do zero a cada troca (com o
# robo ja torto no meio do resgate/saida, nao mais apontando pra onde
# apontava no boot de verdade). /tmp sobrevive a troca de processo mas
# some sozinho quando a Raspberry Pi reinicia - assim uma rodada nova
# (Pi religada, robo reposicionado na pista) recalibra do zero naturalmente.
_ARQUIVO_NORTE_ABSOLUTO = "/tmp/robo_dogo_norte_absoluto.txt"


def calibrar_norte_absoluto(forcar=False):
    """Chamar no boot, ANTES do robo comecar a se mover - fixa a leitura
    atual do giroscopio como o 0 graus/"norte" do campo pra
    guinada_absoluta()/angulo_absoluto(), e salva num arquivo em /tmp. Se
    esse arquivo ja existir (viemos de um os.execv, nao de um boot fresco
    de verdade), CARREGA o norte salvo em vez de recalibrar - assim toda a
    cadeia de scripts (main.py -> resgate.py -> minha_saida.py -> main.py
    --auto ...) mantem o MESMO norte do primeiro boot. Passe forcar=True
    pra ignorar o arquivo e recalibrar de verdade mesmo se ele existir."""
    global _norte_absoluto

    if not forcar and os.path.exists(_ARQUIVO_NORTE_ABSOLUTO):
        try:
            with open(_ARQUIVO_NORTE_ABSOLUTO) as f:
                _norte_absoluto = float(f.read().strip())
            print(f"[GUINADA ABSOLUTA] Norte carregado de {_ARQUIVO_NORTE_ABSOLUTO}: {_norte_absoluto:.1f} graus")
            return
        except Exception as e:
            print(f"[GUINADA ABSOLUTA] Falha ao carregar norte salvo ({e}), calibrando de novo")

    _norte_absoluto = config._ler_heading_bruto()
    try:
        with open(_ARQUIVO_NORTE_ABSOLUTO, "w") as f:
            f.write(str(_norte_absoluto))
    except Exception as e:
        print(f"[GUINADA ABSOLUTA] Falha ao salvar norte em {_ARQUIVO_NORTE_ABSOLUTO} ({e})")
    print(f"[GUINADA ABSOLUTA] Norte calibrado em {_norte_absoluto:.1f} graus (heading bruto)")


def angulo_absoluto():
    """Heading absoluto desde o norte fixado em calibrar_norte_absoluto(),
    sempre normalizado 0-360 (da a volta e reinicia em 0, nunca acumula
    alem disso). Independente de config.resetar_giroscopio() - guinada2()/
    guinada_pid() podem zerar a referencia RELATIVA deles a vontade que
    esse numero aqui nao muda por causa disso. Se o norte ainda nao foi
    calibrado, calibra na hora usando a leitura atual."""
    global _norte_absoluto
    if _norte_absoluto is None:
        calibrar_norte_absoluto()
    try:
        bruto = config._ler_heading_bruto()
        _ultimo_bruto_absoluto[0] = bruto
    except Exception as e:
        print(f"[GUINADA ABSOLUTA] Falha ao ler heading ({e}), mantendo ultimo valor")
        bruto = _ultimo_bruto_absoluto[0]
    return (bruto - _norte_absoluto) % 360


def guinada_absoluta(LADO, GRAU_ALVO, VELOCIDADE=15):
    """Gira ate o heading ABSOLUTO (angulo_absoluto(), fixo desde o norte
    calibrado no boot) bater GRAU_ALVO - diferente de guinada2(), que gira
    um tanto relativo a onde o robo esta apontando agora. LADO escolhe o
    sentido do giro ('E' = esquerda, qualquer outra coisa = direita); como
    o alvo e absoluto, virar no sentido "errado" pro caso da volta inteira
    por fora em vez do caminho curto - escolha o LADO certo pro seu alvo.

    Por baixo calcula UMA VEZ SO quantos graus RELATIVOS faltam pra chegar
    no alvo absoluto nesse sentido, e delega pra guinada2() - reaproveita a
    mesma logica de giro/freio ja testada. guinada2() decide quando parar
    checando config.angulo_z(), que e SEMPRE normalizado entre -180 e 180
    - pedir mais de ~180 graus numa chamada so faz a condicao de parada
    dela nunca bater (o normalizado nunca chega la) e o robo gira pra
    sempre. Por isso, se o total passar de LIMITE_POR_CHAMADA, quebra em
    pedacos MENORES calculados de uma vez so no inicio (nao reconfere nem
    corrige sobra depois de cada pedaco - qualquer erro residual de
    guinada2 e ignorado de proposito, sem giro extra pra tentar acertar).
    O resetar_giroscopio() que guinada2() chama por dentro so mexe na
    referencia RELATIVA dela (config._heading_offset), nunca no norte
    absoluto fixado aqui."""
    LIMITE_POR_CHAMADA = 170
    alvo = GRAU_ALVO % 360
    atual = angulo_absoluto()
    if LADO == 'E':
        total = (atual - alvo) % 360
    else:
        total = (alvo - atual) % 360

    print(f"[GUINADA ABSOLUTA] atual={atual:.0f} alvo={alvo:.0f} lado={LADO} -> gira {total:.0f} graus no total")
    restante = total
    while restante > 0:
        passo = min(restante, LIMITE_POR_CHAMADA)
        guinada2(LADO, passo, VELOCIDADE)
        restante -= passo


def guinada_absoluta_alinhada(LADO, GRAU_APROX, VELOCIDADE=15):
    """Guinada "alinhada a grade do campo": o robo vai desviando um pouco
    conforme anda na pista, entao girar exatamente GRAU_APROX graus
    RELATIVOS (guinada2 normal) pode deixar ele apontando torto em vez de
    numa direcao "redonda" do campo (multiplo de GRAU_APROX, ex: 0/90/180/
    270 pra GRAU_APROX=90). Essa versao, em vez de girar exatamente
    GRAU_APROX, escolhe entre os dois multiplos de GRAU_APROX mais
    proximos na grade absoluta (fixada em calibrar_norte_absoluto()) na
    direcao pedida, e vai pro que exigir um giro mais PROXIMO de
    GRAU_APROX - pode ser um pouco menos ou um pouco mais que o pedido.
    Assim o robo sempre termina alinhado a grade do campo, nunca torto.

    Exemplo: parado em 260 graus absolutos, guinada_absoluta_alinhada('D',
    90, ...): o proximo multiplo de 90 pra direita e 270 (so 10 graus de
    giro), o de depois e 360/0 (100 graus de giro). 100 fica mais perto de
    90 do que 10, entao gira os 100 graus ate o 0 - nao os 10 ate o 270."""
    atual = angulo_absoluto()
    if LADO == 'E':
        r = atual % GRAU_APROX
    else:
        r = GRAU_APROX - (atual % GRAU_APROX)
    if r == 0:
        r = GRAU_APROX

    escolhido = r if abs(r - GRAU_APROX) <= abs(r + GRAU_APROX - GRAU_APROX) else r + GRAU_APROX
    alvo = (atual - escolhido) % 360 if LADO == 'E' else (atual + escolhido) % 360

    print(f"[GUINADA ALINHADA] atual={atual:.0f} pedido~{GRAU_APROX} lado={LADO} "
          f"-> giro mais proximo = {escolhido:.0f} graus (alvo absoluto {alvo:.0f})")
    guinada_absoluta(LADO, alvo, VELOCIDADE)

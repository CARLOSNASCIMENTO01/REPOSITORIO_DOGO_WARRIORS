"""
Teste de bancada dos servos (garra SG90 + servo pesado). Script isolado,
nao importa config.py - so abre o GPIO desses 2 pinos, sem motor/camera/
serial/giroscopio.

mover_servo()/mover_servo_direto() copiados de config.py pra bater exatamente
com o robo de verdade (mesma rampa, mesmo corte de PWM ao assentar).
"""

import time

import lgpio

# ===================== CONFIG =====================
PINO_SG90 = 5     # garra
PINO_PESADO = 20  # servo pesado

ANGULO_INICIAL = 90  # mesmo valor de config.py - ponto de partida assumido pra rampa

# "Zero" de cada servo - esquema novo do copo (garra): 0 antigo agora e 180,
# "frouxo" (solta a bola) agora e 160 a velocidade 100 (so resgate.py mexe
# nesses angulos no robo de verdade - aqui e so bancada. Calibrado ao vivo,
# ja foi 130/20).
ZERO_GARRA = 180
ZERO_PESADO = 90
FROUXO_GARRA = 160
VELOCIDADE_FROUXO_GARRA = 100

VELOCIDADE_PADRAO = 60

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL,
    PINO_PESADO: ANGULO_INICIAL,
}


# ===================== SERVOS =====================
def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass


def mover_servo(pino, angulo_alvo, velocidade=VELOCIDADE_PADRAO, pulso_min=500, pulso_max=2500, delay_base=0.02):
    """Rampa suave ate angulo_alvo, igual config.py - velocidade 1-100
    (menor = mais lento/mais suave)."""
    angulo_alvo = max(0, min(180, angulo_alvo))
    velocidade = max(1, min(100, velocidade))

    atual = _angulo_atual.get(pino, ANGULO_INICIAL)
    delay_por_passo = delay_base * (100 - velocidade) / 100.0
    passo = 2 if angulo_alvo > atual else -2

    for ang in range(int(atual), int(angulo_alvo), passo):
        largura_pulso = pulso_min + (ang / 180.0) * (pulso_max - pulso_min)
        try:
            lgpio.tx_servo(h, pino, int(largura_pulso))
        except lgpio.error as e:
            print(f"[SERVO][ERRO] pino={pino} pulso={largura_pulso:.0f}: {e}")
            continue
        if delay_por_passo > 0:
            time.sleep(delay_por_passo)

    largura_pulso = pulso_min + (angulo_alvo / 180.0) * (pulso_max - pulso_min)
    try:
        lgpio.tx_servo(h, pino, int(largura_pulso))
    except lgpio.error as e:
        print(f"[SERVO][ERRO] pino={pino} pulso={largura_pulso:.0f}: {e}")

    _angulo_atual[pino] = angulo_alvo
    time.sleep(0.8)
    parar_servo(pino)


def mover_servo_direto(pino, angulo_alvo, pulso_min=500, pulso_max=2500):
    """Salto direto (sem rampa) - so pra comparar/testar, o robo de verdade
    so usa isso no boot (posicao desconhecida ainda)."""
    angulo_alvo = max(0, min(180, angulo_alvo))
    largura_pulso = pulso_min + (angulo_alvo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    _angulo_atual[pino] = angulo_alvo
    time.sleep(0.5)
    parar_servo(pino)


def zero_garra(velocidade=VELOCIDADE_PADRAO):
    print(f"[SERVO] Garra -> zero ({ZERO_GARRA})")
    mover_servo(PINO_SG90, ZERO_GARRA, velocidade)


def frouxo_garra():
    print(f"[SERVO] Garra -> frouxo ({FROUXO_GARRA}, vel={VELOCIDADE_FROUXO_GARRA} fixa)")
    mover_servo(PINO_SG90, FROUXO_GARRA, VELOCIDADE_FROUXO_GARRA)


def zero_pesado(velocidade=VELOCIDADE_PADRAO):
    print(f"[SERVO] Pesado -> zero ({ZERO_PESADO})")
    mover_servo(PINO_PESADO, ZERO_PESADO, velocidade)


def zero_todos(velocidade=VELOCIDADE_PADRAO):
    zero_garra(velocidade)
    zero_pesado(velocidade)


def loop_servo(pino, nome, ang_min, ang_max, repeticoes, velocidade, pausa=0.3):
    """Movimento continuo: fica indo de ang_min pra ang_max e voltando,
    repeticoes vezes. Ctrl+C interrompe antes de terminar."""
    print(f"[SERVO] {nome}: {ang_min:.0f} <-> {ang_max:.0f} x{repeticoes} (Ctrl+C pra parar antes)")
    try:
        for i in range(repeticoes):
            mover_servo(pino, ang_max, velocidade)
            time.sleep(pausa)
            mover_servo(pino, ang_min, velocidade)
            time.sleep(pausa)
    except KeyboardInterrupt:
        print("\n[SERVO] Loop interrompido")


# ===================== MENU INTERATIVO =====================
AJUDA = """
===================== TESTE DE SERVOS =====================
  g<graus>[,vel] - move a GARRA (rampa suave) - ex: g90  ou  g90,30 (vel especifica so essa vez)
  p<graus>[,vel] - move o PESADO (rampa suave) - ex: p140  ou  p140,80
  G<graus>   - move a GARRA direto, sem rampa (salto)
  P<graus>   - move o PESADO direto, sem rampa (salto)
  l<min>,<max>[,rep] - GARRA fica indo/voltando entre min e max, rep vezes
                       (padrao 5x) - ex: l0,90  ou  l0,90,10
  L<min>,<max>[,rep] - mesma coisa, mas no PESADO
  v<vel>     - muda a velocidade da rampa, 1-100 (ex: v40)
  1          - garra -> zero (ZERO_GARRA=180)
  2          - pesado -> zero (ZERO_PESADO)
  f          - garra -> frouxo (FROUXO_GARRA=130, sempre vel 20)
  z          - os dois -> zero
  x          - corta o PWM dos dois na hora (para de segurar posicao)
  h          - mostra essa ajuda
  ESC / Ctrl+C - sai
=============================================================
"""

velocidade_atual = VELOCIDADE_PADRAO


def _get_key():
    import sys
    import termios
    import tty

    fd = sys.stdin.fileno()
    antigo = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, antigo)
    return ch


def _ler_texto():
    resto = ""
    while True:
        c = _get_key()
        if c in ('\r', '\n'):
            print()
            break
        if c == '\x7f':  # backspace
            resto = resto[:-1]
            continue
        resto += c
        print(c, end="", flush=True)
    return resto


def _ler_numero():
    texto = _ler_texto()
    try:
        return float(texto)
    except ValueError:
        print(f"[TESTE] valor invalido: {texto!r}")
        return None


def main():
    global velocidade_atual

    print(AJUDA)
    print(f"[TESTE] velocidade_atual={velocidade_atual}")

    try:
        while True:
            tecla = _get_key()

            if tecla in ('\x1b', '\x03'):  # ESC ou Ctrl+C
                break

            elif tecla == 'h':
                print(AJUDA)

            elif tecla in ('g', 'p'):
                # <ang>[,vel] - vel opcional, senao usa velocidade_atual
                print(tecla, end="", flush=True)
                texto = _ler_texto()
                partes = texto.split(',')
                try:
                    ang = float(partes[0])
                    vel = max(1, min(100, int(float(partes[1])))) if len(partes) > 1 else velocidade_atual
                except (ValueError, IndexError):
                    print(f"[TESTE] uso: {tecla}<angulo>[,velocidade] - ex: {tecla}90 ou {tecla}90,30 (valor invalido: {texto!r})")
                    continue
                nome = "Garra" if tecla == 'g' else "Pesado"
                pino = PINO_SG90 if tecla == 'g' else PINO_PESADO
                print(f"[SERVO] {nome} -> {ang:.0f} (rampa, vel={vel})")
                mover_servo(pino, ang, vel)

            elif tecla in ('G', 'P', 'v'):
                print(tecla, end="", flush=True)
                valor = _ler_numero()
                if valor is None:
                    continue
                if tecla == 'G':
                    print(f"[SERVO] Garra -> {valor:.0f} (direto)")
                    mover_servo_direto(PINO_SG90, valor)
                elif tecla == 'P':
                    print(f"[SERVO] Pesado -> {valor:.0f} (direto)")
                    mover_servo_direto(PINO_PESADO, valor)
                elif tecla == 'v':
                    velocidade_atual = max(1, min(100, int(valor)))
                    print(f"[TESTE] velocidade_atual={velocidade_atual}")

            elif tecla in ('l', 'L'):
                print(tecla, end="", flush=True)
                texto = _ler_texto()
                partes = texto.split(',')
                try:
                    ang_min = float(partes[0])
                    ang_max = float(partes[1])
                    repeticoes = int(partes[2]) if len(partes) > 2 else 5
                except (ValueError, IndexError):
                    print(f"[TESTE] uso: l<min>,<max>[,repeticoes] - ex: l0,90,5 (valor invalido: {texto!r})")
                    continue
                pino, nome = (PINO_SG90, "Garra") if tecla == 'l' else (PINO_PESADO, "Pesado")
                loop_servo(pino, nome, ang_min, ang_max, repeticoes, velocidade_atual)

            elif tecla == '1':
                zero_garra(velocidade_atual)

            elif tecla == '2':
                zero_pesado(velocidade_atual)

            elif tecla == 'f':
                frouxo_garra()

            elif tecla == 'z':
                zero_todos(velocidade_atual)

            elif tecla == 'x':
                print("[SERVO] Cortando PWM dos dois")
                parar_servo(PINO_SG90)
                parar_servo(PINO_PESADO)

    except KeyboardInterrupt:
        pass
    finally:
        parar_servo(PINO_SG90)
        parar_servo(PINO_PESADO)
        lgpio.gpiochip_close(h)
        print("\n[TESTE] Encerrado")


if __name__ == "__main__":
    main()

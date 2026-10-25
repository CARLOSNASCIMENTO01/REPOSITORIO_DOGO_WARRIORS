"""
REFERENCIA DE HARDWARE - LINE_FOLLOWER
=======================================
Nao e um script pra rodar - e um resumo de tudo que ja existe pronto em
config.py / movimentos_bases.py / seguimento_de_linha.py / obstaculos_trajeto.py
pra voce usar direto (import config, import movimentos_bases, etc) na hora de
programar uma saida (ou qualquer rotina nova) do zero, sem precisar reler os
arquivos inteiros pra lembrar nome/assinatura de cada funcao.

Import de referencia (o que voce vai usar num modulo novo):
    import config
    import movimentos_bases
    import seguimento_de_linha
    import obstaculos_trajeto

Padrao geral do projeto: nenhuma funcao aqui bloqueia "escondido" por muito
tempo sem dar chance de parar - todo while que segura o robo mais de um
instante tem que chamar config.checar_parada() a cada volta (ver secao 1).


1) PARADA IMEDIATA (botao) - config.py
------------------------------------------------------------------
config.RoboParado
    Excecao levantada por checar_parada() quando o botao pede parada.
    Se voce escrever um while True novo, precisa deixar essa excecao
    propagar (nao dar "except Exception" generico em cima dela) - quem
    ja trata isso e o loop principal (main.py).

config.robo_ativo
    threading.Event(). .is_set() == True quer dizer "robo pode andar".
    O botao (config.botao_thread(), ja rodando numa thread) faz
    .clear()/.set() sozinho.

config.checar_parada()
    Chama a cada iteracao de qualquer while/for que segura o robo por
    mais de um instante (guinada, busca, alinhamento, etc). Custo
    desprezivel. Levanta RoboParado se o botao tiver sido apertado.


2) MOTORES - config.py
------------------------------------------------------------------
config.VEL_MAX / config.VEL_MIN
    Faixa valida de velocidade (unidade antiga do L298N, convertida por
    dentro pra PWM da placa Yahboom). VEL_MIN = -VEL_MAX.

config.mover(velL, velR)
    Aplica velocidade nas duas rodas. Aceita qualquer numero, mas so o
    que estiver dentro de VEL_MIN..VEL_MAX tem efeito de verdade (e
    clampado por dentro). Positivo = frente, negativo = re, sinais
    opostos = giro no proprio eixo.

config.parar()
    Freia com um pulso curto reverso (reduz inercia) e zera. Sempre usar
    essa em vez de mover(0,0) direto quando for parar de verdade.


3) SERVOS - config.py
------------------------------------------------------------------
config.PINO_SG90       = 5   (garra)
config.PINO_PESADO     = 20  (servo pesado / indicador visual)
config.ANGULO_INICIAL  = 90
config.SERVO_PESADO_ANGULO_ATIVO  = 140
config.SERVO_PESADO_ANGULO_PARADO = 90

config.angulo(ang)
    Move config.PINO_SG90 (a garra) direto pro angulo dado (0-180), com
    um pulso curto so. E o jeito mais simples de mexer na garra.

config.mover_servo(pino, angulo_alvo, velocidade=100, pulso_min=500, pulso_max=2500, delay_base=0.02)
    Rampa suave (passo a passo) ate o angulo alvo, chama checar_parada()
    a cada passo, e desliga o PWM no final (evita o servo ficar
    "tremendo"/brigando com o sinal o resto da execucao). Serve pra
    qualquer um dos dois pinos (PINO_SG90 ou PINO_PESADO).

config.mover_servo_direto(pino, angulo_alvo, pulso_min=500, pulso_max=2500)
    Igual, mas sem rampa (vai direto) e sem checar_parada() - usada
    dentro do botao_thread() pra nao se auto-interromper. Tambem desliga
    o PWM no final.

Pra saber o angulo atual de um pino sem mover nada:
    getattr(config.mover_servo, "_angulo_atual", {}).get(config.PINO_SG90, config.ANGULO_INICIAL)
(so funciona depois que mover_servo() ja foi chamado alguma vez nesse pino;
antes disso o dict nem existe, por isso o getattr com default {}).


4) GIROSCOPIO (BNO055) - config.py
------------------------------------------------------------------
config.resetar_giroscopio()
    Zera a referencia - chamar ANTES de comecar a medir uma guinada nova
    (guinada2/guinada_pid ja fazem isso sozinhas).

config.angulo_z()
    Heading (yaw) relativo ao ultimo resetar_giroscopio(), em graus.
    Positivo/negativo dependendo do sentido do giro. E o que guinada2()
    usa pra saber quando parar de girar.

config.angulo_y()
    Angulo de inclinacao (rampa/gangorra), ja filtrado (alpha=0.97).
    controle() usa isso pra saber que esta numa rampa e mudar o
    comportamento do PID. Faixa "piso plano" usada em varios lugares do
    codigo: -5 < angulo_y() < 3.

config.angulo_x()
    Existe, mas nao e usado em nenhuma logica ativa hoje.


5) SONAR (Arduino via serial) - config.py
------------------------------------------------------------------
config.distancia_frente() / config.distancia_esquerda()
    Ultima leitura em cm de cada sonar (nao bloqueia, so le o cache
    atualizado pela serial_thread). ATENCAO: em config.py essas funcoes
    NAO filtram o valor de erro do firmware (o Arduino manda 255.0 exato
    quando o eco falha ou sai de faixa - ver codigo_arduino.ino:lerSonar).
    Isso foi corrigido em resgate.py (distancia_esquerda/frente la
    devolvem SONAR_INVALIDO=9999 quando o valor cru for >= 254, e
    lateral_aberta() exige varias leituras seguidas antes de confiar) -
    se sua saida nova for decidir algo tipo "lateral aberta" a partir de
    uma distancia grande, vale repetir esse mesmo cuidado aqui, senao
    uma falha de leitura isolada do sonar vai ler como "aberto" sem ser.

config.sonar_valido()
    True se chegou algum dado da serial ha menos de
    config.SERIAL_TIMEOUT_DADO (0.5s). Sempre checar isso ANTES de
    confiar numa leitura de distancia.


6) CAMERA DE LINHA - config.py
------------------------------------------------------------------
config.cap
    Objeto cv2.VideoCapture ja aberto (atribuido por iniciar_camera() no
    boot do main.py). Ler direto: ret, frame = config.cap.read()

config.iniciar_camera()
    (Re)abre a camera de linha, com retry e reset de USB embutido se
    precisar. Normalmente voce nao chama isso de novo - so main.py faz
    no boot e quando detecta falha repetida.

config.resetar_usb_camera(video_path_ou_indice)
    Reset via ioctl(USBDEVFS_RESET) - usa se a camera travar de vez.


7) GUINADAS - movimentos_bases.py
------------------------------------------------------------------
movimentos_bases.guinada2(LADO, GRAUS, VELOCIDADE)
    LADO='E' gira esquerda, qualquer outra coisa gira direita. Tempo
    fixo controlado pelo giroscopio (para quando angulo_z() bate GRAUS).
    E a guinada "padrao" usada em quase todo canto do codigo.

movimentos_bases.guinada_pid(LADO, GRAUS, VEL_MAX_GIRO=100, VEL_MIN_GIRO=29, ZONA_CHEGADA=10)
    Mesma ideia, mas com PID fechado sobre o angulo (desacelera suave
    perto do alvo em vez de vel constante ate bater). LADO='D' ou 'E'.
    Usada na saida do resgate; mais suave mas mais lenta pra escrever/
    calibrar do zero.


8) VISAO DA LINHA / MARCADORES - seguimento_de_linha.py
------------------------------------------------------------------
seguimento_de_linha.visao(frame)
    -> cx, contours, roi, mask
    ROI = parte de baixo do frame. cx = centro do maior contorno preto
    (None se nao achou nada). Base de tudo que segue linha.

seguimento_de_linha.gap_debug(contours)
    -> bool. True quando nao ha contorno "de linha" valido (sumiu ou
    area pequena demais) - e o sinal de "gap"/possivel sala de resgate
    usado no main.py.

seguimento_de_linha.detectar_travamento(roi)
    -> bool. Compara frames subsequentes (parado ha muito tempo sem
    diferenca visual = travado).

seguimento_de_linha.detectar_fita_vermelha(frame)
    -> bool. Fim de percurso - se True, ja para o robo sozinha por
    dentro (config.parar() + sleep).

seguimento_de_linha.verde(roi)
    Detecta marcador(es) verde(s) e ja executa a curva 90/180 sozinha
    quando decide - nao e so deteccao passiva, mexe no robo.

seguimento_de_linha.controle(cx, contours, roi)
    O PID que segue a linha de verdade - ja chama config.mover() por
    dentro. Ver bloco de CONFIG no topo do proprio modulo (Kp, Kd,
    VEL_BASE, DEADZONE, ALPHA) pra tuning.


9) OBSTACULO - obstaculos_trajeto.py
------------------------------------------------------------------
obstaculos_trajeto.desvio()
    Confere o sonar frontal e, se achar obstaculo (<=5cm, so em piso
    plano), executa a manobra de desvio inteira sozinha (contorna pela
    direita e volta pra linha). Chamar a cada volta do loop principal.


10) HANDOFF PRO RESGATE - resgate_de_vitimas.py
------------------------------------------------------------------
resgate_de_vitimas.area(contours)
    Quando main.py decide que achou a sala de resgate (gap continuo por
    tempo demais), chama essa funcao - ela faz os.execv() e TROCA o
    processo inteiro pelo resgate.py (nao e uma chamada normal que
    retorna). Se sua "saida nova" for pra dentro do proprio seguidor
    (nao pro resgate.py), voce nao precisa mexer nisso - so se aplica se
    for alterar o que acontece QUANDO acha a sala.


ONDE ENCAIXAR UMA SAIDA NOVA
------------------------------------------------------------------
Se for uma funcao nova tipo "minha_saida()" pra rodar dentro do proprio
seguidor de linha (nao dentro do resgate.py), o esqueleto minimo e:

    import time
    import config
    import movimentos_bases

    def minha_saida():
        while True:
            config.checar_parada()
            if not config.sonar_valido():
                time.sleep(0.02)
                continue

            frente = config.distancia_frente()
            esquerda = config.distancia_esquerda()
            # ... sua logica de decisao aqui, usando config.mover(),
            # movimentos_bases.guinada2(), config.cap.read() +
            # seguimento_de_linha.visao() pra achar linha/abertura, etc.

            time.sleep(0.02)

Chame ela de dentro do loop principal (main.py) no ponto que fizer
sentido pro seu fluxo, do mesmo jeito que obstaculos_trajeto.desvio() e
chamado hoje.
"""

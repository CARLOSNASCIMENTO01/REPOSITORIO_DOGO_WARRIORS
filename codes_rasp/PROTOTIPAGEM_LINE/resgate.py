from ultralytics import YOLO
import cv2
import lgpio as GPIO
import time
import lgpio
import numpy as np
import sys
import os
import threading

# ===================== BOTAO DE PARADA IMEDIATA (mesmo padrao do seguidor_debug2.py) =====================
BUTTON = 24  # mesmo pino usado no codigo de linha

class RoboParado(Exception):
    """Levantada quando o botao pede parada, pra abortar QUALQUER loop na hora."""
    pass

robo_ativo = threading.Event()
robo_ativo.set()  # comeca ligado

def checar_parada():
    """Chamar a cada iteracao de todo while/for que segura o robo por mais
    de um instante (buscas, centralizacao, aproximacao, entrega, etc)."""
    if not robo_ativo.is_set():
        raise RoboParado()

def botao_thread():
    """Toggle com debounce: aperta -> para tudo na hora (mesmo no meio de
    uma busca/aproximacao/entrega). Aperta de novo -> deixa o robo continuar
    (retoma a busca por bolas do zero)."""
    anterior = 0
    while True:
        estado = GPIO.gpio_read(h, BUTTON)
        if estado == 0 and anterior == 1:
            time.sleep(0.03)
            if GPIO.gpio_read(h, BUTTON) == 0:
                if robo_ativo.is_set():
                    robo_ativo.clear()
                    parar_motores()
                    print("[BOTAO] PAROU")
                else:
                    print("[BOTAO] REINICIANDO...")
                    robo_ativo.set()
                while GPIO.gpio_read(h, BUTTON) == 1:
                    time.sleep(0.02)
        anterior = estado
        time.sleep(0.02)

# ===================== DETECÇÃO DE TRIÂNGULO (área de entrega) =====================
ALVO_COR = None  # None = aceita verde OU vermelho; setado dinamicamente em entregar_bola()
TIPO_BOLA = None

# ===================== MAPEAMENTO CLASSE DA BOLA -> COR DA ÁREA DE ENTREGA =====================
# Mapeado pelo NOME da classe (não pelo índice), então não importa a ordem
# que o YOLO carregar as classes internamente.
NOME_COR_ENTREGA = {
    "bola-preta": "vermelho",
    "bola-prata": "verde",
}
# Preenchido automaticamente depois que o modelo é carregado (ver mais
# abaixo, logo após `model = YOLO(MODEL_PATH)`): vira {cls_id: cor}.
CLASSE_COR_ENTREGA = {}

green_min = np.array([40, 50, 45])
green_max = np.array([85, 255, 255])

red_min1 = np.array([0, 100, 100])
red_max1 = np.array([10, 255, 255])
red_min2 = np.array([170, 100, 100])
red_max2 = np.array([180, 255, 255])

kernel_tri = np.ones((3, 3), np.uint8)

PROPORCAO_ESPERADA = 30 / 6
TOLERANCIA_PROPORCAO = 0.8
AREA_MIN_TRI = 500
AREA_MAX_TRI = 1000000

STOP_AREA_RATIO_TRIANGULO = 0.80   # ajustar na prática, igual fizemos com a bola
TRIANGULO_MAX_GIROS = 90
TRIANGULO_MAX_APPROACH_STEPS = 90
TRIANGULO_LOST_FRAMES = 15
TRIANGULO_APPROACH_SPEED = 40 #30
TRIANGULO_APPROACH_STEP_TIME = 0.15
TRIANGULO_CENTER_MAX_TENTATIVAS = 25
TRIANGULO_CENTER_LOST_FRAMES = 15
# ===================== CONFIGURAÇÕES =====================
MODEL_PATH = "model.onnx"
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
CROP_W, CROP_H = 320, 240
CONF_THRES = 0.35
IMG_CENTER_X = CROP_W // 2
TOLERANCE = 30

IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13

VEL_MAX = 50
VEL_MIN = -VEL_MAX
# ===================== APROXIMAÇÃO POR TAMANHO DE BBOX =====================
APPROACH_SPEED = 50        # velocidade do avanço durante a aproximação # 30
APPROACH_STEP_TIME = 0.18  # duração de cada "passo" de avanço/correção # 0.15
STOP_AREA_RATIO = 0.35     # bbox_area / crop_area -> ajustar na prática!
MAX_APPROACH_STEPS = 60    # trava de segurança (evita loop infinito)
LOST_BALL_FRAMES = 8       # quantos frames sem detectar até desistir
TURN_PULSE = 0.24 #0.2
STOP_TIME = 0.12 #0.2
FORWARD_TIME = 0.6

# ===================== CENTRALIZAÇÃO PROPORCIONAL =====================
CENTER_TURN_SPEED_MIN = 30   # giro mais fraco quando o erro é pequeno
CENTER_TURN_SPEED_MAX = 42   # giro mais forte quando o erro é grande
CENTER_MAX_TENTATIVAS = 15
CENTER_LOST_FRAMES = 5


MAX_ENTREGAS = 3
BUSCA_FINAL_MAX_GIROS = 20  # ajustar na prática

# Pinos
PINO_SG90 = 21
PINO_PESADO = 20

ANGULO_INICIAL = 90

h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, PINO_SG90)
lgpio.gpio_claim_output(h, PINO_PESADO)
GPIO.gpio_claim_input(h, BUTTON, GPIO.SET_PULL_UP)

_angulo_atual = {
    PINO_SG90: ANGULO_INICIAL,
    PINO_PESADO: ANGULO_INICIAL,
}

DELAY_BASE = 0.03


# ===================== SETUP GPIO (lgpio) =====================
pins = [IN1, IN2, IN3, IN4, ENA, ENB]
for p in pins:
    GPIO.gpio_claim_output(h, p)
GPIO.tx_pwm(h, ENA, 50, 0)
GPIO.tx_pwm(h, ENB, 50, 0)


# ===================== MOTORES (lgpio) =====================
def aplicar_motor(in1, in2, pwm_pin, vel):
    vel = max(min(vel, VEL_MAX), VEL_MIN)
    if vel > 0:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 1)
    elif vel < 0:
        GPIO.gpio_write(h, in1, 1)
        GPIO.gpio_write(h, in2, 0)
    else:
        GPIO.gpio_write(h, in1, 0)
        GPIO.gpio_write(h, in2, 0)
    duty = min(abs(vel), 100)
    GPIO.tx_pwm(h, pwm_pin, 50, duty)


def mover(velL, velR):
    aplicar_motor(IN2, IN1, ENA, velL)
    aplicar_motor(IN4, IN3, ENB, velR)


def parar_motores():
    mover(0, 0)


def forward(speed=30):
    mover(speed, speed)


def turn_left(speed=30):
    mover(-speed, speed)


def turn_right(speed=30):
    mover(speed, -speed)


def buscar_bola():
    """
    Gira no lugar até achar qualquer bola (usado só na busca final,
    depois da 3ª entrega). Retorna True se achou.
    """
    for _ in range(BUSCA_FINAL_MAX_GIROS):
        checar_parada()
        turn_right(speed=55)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.25)

        crop = capturar_crop()
        if crop is None:
            continue

        cx, area, bbox, cls_id = detectar_bola(crop)
        if cx is not None:
            return True
    return False


def acionar_saida():
    print("saindo")
    while True:
        mover(30,30)

    os.execv(sys.executable, [sys.executable, "/home/pi/ROBO_DOGO_2026/codes_rasp/PROTOTIPAGEM_LINE/seguidor_debug2.py"])


def _mascara_cor_tri(hsv, cor):
    if cor == "verde":
        mask = cv2.inRange(hsv, green_min, green_max)
    elif cor == "vermelho":
        m1 = cv2.inRange(hsv, red_min1, red_max1)
        m2 = cv2.inRange(hsv, red_min2, red_max2)
        mask = cv2.bitwise_or(m1, m2)
    else:
        raise ValueError("cor deve ser 'verde' ou 'vermelho'")
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_tri, iterations=2)
    mask = cv2.dilate(mask, kernel_tri, iterations=2)
    return mask

def desenhar_deteccoes_tri(frame, deteccoes):
    saida = frame.copy()
    for d in deteccoes:
        cor_bgr = (0, 255, 0) if d["cor"] == "verde" else (0, 0, 255)
        if "vertices" in d:
            cv2.drawContours(saida, [d["vertices"]], -1, cor_bgr, 2)
        cv2.circle(saida, d["centro"], 4, cor_bgr, -1)
        texto = f'{d["cor"]} :1 area={d["area"]:.0f}'
        cv2.putText(saida, texto, (d["centro"][0] - 20, d["centro"][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, cor_bgr, 1)
    return saida

def _eh_triangulo(contorno, encostando_borda=False):
    print("ENTROU _eh_triangulo")
    area = cv2.contourArea(contorno)
    if area < AREA_MIN_TRI or area > AREA_MAX_TRI:
        print("REJEITOU AREA")
        return False, None

    perimetro = cv2.arcLength(contorno, True)
    aproximado = cv2.approxPolyDP(contorno, 0.10 * perimetro, True)

    print("Vertices internos:", len(aproximado))

    # Se o contorno está encostando na borda do crop, o polígono real
    # foi cortado pela imagem -- é normal ter menos vértices quando
    # estamos muito perto. Nesse caso aceitamos a partir de 2 vértices.
    min_vertices = 2 if encostando_borda else 3
    if len(aproximado) < min_vertices or len(aproximado) > 4:
        print("REJEITOU VERTICES")
        return False, None

    # Convexidade só faz sentido checar quando o polígono não foi
    # cortado pela borda (senão vira côncavo naturalmente e não é bug).
    if not encostando_borda and not cv2.isContourConvex(aproximado):
        print("REJEITOU CONVEX")
        return False, None

    print("ACEITOU")

    # Centroide real da área visível (média dos pixels do contorno),
    # em vez do centro do bounding rect -- evita que o centro fique
    # puxado pra ponta quando a câmera só enxerga parte do retângulo.
    M = cv2.moments(contorno)
    if M["m00"] == 0:
        return False, None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]

    return True, {
        "centro": (int(cx), int(cy)),
        "area": area,
        "cor": None,
        "vertices": aproximado.reshape(-1, 2),
    }

def detectar_entrega(crop):
    """
    Procura triângulos (cantos de parede) no crop. Se ALVO_COR estiver
    definido, só aceita essa cor; senão aceita verde ou vermelho e
    devolve o de MAIOR área (mais perto). Retorna (cx, area, info) ou
    (None, None, None). Sempre mostra as janelas de debug (normal +
    máscara verde + máscara vermelho).
    """
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    cores = [ALVO_COR] if ALVO_COR else ["verde", "vermelho"]

    todas_deteccoes = []
    mask_verde = None
    mask_vermelho = None

    for cor in cores:
        mask = _mascara_cor_tri(hsv, cor)
        if cor == "verde":
            mask_verde = mask
        else:
            mask_vermelho = mask

        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Quantidade de contornos: {len(contornos)}")
        for c in contornos:
            area = cv2.contourArea(c)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            print(f"Area={area:.1f}  Vertices={len(approx)}")

            x, y, w, h_bb = cv2.boundingRect(c)
            margem = 2
            encostando_borda = (
                x <= margem or y <= margem or
                x + w >= crop.shape[1] - margem or
                y + h_bb >= crop.shape[0] - margem
            )

            print("Chamando _eh_triangulo")
            ok, info = _eh_triangulo(c, encostando_borda=encostando_borda)
            print("Resultado:", ok)
            if ok:
                info["cor"] = cor
                todas_deteccoes.append(info)

    # ---- debug visual ----
    debug = desenhar_deteccoes_tri(crop, todas_deteccoes)
    cv2.imshow("Entrega - Normal", debug)
    if mask_verde is not None:
        cv2.imshow("Entrega - Mascara Verde", mask_verde)
    if mask_vermelho is not None:
        cv2.imshow("Entrega - Mascara Vermelho", mask_vermelho)
    cv2.waitKey(1)

    if not todas_deteccoes:
        return None, None, None

    melhor = max(todas_deteccoes, key=lambda d: d["area"])
    print(f"Detecções encontradas: {len(todas_deteccoes)}")
    for d in todas_deteccoes:
        print(d["area"], d["centro"])
    return melhor["centro"][0], melhor["area"], melhor

def centralizar_triangulo():
    """Igual centralizar_bola(), mas usando detectar_entrega()."""
    print(f"🔎 [centralizar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    perdida = 0
    for _ in range(TRIANGULO_CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, info = detectar_entrega(crop)

        print("cx =", cx, "| cor detectada =", info["cor"] if info else None)
        print("area =", area)

        if cx is None:
            perdida += 1
            if perdida >= TRIANGULO_CENTER_LOST_FRAMES:
                print("❌ Perdeu o triângulo durante centralização")
                parar_motores()
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"✅ Centralizado no triângulo (cx={cx})")
            parar_motores()
            return True

        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN)
        pulso = 0.05 + erro_norm * 0.10

        if erro < 0:
            turn_left(speed=speed)
        else:
            turn_right(speed=speed)

        time.sleep(pulso)
        parar_motores()
        time.sleep(0.05)

    print("⚠️ Estourou tentativas de centralização no triângulo")
    parar_motores()
    return False


def aproximar_triangulo():
    """Igual aproximar_bola(), mas até chegar perto do triângulo pra soltar a bola."""
    print(f"🔎 [aproximar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    crop_area = CROP_W * CROP_H
    perdida = 0

    for passo in range(TRIANGULO_MAX_APPROACH_STEPS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, info = detectar_entrega(crop)

        if cx is None:
            perdida += 1
            print(f"⚠️ Triângulo sumiu durante aproximação ({perdida}/{TRIANGULO_LOST_FRAMES})")
            if perdida >= TRIANGULO_LOST_FRAMES:
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"📐 [entrega] passo={passo} area_ratio={ratio:.2f} cx={cx} cor={info['cor']}")

        if ratio >= STOP_AREA_RATIO_TRIANGULO:
            print("✅ Perto o suficiente do triângulo, parando")
            parar_motores()
            return True

        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("↔️ Saiu do centro durante aproximação da entrega, recentralizando")
            if not centralizar_triangulo():
                return False
            continue

        forward(speed=TRIANGULO_APPROACH_SPEED)
        time.sleep(TRIANGULO_APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("⚠️ Estourou o limite de passos na aproximação da entrega")
    parar_motores()
    return False

def buscar_triangulo():
    """Gira no lugar até achar um triângulo. Retorna True se achou."""
    print(f"🔎 [buscar_triangulo] ALVO_COR ativo = {ALVO_COR!r}")
    for _ in range(TRIANGULO_MAX_GIROS):
        checar_parada()
        crop = capturar_crop()
        if crop is not None:
            cx, area, info = detectar_entrega(crop)
            if cx is not None:
                return True
        turn_right(speed=45)
        time.sleep(TURN_PULSE)
        parar_motores()
        time.sleep(0.3)
    return False


def entregar_bola(cor_alvo=None):
    """
    Sequência completa pós-captura: busca o triângulo (canto de parede)
    da COR CERTA para a bola que está sendo carregada, centraliza,
    aproxima e solta a bola oscilando a garra pesada.

    cor_alvo: "vermelho" ou "verde". Se None, aceita qualquer triângulo
    (comportamento antigo, sem diferenciação por classe).
    """
    global ALVO_COR
    ALVO_COR = cor_alvo

    checar_parada()
    parar_motores()
    time.sleep(0.5)

    for _ in range(8):
        cap.read()

    if cor_alvo:
        print(f"🔺 Procurando área de entrega {cor_alvo.upper()}...")
    else:
        print("🔺 Procurando área de entrega (qualquer cor)...")

    if not buscar_triangulo():
        print("❌ Não achou nenhum triângulo, abortando entrega")
        ALVO_COR = None
        return False

    if not centralizar_triangulo():
        print("❌ Não conseguiu centralizar no triângulo")
        ALVO_COR = None
        return False

    print("⬆️ Centralizado no triângulo → aproximando")
    if not aproximar_triangulo():
        print("❌ Não conseguiu se aproximar do triângulo")
        ALVO_COR = None
        return False

    print("📤 Soltando a bola...")
    print("⬆️ Centralizado no triângulo → aproximando")
    if not aproximar_triangulo():
        print("❌ Não conseguiu se aproximar do triângulo")
        ALVO_COR = None
        return False

    # Recentraliza uma última vez, já perto, pra garantir que a bola
    # caia no MEIO do retângulo e não na ponta.
    checar_parada()
    print("🎯 Recentralizando antes de soltar...")
    if not centralizar_triangulo():
        print("❌ Perdeu o triângulo na recentralização final")
        ALVO_COR = None
        return False
    
    print("🎯 Recentralizando antes de soltar...")
    if not centralizar_triangulo():
        print("❌ Perdeu o triângulo na recentralização final")
        ALVO_COR = None
        return False

    checar_parada()
    verificar_e_afastar_bola_bloqueio()   # <-- checa e afasta bola do canto, se houver

    checar_parada()
    print("📤 Soltando a bola...")
    mover(40,40)
    time.sleep(1.5)
    parar_motores()          # <-- adiciona os parênteses
    angulo(170)
    oscilar_servo_lgpio(PINO_PESADO, 40, 90,10,0.2)
   
    mover_servo(PINO_PESADO, 150, velocidade=50)
    time.sleep(0.5)
    mover_servo(PINO_PESADO, 140, velocidade=50)
    time.sleep(1.0)
    mover(-30,-30)
    time.sleep(1.0)
    parar_motores()          # <-- adiciona essa também, no final da ré
    #parar_servo(PINO_PESADO)
    #parar_servo(PINO_SG90)
    print("✅ Bola entregue")
    ALVO_COR = None
    return True    

def mover_servo(pino, angulo, velocidade=100, pulso_min=500, pulso_max=2500):
    angulo = max(0, min(180, angulo))
    velocidade = max(1, min(100, velocidade))

    angulo_atual = _angulo_atual.get(pino, ANGULO_INICIAL)
    delay_por_passo = DELAY_BASE * (100 - velocidade) / 100.0

    passo = 2 if angulo > angulo_atual else -2

    for ang in range(int(angulo_atual), int(angulo), passo):
        checar_parada()
        largura_pulso = pulso_min + (ang / 180.0) * (pulso_max - pulso_min)
        try:
            lgpio.tx_servo(h, pino, int(largura_pulso))
        except lgpio.error as e:
            print(f"⚠️ Falha ao mover servo (pino={pino}, pulso={largura_pulso:.0f}): {e}")
            continue
        if delay_por_passo > 0:
            time.sleep(delay_por_passo)

    _angulo_atual[pino] = angulo
    time.sleep(0.8)

    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error as e:
        print(f"⚠️ Falha ao desligar PWM do servo (pino={pino}): {e}")

def angulo(ang):
    ang = max(0, min(180, ang))

    # 500 us -> 0�
    # 2500 us -> 180�
    pulso = int(500 + (2000 * ang / 180))

    print("Pulso:", pulso)

    lgpio.tx_servo(h, PINO_SG90, pulso)

def mover_servo_direto(pino, angulo, pulso_min=500, pulso_max=2500):
    angulo = max(0, min(180, angulo))
    largura_pulso = pulso_min + (angulo / 180.0) * (pulso_max - pulso_min)
    lgpio.tx_servo(h, pino, int(largura_pulso))
    time.sleep(0.01)
    parar_servo(pino)
    _angulo_atual[pino] = angulo 


def oscilar_servo_lgpio(pino, angulo_a, angulo_b, repeticoes=5, delay=0.15):
    """
    Oscila servo rapidamente usando diretamente lgpio.tx_servo().
    Não usa mover_servo() nem parar_servo().
    """

    pulso_min = 500
    pulso_max = 2500

    def angulo_para_pulso(angulo):
        angulo = max(0, min(180, angulo))
        return int(
            pulso_min +
            (angulo / 180.0) * (pulso_max - pulso_min)
        )

    pulso_a = angulo_para_pulso(angulo_a)
    pulso_b = angulo_para_pulso(angulo_b)

    print(f"Oscilando servo GPIO {pino}: {angulo_a}({pulso_a}) <-> {angulo_b}({pulso_b})")

    try:
        for _ in range(repeticoes):
            checar_parada()

            # vai para posição A
            lgpio.tx_servo(h, pino, pulso_a)
            time.sleep(delay)

            # vai para posição B
            lgpio.tx_servo(h, pino, pulso_b)
            time.sleep(delay)

    except lgpio.error as e:
        print("ERRO OSCILA SERVO:", e)

    finally:
        # mantém o último pulso por um pouco
        # não corta PWM imediatamente
        time.sleep(0.2)


def parar_servo(pino):
    try:
        lgpio.tx_servo(h, pino, 0)
    except lgpio.error:
        pass

def capturar_crop():
    for _ in range(6):
        cap.read()
    ret, frame = cap.read()
    if not ret:
        return None
    return center_crop(frame, CROP_W, CROP_H)


def centralizar_bola():
    """
    Gira o robô até a bola cair dentro de TOLERANCE do centro.
    Velocidade e duração do pulso escalam com o tamanho do erro
    (proporcional), em vez do pulso fixo de antes -- evita corrigir
    de menos com erro grande e passar do ponto com erro pequeno.
    Retorna True se centralizou, False se perdeu a bola.
    """
    perdida = 0
    for _ in range(CENTER_MAX_TENTATIVAS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop)

        if cx is None:
            perdida += 1
            if perdida >= CENTER_LOST_FRAMES:
                print("❌ Perdeu a bola durante centralização")
                parar_motores()
                return False
            continue

        perdida = 0
        erro = cx - IMG_CENTER_X

        if abs(erro) <= TOLERANCE:
            print(f"✅ Centralizado (cx={cx})")
            parar_motores()
            return True

        erro_norm = min(abs(erro) / IMG_CENTER_X, 1.0)  # 0..1
        speed = CENTER_TURN_SPEED_MIN + erro_norm * (CENTER_TURN_SPEED_MAX - CENTER_TURN_SPEED_MIN) 
        pulso = 0.05 + erro_norm * 0.10  # erro grande -> pulso mais longo tb

        if erro < 0:
            print(f"⬅️ Corrigindo esquerda (erro={erro}, speed={speed:.0f})")
            turn_left(speed=speed)
        else:
            print(f"➡️ Corrigindo direita (erro={erro}, speed={speed:.0f})")
            turn_right(speed=speed)

        time.sleep(pulso)
        parar_motores()
        time.sleep(0.05)  # deixa a vibração assentar antes de recapturar

    print("⚠️ Estourou tentativas de centralização")
    parar_motores()
    return False

def detectar_bola(crop):
    """
    Roda o modelo num crop e devolve (cx, area, bbox, cls_id) ou
    (None, None, None, None). cls_id é o índice da classe detectada
    (ex.: bola preta ou bola prata), usado depois pra saber em qual
    área de entrega (cor) soltar a bola.
    """
    results = model.predict(
        crop, imgsz=(320, 256), conf=CONF_THRES, verbose=False, show=False
    )
    r = results[0]
    boxes = getattr(r, "boxes", None)

    annotated = r.plot()
    cv2.imshow("Detecção", annotated)
    cv2.waitKey(1)

    if boxes is not None and len(boxes) > 0:
        idx = escolher_indice_mais_perto(boxes)
        x1, y1, x2, y2 = map(int, boxes.xyxy[idx].tolist())
        cx = (x1 + x2) // 2
        area = (x2 - x1) * (y2 - y1)
        cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
        return cx, area, (x1, y1, x2, y2), cls_id

    return None, None, None, None


def aproximar_bola():
    """
    Avança em passos curtos até a bola ocupar uma fração grande o
    suficiente da imagem (proxy de distância real). Sempre que a bola
    sai da tolerância central durante a aproximação, chama
    centralizar_bola() pra recentralizar antes de continuar avançando.
    Retorna True se chegou perto o suficiente, False se perdeu a bola
    ou estourou o limite de passos.
    """
    crop_area = CROP_W * CROP_H
    perdida = 0

    for passo in range(MAX_APPROACH_STEPS):
        checar_parada()
        crop = capturar_crop()
        if crop is None:
            print("❌ Erro na câmera durante aproximação")
            parar_motores()
            return False

        cx, area, bbox, cls_id = detectar_bola(crop)

        if cx is None:
            perdida += 1
            print(f"⚠️ Bola sumiu durante aproximação ({perdida}/{LOST_BALL_FRAMES})")
            if perdida >= LOST_BALL_FRAMES:
                print("❌ Perdeu a bola de vez, abortando aproximação")
                parar_motores()
                return False
            time.sleep(APPROACH_STEP_TIME)
            continue

        perdida = 0
        ratio = area / crop_area
        print(f"📐 passo={passo} area_ratio={ratio:.2f} cx={cx}")

        if ratio >= STOP_AREA_RATIO:
            print("✅ Perto o suficiente pra garra, parando")
            parar_motores()
            return True

        # se saiu do centro, delega a correção pro centralizar_bola
        if abs(cx - IMG_CENTER_X) > TOLERANCE:
            print("↔️ Saiu do centro durante aproximação, recentralizando")
            centralizada = centralizar_bola()
            if not centralizada:
                print("❌ Perdeu a bola tentando recentralizar na aproximação")
                return False
            continue  # depois de recentralizar, reavalia área no próximo passo

        # já tá centralizada -> só avança
        forward(speed=APPROACH_SPEED)
        time.sleep(APPROACH_STEP_TIME)
        parar_motores()
        time.sleep(0.05)

    print("⚠️ Estourou o limite de passos na aproximação")
    parar_motores()
    return False
# ===================== FUNÇÕES AUXILIARES =====================
def center_crop(frame, crop_w, crop_h):
    hh, ww = frame.shape[:2]
    cx, cy = ww // 2, hh // 2
    x1 = max(0, cx - crop_w // 2)
    y1 = max(0, cy - crop_h // 2)
    x2, y2 = x1 + crop_w, y1 + crop_h
    return frame[y1:y2, x1:x2].copy()

def escolher_indice_mais_perto(boxes):
    """
    Quando o YOLO detecta mais de uma bola no mesmo frame, escolhe a
    de MAIOR área de bbox (ou seja, a mais perto da câmera) e ignora
    as outras nesse ciclo, pra não ficar indeciso sobre qual perseguir.
    """
    if boxes is None or len(boxes) == 0:
        return None
    maior_area = -1
    melhor_idx = 0
    for i in range(len(boxes)):
        x1, y1, x2, y2 = map(int, boxes.xyxy[i].tolist())
        area = (x2 - x1) * (y2 - y1)
        if area > maior_area:
            maior_area = area
            melhor_idx = i
    return melhor_idx

def verificar_e_afastar_bola_bloqueio():
    """
    Verifica, usando o modelo YOLO (mesma detecção que já usamos pra
    achar bola normalmente), se tem alguma bola bloqueando o canto/
    triângulo de entrega. Se tiver, chama o movimento de afastar antes
    de soltar a bola que estamos carregando -- senão ela pode empurrar
    ou trombar na bola que já tá lá.
    """
    crop = capturar_crop()
    if crop is None:
        return

    cx, area, bbox, cls_id = detectar_bola(crop)
    if cx is not None:
        print("⚠️ Bola bloqueando a área de entrega, afastando...")
        afastar_bola_do_canto()


def afastar_bola_do_canto():
    '''checar_parada()
    mover(-30, -30)
    time.sleep(0.5)
    mover_servo(PINO_PESADO, 5, velocidade=10)
    mover(30, 30)
    time.sleep(0.6)
    mover(-30, 30)
    time.sleep(0.2)
    mover(30, -30)
    time.sleep(0.4)
    mover(-30, 30)
    time.sleep(0.4)
    mover(30, -30)
    time.sleep(0.2)'''
    pass    

# ===================== SETUP CÂMERA E MODELO =====================
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print(f"❌ Não consegui abrir a câmera de resgate em {CAMERA_INDEX}")
    exit(1)

model = YOLO(MODEL_PATH)
print("📋 Classes do modelo:", model.names)

def _normalizar_nome(nome):
    return str(nome).strip().lower()

_NOME_COR_ENTREGA_NORM = {_normalizar_nome(k): v for k, v in NOME_COR_ENTREGA.items()}

# Monta CLASSE_COR_ENTREGA (id -> cor) a partir do nome de cada classe
for _cls_id, _nome in model.names.items():
    _nome_norm = _normalizar_nome(_nome)
    _cor = _NOME_COR_ENTREGA_NORM.get(_nome_norm)
    if _cor is None:
        print(f"⚠️ Classe '{_nome}' (repr={_nome!r}, id={_cls_id}) não está em NOME_COR_ENTREGA, "
              f"bolas dessa classe vão ser entregues em qualquer área")
    else:
        CLASSE_COR_ENTREGA[_cls_id] = _cor
print("🎯 Mapeamento classe -> cor de entrega:", CLASSE_COR_ENTREGA)
cv2.namedWindow("Detecção", cv2.WINDOW_NORMAL)
mover_servo(PINO_PESADO, 150, velocidade=100)
entregas_realizadas = 0

# Sobe a thread do botão antes do loop principal, igual no seguidor_debug2.py
threading.Thread(target=botao_thread, daemon=True).start()

# ===================== LOOP PRINCIPAL =====================
try:
    print("🚗 Iniciando busca por bolas...")
    while True:
        try:
            checar_parada()
            turn_right(speed=55)
            time.sleep(TURN_PULSE)

            parar_motores()

            # Espera a vibração parar
            time.sleep(0.25)

            # Descarta alguns frames antigos da câmera
            for _ in range(4):
                cap.read()

            # Captura um frame atualizado
            ret, frame = cap.read()
            if not ret:
                print("❌ Erro na câmera")
                break
            crop = center_crop(frame, CROP_W, CROP_H)
            inicio = time.perf_counter()
            results = model.predict(
                crop,
                imgsz=(320, 256),
                conf=CONF_THRES,
                verbose=False,
                show=False
            )
            print((time.perf_counter() - inicio) * 1000)
            r = results[0]
            boxes = getattr(r, "boxes", None)

            # SEMPRE mostra a tela, com ou sem deteccao (r.plot() desenha as
            # caixas se houver, ou so a imagem crua se nao houver nada)
            annotated = r.plot()
            cv2.imshow("Detecção", annotated)
            

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

            if boxes is not None and len(boxes) > 0:
                idx = escolher_indice_mais_perto(boxes)
                xyxy = boxes.xyxy[idx].tolist()
                x1, y1, x2, y2 = map(int, xyxy)
                cx = (x1 + x2) // 2
                cls_id = int(boxes.cls[idx].item()) if getattr(boxes, "cls", None) is not None else None
                nome_classe = model.names.get(cls_id, "desconhecida") if cls_id is not None else "desconhecida"
                print(f"🎯 Bola detectada em cx={cx} | classe={cls_id} ({nome_classe})")

                if abs(cx - IMG_CENTER_X) > TOLERANCE:
                    if not centralizar_bola():
                        print("🔄 Não conseguiu centralizar, voltando pra busca")
                        continue

                print("⬆️ Centralizado → avançando")
                if not aproximar_bola():
                    print("🔄 Não conseguiu se aproximar, voltando pra busca")
                    continue

                print("🦾 Fechando garra na bola")
                checar_parada()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                angulo(0)
                mover_servo(PINO_PESADO, 30, velocidade=10)
                parar_servo(PINO_SG90)
                mover(-30,-30)
                time.sleep(0.1)
                parar_motores()
                mover(26,26)
                time.sleep(0.63)
                parar_motores()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 0, velocidade=5)
                mover(30,30)
                time.sleep(0.5)
                parar_motores()
                mover(-30,-30)
                time.sleep(0.5)
                parar_motores()
                mover(30,30)
                time.sleep(0.1)
                parar_motores()
                mover(-30,-30)
                time.sleep(0.1)
                parar_motores()
                mover(-10,-10)
                time.sleep(0.09)
                parar_motores()
                mover_servo(PINO_PESADO, 150, velocidade=5)
                mover(-50,-50)
                time.sleep(0.3)
                parar_motores()
                cor_alvo = CLASSE_COR_ENTREGA.get(cls_id)
                if cor_alvo is None:
                    print(f"⚠️ Classe {cls_id} não mapeada em CLASSE_COR_ENTREGA, entregando em qualquer área")
                entregar_bola(cor_alvo)
                entregas_realizadas += 1

                if entregas_realizadas >= MAX_ENTREGAS:
                    print("🏁 3 entregas concluídas, fazendo busca final...")
                    if not buscar_bola():
                        print("🚪 Nenhuma bola na busca final, acionando saída")
                        acionar_saida()
                        break
                    else:
                        print("🎯 Achou mais uma bola na busca final, continuando normalmente")
                        # deixa o loop seguir e a próxima iteração já detecta e pega essa bola

            else:
                print("🔄 Nenhuma bola — novo passo de giro")

        except RoboParado:
            print("🛑 Robo parado pelo botao,voltando pro seguidor...")
            parar_motores()
            cap.release()
            GPIO.gpiochip_close(h)
            os.execv(sys.executable, [sys.executable, "/home/pi/ROBO_DOGO_2026/codes_rasp/PROTOTIPAGEM_LINE/seguidor_debug2.py"])
            '''robo_ativo.wait()
            print("▶️ Retomando busca por bolas...")
            continue'''

except KeyboardInterrupt:
    print("\n🛑 Interrompido pelo usuário.")
finally:
    parar_motores()
    GPIO.gpiochip_close(h)
    cap.release()
    cv2.destroyAllWindows()
    print("✔️ Finalizado com segurança.")
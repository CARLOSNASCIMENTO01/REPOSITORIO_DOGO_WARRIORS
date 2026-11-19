
import time

import cv2
import numpy as np

import config
import movimentos_bases

# =========================================================================
# AJUSTE FINO
# =========================================================================
LIMIAR   = 80 #80
USAR_CX_TOPO = True   # False - cx do contorno inteiro; True - cx so da parte de cima da linha (antecipa a curva)
FAIXA_CX_TOPO = 0.35  # fracao da altura da imagem, a partir do ponto mais alto da linha, usada pro cx
VEL_BASE_RAMPA = 35  # 45, 28 - velocidade base na rampa (PWM da placa satura em ~51)
VEL_BASE_GANGORRA = 25  # velocidade base na gangorra (mais baixa que a da rampa)
VEL_MAX_GANGORRA = 25  # 32 - gangorra (inclinacao nao passou de -20): roda rapida no maximo isso
VEL_MIN_GANGORRA = 5  # gangorra: roda lenta no minimo isso
VEL_TRAS_RAMPA_INGREME = 70  # rampa mais inclinada (passou de -20): rodas traseiras M2/M3 fixas nisso, as da frente normal (PID)
VEL_MIN_RAMPA = 10   #17, 0 - velocidade minima de cada roda na rampa
GAP_IGNORA_VERDE = True  # False - no gap, pixel verde nao conta como falta de linha (soma a area de preto)
CORTE_TOPO_ROI = 0.0  # 0.1 - fracao da imagem cortada em cima (0 = usa a imagem inteira)
AREA_MIN = 10000 # 11000
kernal = np.ones((3, 3), np.uint8) #3

Kp = 1.2  #0.9, 1.4, 0.9
Kd = 0.2  #0.3, 0.5, 0.35, 0.3
VEL_BASE =22  #20, 24, 21, 29, 21, 20
VEL_MAX = 22  #20, 21, 23, 21, 22, 30, 26 - teto de cada roda (vale pro robo inteiro, nao so' pro seguidor)
config.VEL_MAX = VEL_MAX   # o corte de verdade e' no config.aplicar_motor() - aqui so' manda o valor pra la
config.VEL_MIN = -VEL_MAX
DEADZONE = 0 #  0
DERIV_ALPHA = 0.6  #0.45, 0.5, 0.6, 0.5, 0.7
DT_MIN_DERIV = 0.03  #0.03
ALPHA = 1.2  #1.5, 1.4, 1.3, 1.4, 1.45, 1.5, 1.4, 1.5, 0.4 - 1.0 = sem filtro no erro (suspenso); so o DERIV_ALPHA filtra

# -------------------------------------------------------------------------
# SEGUIDOR POR ANGULO
# -------------------------------------------------------------------------
USAR_SEGUIDOR_ANGULO = False
ANG_VEL = 18        # 20, 16, 10 - velocidade base (equipe usa 80 numa escala de 100)
ANG_VEL_GANHO = 16
ANG_KP = 100.0      # 110.0, 100.0, ganho do angulo (rad) - da equipe, escalado por ANG_VEL/80
ANG_KD = 60.0       # 32.0, 60.0, 32, 28, 20, 12 (da equipe) - ganho da derivada do angulo (rad/s): amortece, segura o robo de balancar
ANG_FREIO = 3.5     # 4.0, 3.5, 6.0, 4.5, 3.5, 2.5 (da equipe) - a roda que freia freia isso vezes mais forte; maior = roda de dentro reverte mais na curva
ANG_ALCANCE = 0.62  # distancia (fracao da altura da imagem) de onde mais olha - 0.6 e' o da equipe
ANG_CURVA = 0.3    # 0.35, 0.45 - angulo (rad, ~26 graus) a partir do qual ele age com forca total (curva)
ANG_VEL_CURVA = 18   # 16, 18, 8, 12 (18 = igual a reta, sem queda de velocidade na curva) - velocidade de avanco na curva fechada (a reta continua em ANG_VEL). Menor = quase gira parado
ANG_TETO_CURVA = 26  # 20, 20 (26 = sem teto) - teto de velocidade de cada roda na curva fechada (igual ao VEL_MAX = sem teto). Menor = giro mais lento e suave
ANG_RE_MAX = 22      # 18, 16, 16 (a roda de dentro parou de reverter na curva), 26 = sem limite (re chega a -20) - limite da RE da roda de dentro na curva fechada (igual ao VEL_MAX = sem limite). Menor = a roda de dentro reverte mais devagar, gira menos
ANG_KI = 40.0        # 50.0, 0 -  ganho integral: corrige a tendencia de puxar pra um lado (roda mais forte que a outra) - so atua perto da reta
ANG_GANHO_RETA = 0.35 # 0.30, 0.20, 0.4 - fracao do ganho e do freio usada quando esta quase reto (1 = igual a curva) - menor = reta mais calma
USAR_SEGUIDOR_MPC = False
MPC_VEL = 16.48  #16 (+3%) - velocidade de avanco na reta
MPC_R = 0.05        # peso do esforco: maior = comando mais calmo, corrige mais devagar
MPC_SDU = 0.15      # peso da variacao do comando entre quadros: maior = rodas mais suaves
MPC_N = 15          # horizonte (quadros, 0.04 s cada)
MPC_Q_E = 1.0       # peso do erro da camera
MPC_Q_PSI = 0.5     # peso do angulo do robo
MPC_KG = 3.4        # modelo medido: graus/s de giro por unidade de diferenca entre as rodas
MPC_ATRASO = 3      # modelo medido: atraso do comando ao giro, em quadros (113 ms)
MPC_A_POR_S = 0.162 # modelo medido: %/s de erro por grau, por unidade de velocidade efetiva
MPC_Q_CAM = -0.25   # modelo medido: %/grau de efeito direto do giro no erro da camera
MPC_DZ = 1250 / 70  # zona morta da placa em unidades de comando ($deadzone:1250#)
MPC_FREIO = 0.5      # quanto a velocidade cai conforme o erro da camera cresce (0 = nao cai, 0.5 = metade quando o erro chega a 40%): freia aos poucos antes da curva
MPC_LOG = True      # grava /tmp/mpc_log.csv a cada rodada (cx, erro, estimativas, comando, rodas) pra diagnostico
MPC_U_MAX = 21      # diferenca maxima entre as rodas do MPC (duas rodas pra frente)
MPC_PD_KP = 0.6     # 0.9 - ganho do PD que assume sozinho quando o erro passa de MPC_ERRO_MAX
MPC_PD_KD = 0.0     # D do PD que soma (0 = sem D: o erro da camera pula nas curvas e o D amplifica em pulsos)
MPC_U_SLEW = 30     # variacao maxima do comando entre dois quadros (diferenca entre as rodas): corta os picos quando o erro da camera pula
MPC_ERRO_MAX = 20   # % de erro da camera: ate aqui so o MPC; acima o PD entra aos poucos
MPC_FADE = 20       # % de erro para o PD assumir 100% (MPC sai gradualmente, sem soma nem briga)
# -------------------------------------------------------------------------

green_min = np.array([17, 48, 39]) #40, 50, 45
green_max = np.array([89, 255, 255]) #85, 255, 255
black_min     = np.array([0, 0, 0])
black_max_top = np.array([78, 78, 78]) #90, 90, 90
black_max_bot = np.array([91, 91, 91]) #135, 135, 135
MIN_GREEN_AREA = 9200  #12000, 10000, 2500
LIMIAR_PROPORCAO_PRETO_LADO = 0.3  # 0.5 - afrouxado, exigia preto DEMAIS em cada lado pra contar
LIMIAR_PIXELS_VERDE_180 = 23000  # 22000, 23000 - pixel verde total acima disso = os dois marcadores juntos, vira 180 em vez de olhar o lado

CONFIRMACOES_ACAO_VERDE = 2
VEL_RE_APOS_VERDE = 17
TEMPO_RE_APOS_VERDE = 0.1   # pequena re no fim de tudo do verde (segundos). 0 desliga
TEMPO_SEGUIR_LINHA_APOS_VERDE = 0.5  # depois de TUDO do verde, so segue a linha por esse tempo (segundos) sem olhar verde de novo. 0 desliga
ALINHAR_APOS_VERDE = True


# =========================================================================
# ESTADO INTERNO
# =========================================================================
_trav_frame_ref = None
_trav_tempo_ref = time.time()
contador_gangorra = 0
_pico_rampa = 0  # angulo_y mais inclinado (mais negativo) da subida atual
_rampa_rapida = False  # ja trocou pra velocidade de rampa (35) nessa subida
_t_ingreme = None  # desde quando o angulo rapido esta em -20 ou menos
teve_gap_na_subida = False  # main.py marca True se caiu no gap durante a subida (topo da gangorra perde a linha)
_pico_lento = 0  # angulo_y() (o do print "inclinacao") mais inclinado da subida - separa rampa de gangorra
ultimo_erro = 0
_deriv_filtrada = 0.0
ultimo_tempo = time.time()
_ultima_acao_verde = None
_repeticoes_acao_verde = 0
_ang_integ = 0.0
_ang_ultimo = 0.0
_ang_tempo = time.time()
_ang_mapas = None       # mapas de angulo/peso por pixel, criados no 1o uso
_ultima_mask = None
_ultima_roi = None


# =========================================================================
# NUCLEO
# =========================================================================
def visao(frame):
    h_ = frame.shape[0]
    roi = frame[int(h_ * CORTE_TOPO_ROI):h_, :]

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx = None
    if contours:
        main = max(contours, key=cv2.contourArea)
        if USAR_CX_TOPO:
            y0 = cv2.boundingRect(main)[1]
            y1 = y0 + max(int(mask.shape[0] * FAIXA_CX_TOPO), 4)
            faixa = np.zeros_like(mask)
            cv2.drawContours(faixa, [main], -1, 255, -1)
            faixa[y1:, :] = 0
            M = cv2.moments(faixa, binaryImage=True)
        else:
            M = cv2.moments(main)
        if M["m00"] != 0:
            # Sem int(): arredondar pro pixel fazia o cx pular 1 pixel de um
            # quadro pro outro quando a linha ficava entre dois pixels, e o
            # filtro do erro (ALPHA > 1) amplificava isso em tremor.
            cx = M["m10"] / M["m00"]

    global _ultima_mask, _ultima_roi
    _ultima_mask = mask
    _ultima_roi = roi
    return cx, contours, roi, mask


ANG_W, ANG_H = 80, 54   # mascara reduzida pra esse tamanho (mantem a proporcao do ROI 320x216)


def _criar_mapas_angulo():
    cx0, cy0 = ANG_W / 2, ANG_H
    ys, xs = np.mgrid[0:ANG_H, 0:ANG_W].astype(np.float32)
    dx, dy = xs - cx0, ys - cy0
    dist = np.sqrt(dx * dx + dy * dy)
    ang = np.arctan2(dy, dx) + np.pi / 2
    e = 3.25 * (dist / ANG_H) - 2.0 * (ANG_ALCANCE / 0.615)   # pico em ~ANG_ALCANCE da altura
    w = np.maximum(np.exp(-e * e) - 0.1, 0.0)
    w[ys > ANG_H - int(ANG_H * 0.15)] = 0.0                    # faixa de baixo ignorada (a equipe corta 7 de 48)
    w *= np.exp(-ang * ang) / 2.0 + 0.5                        # leve preferencia pelo centro
    return ang.astype(np.float32), w.astype(np.float32)


def _angulo_da_linha(mask, ang_ultimo):
    global _ang_mapas
    if _ang_mapas is None:
        _ang_mapas = _criar_mapas_angulo()
    ang, w_dist = _ang_mapas
    pequena = cv2.resize(mask, (ANG_W, ANG_H), interpolation=cv2.INTER_AREA) > 127
    peso = w_dist * pequena
    if np.count_nonzero(peso) < 80:                            # LINE_MIN_NUM_ANGLES da equipe
        return None
    d = (ang - ang_ultimo) / np.pi * 2.0
    peso = peso * np.clip(0.25 + 0.75 * np.exp(-16.0 * d * d), 0.0, 1.0)   # fica na mesma linha
    total = float(peso.sum())
    if total <= 0.0:
        return None
    return float((peso * ang).sum() / total)


def _controle_angulo(cx, contours, roi):
    global _ang_ultimo, _ang_tempo, _ang_integ, ultimo_erro, ultimo_tempo

    agora = time.time()
    dt = agora - _ang_tempo
    _ang_tempo = agora
    voltou_de_pausa = dt > 0.25 or dt <= 0
    if voltou_de_pausa:
        dt = 1 / 30
        _ang_ultimo = 0.0
        _ang_integ = 0.0

    mask = _ultima_mask
    if mask is None or mask.shape[:2] != roi.shape[:2]:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, LIMIAR, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

    angulo = None if gap_debug(contours) else _angulo_da_linha(mask, _ang_ultimo)
    if angulo is None:
        angulo, deriv = 0.0, 0.0
    elif voltou_de_pausa:
        deriv = 0.0
    else:
        deriv = float(np.clip((angulo - _ang_ultimo) / dt, -2.5, 2.5))

    esc = ANG_VEL_GANHO / 80.0                      # ganhos da equipe sao pra base 80
    t = min(abs(angulo) / ANG_CURVA, 1.0) ** 2
    ganho = ANG_GANHO_RETA + (1.0 - ANG_GANHO_RETA) * t
    freio = 1.0 + (ANG_FREIO - 1.0) * t
    if abs(angulo) < ANG_CURVA * 0.5:
        _ang_integ = float(np.clip(_ang_integ + angulo * dt, -0.15, 0.15))
    else:
        _ang_integ *= 0.9
    u = (ANG_KP * ganho * angulo + ANG_KD * deriv + ANG_KI * _ang_integ) * esc
    u_esq = u * freio if u < 0 else u
    u_dir = -u * freio if -u < 0 else -u
    avanco = ANG_VEL + (ANG_VEL_CURVA - ANG_VEL) * t
    teto = config.VEL_MAX + (min(ANG_TETO_CURVA, config.VEL_MAX) - config.VEL_MAX) * t
    re_max = config.VEL_MAX + (min(ANG_RE_MAX, config.VEL_MAX) - config.VEL_MAX) * t   # limite da re (so na curva)
    piso = -min(teto, re_max)
    velL = float(np.clip(avanco + u_esq, piso, teto))
    velR = float(np.clip(avanco + u_dir, piso, teto))
    config.mover(velL, velR)

    _ang_ultimo = angulo
    ultimo_erro = angulo
    ultimo_tempo = agora
    return u, contours


class _MPC:
    def __init__(s, vel, KG, ATRASO, A_POR_S, DZ, N, q_e, q_psi, r, sdu, h=0.04, U_MAX=21.0, sig_e=1.0, q_psi_n=4.0, q_b_n=0.05, q_e_n=0.05, q_cam=0.0):
        s.h = h; s.N = N; s.d = ATRASO; s.KG = KG; s.umax = U_MAX; s.qc = q_cam
        a = A_POR_S * (DZ + vel); n = 3 + ATRASO
        A = np.zeros((n, n)); A[0, 0] = 1; A[0, 1] = -h * a; A[1, 1] = 1; A[1, 2] = h; A[2, 2] = 1; A[1, n - 1] = h * KG
        for i in range(ATRASO - 1): A[4 + i, 3 + i] = 1
        B = np.zeros((n, 1)); B[3, 0] = 1
        keep = [0, 1] + list(range(3, n)); Ak = A[np.ix_(keep, keep)]; Bk = B[keep]
        Qk = np.zeros((len(keep), len(keep))); Qk[0, 0] = q_e; Qk[0, 1] = Qk[1, 0] = q_e * q_cam; Qk[1, 1] = q_psi + q_e * q_cam ** 2
        P = Qk.copy(); Rr = np.array([[r + sdu]])
        for _ in range(2000):
            Pn = Qk + Ak.T @ P @ Ak - Ak.T @ P @ Bk @ np.linalg.inv(Rr + Bk.T @ P @ Bk) @ Bk.T @ P @ Ak
            if np.abs(Pn - P).max() < 1e-9: P = Pn; break
            P = Pn
        Pf = np.zeros((n, n)); Pf[np.ix_(keep, keep)] = P
        Q = np.zeros((n, n)); Q[0, 0] = q_e; Q[0, 1] = Q[1, 0] = q_e * q_cam; Q[1, 1] = q_psi + q_e * q_cam ** 2
        Phi = []; Gam = np.zeros((n * N, N)); Ai = np.eye(n)
        for i in range(N):
            Ai = A @ Ai; Phi.append(Ai)
            for j in range(i + 1): Gam[n * i:n * (i + 1), j:j + 1] = np.linalg.matrix_power(A, i - j) @ B
        s.Phi = np.vstack(Phi); Qb = np.zeros((n * N, n * N))
        for i in range(N): Qb[n * i:n * (i + 1), n * i:n * (i + 1)] = (Pf if i == N - 1 else Q)
        D = np.eye(N) - np.eye(N, k=-1)
        s.M = Gam.T @ Qb @ Gam + r * np.eye(N) + sdu * D.T @ D
        s.GQP = Gam.T @ Qb @ s.Phi; s.D = D; s.r = r; s.sdu = sdu
        s.Minv = np.linalg.inv(s.M); s.step = 1.0 / np.linalg.eigvalsh(2 * s.M).max()
        Ax = np.array([[1, -h * a, 0], [0, 1, h], [0, 0, 1.0]]); Cx = np.array([[1.0, q_cam, 0]])
        Qx = np.diag([q_e_n, q_psi_n * h, q_b_n * h]); Rx = np.array([[sig_e ** 2]]); Pp = np.eye(3)
        for _ in range(3000):
            Pm = Ax @ Pp @ Ax.T + Qx; K = Pm @ Cx.T @ np.linalg.inv(Cx @ Pm @ Cx.T + Rx); Pn = (np.eye(3) - K @ Cx) @ Pm
            if np.abs(Pn - Pp).max() < 1e-10: Pp = Pn; break
            Pp = Pn
        s.Ax = Ax; s.Lk = K; s.Bx = np.array([0, h * KG, 0.0])
        s.reset()

    def reset(s):
        s.x = np.zeros(3); s.hist = [0.0] * (s.d + 1); s.iniciado = False; s.Uprev = np.zeros(s.N)

    def passo(s, e_meas):
        if not s.iniciado:
            s.x = np.array([e_meas, 0.0, 0.0]); s.iniciado = True
        else:
            xp = s.Ax @ s.x + s.Bx * s.hist[-1]
            s.x = xp + s.Lk[:, 0] * (e_meas - (xp[0] + s.qc * xp[1]))
        z0 = np.concatenate([s.x, np.array(s.hist[:s.d])])
        u_ss = float(np.clip(-s.x[2] / s.KG, -s.umax, s.umax))
        c = np.zeros(s.N); c[0] = s.hist[0]
        m = s.GQP @ z0 - s.r * u_ss * np.ones(s.N) - s.sdu * (s.D.T @ c)
        U = -s.Minv @ m
        if np.abs(U).max() > s.umax:
            U = np.clip(np.concatenate([s.Uprev[1:], s.Uprev[-1:]]) if s.Uprev.any() else U, -s.umax, s.umax)
            for _ in range(40):
                U = np.clip(U - s.step * 2 * (s.M @ U + m), -s.umax, s.umax)
        s.Uprev = U
        return float(U[0])

    def aplicou(s, u_real):
        s.hist = [u_real] + s.hist[:-1]


_mpc = None
_mpc_chave = None
_mpc_tempo = time.time()
_mpc_u_ant = 0.0


def _mpc_obter():
    global _mpc, _mpc_chave
    chave = (MPC_VEL, MPC_KG, MPC_ATRASO, MPC_A_POR_S, MPC_DZ, MPC_N, MPC_Q_E, MPC_Q_PSI, MPC_R, MPC_SDU, MPC_Q_CAM, MPC_U_MAX)
    if _mpc is None or chave != _mpc_chave:
        _mpc = _MPC(MPC_VEL, MPC_KG, int(MPC_ATRASO), MPC_A_POR_S, MPC_DZ, int(MPC_N), MPC_Q_E, MPC_Q_PSI, MPC_R, MPC_SDU, U_MAX=MPC_U_MAX, q_cam=MPC_Q_CAM)
        _mpc_chave = chave
    return _mpc


def _mpc_efetiva(c):
    return 0.0 if abs(c) < 1 / 70 else (1 if c > 0 else -1) * (MPC_DZ + abs(c))


def _mpc_rodas(u, base, vmax):
    a = abs(u)
    if a <= vmax - 1:
        be = max(min(base, vmax - a / 2), 1 + a / 2)
        grande, pequena = be + a / 2, be - a / 2
    else:
        grande, pequena = float(vmax), max(vmax - a, -float(vmax))
    return (grande, pequena) if u >= 0 else (pequena, grande)


_mpc_log = None
_mpc_log_n = 0
_mpc_log_t0 = 0.0


def _mpc_registrar(linha):
    global _mpc_log, _mpc_log_n, _mpc_log_t0
    if not MPC_LOG:
        return
    try:
        if _mpc_log is None:
            _mpc_log = open('/tmp/mpc_log.csv', 'w')
            _mpc_log.write('t,cx,e,e_est,psi_est,vies_est,u,L,R,vel_base,sem_linha,pd_extra\n')
            _mpc_log_t0 = time.time()
        _mpc_log.write(','.join(f'{x:.3f}' for x in ((time.time() - _mpc_log_t0,) + linha)) + '\n')
        _mpc_log_n += 1
        if _mpc_log_n % 20 == 0:
            _mpc_log.flush()
    except Exception:
        pass


def _controle_mpc(cx, contours, roi):
    global _mpc_tempo, _mpc_u_ant, ultimo_erro, ultimo_tempo
    mpc = _mpc_obter()
    agora = time.time()
    dt = agora - _mpc_tempo
    _mpc_tempo = agora
    if dt > 0.25 or dt <= 0:
        mpc.reset()
        dt = 0.04

    centro = roi.shape[1] / 2
    sem_linha = cx is None or gap_debug(contours)
    e_meas = 0.0
    base = float(MPC_VEL)
    pd_extra = 0.0

    if sem_linha:
        velL = velR = base; u = 0.0
        mpc.aplicou(0.0)
    else:
        e_meas = float(np.clip((cx - centro) / centro * 100, -100, 100))
        u = mpc.passo(e_meas)
        erro_f = mpc.x[0] + MPC_Q_CAM * mpc.x[1]
        peso = float(np.clip((abs(erro_f) - MPC_ERRO_MAX) / MPC_FADE, 0.0, 1.0))
        if peso:
            pd_extra = MPC_PD_KP * erro_f
            u = (1 - peso) * u + peso * 2 * pd_extra
        ultimo_erro = erro_f
        u = _mpc_u_ant + float(np.clip(u - _mpc_u_ant, -MPC_U_SLEW, MPC_U_SLEW))
        base = MPC_VEL * (1 - MPC_FREIO * min(abs(mpc.x[0] + MPC_Q_CAM * mpc.x[1]) / 40, 1.0))
        velL, velR = _mpc_rodas(u, base, config.VEL_MAX)
        mpc.aplicou(_mpc_efetiva(velL) - _mpc_efetiva(velR))

    _mpc_u_ant = u
    config.mover(velL, velR)
    ultimo_tempo = agora
    _mpc_registrar((cx if cx is not None else -1, e_meas, mpc.x[0], mpc.x[1], mpc.x[2], u, velL, velR, base, int(sem_linha), pd_extra))
    return u, contours


def controle(cx, contours, roi):
    global ultimo_erro, ultimo_tempo, contador_gangorra, _deriv_filtrada, _pico_rampa, _pico_lento, teve_gap_na_subida
    global _rampa_rapida, _t_ingreme

    if USAR_SEGUIDOR_MPC and contador_gangorra != 2 and config.angulo_y() > -10:
        return _controle_mpc(cx, contours, roi)

    if USAR_SEGUIDOR_ANGULO and contador_gangorra != 2 and config.angulo_y() > -10:
        return _controle_angulo(cx, contours, roi)

    w = roi.shape[1]
    tempo_atual = time.time()
    dt = tempo_atual - ultimo_tempo
    if dt <= 0:
        dt = 0.0001

    if cx is not None:
        centro = w // 2
        erro_novo = (cx - centro) / centro * 100
        erro = ALPHA * erro_novo + (1 - ALPHA) * ultimo_erro
    else:
        erro = ultimo_erro

    if gap_debug(contours):
        erro = 0

    derivada = (erro - ultimo_erro) / max(dt, DT_MIN_DERIV)
    derivada = np.clip(derivada, -300, 300)
    _deriv_filtrada = DERIV_ALPHA * derivada + (1 - DERIV_ALPHA) * _deriv_filtrada
    derivada = _deriv_filtrada

    if abs(erro) < DEADZONE:
        ajuste = 0
    else:
        ajuste = (Kp * erro) + (Kd * derivada)

    print("inclinacao", config.angulo_y())

    # contador_gangorra: 0 = plano | 2 = subindo a rampa | 1 = acabou de sair
    # (so volta pra 0, e pode detectar rampa de novo, depois de ficar plano).
    ang_y = config.angulo_y()
    # Angulo rapido atualizado todo loop (sem o atraso do filtro 0.97).
    ang_rap = config.angulo_y_rapido()
    if contador_gangorra == 1 and ang_y > -5:
        contador_gangorra = 0
    if contador_gangorra == 2 or (contador_gangorra == 0 and ang_y <= -9):  # -10
        if contador_gangorra != 2:
            _pico_rampa = config.angulo_y_rapido()
            _pico_lento = ang_y
            teve_gap_na_subida = False
            _rampa_rapida = False
            _t_ingreme = None
        contador_gangorra = 2
        _pico_lento = min(_pico_lento, ang_y)
        config.rampa_ativa = True
        velocidade_base = VEL_BASE_RAMPA
        # Troca pra velocidade de rampa quando o angulo RAPIDO fica em -20 ou
        # menos por 0.2 s seguidos (um pico solto nao conta). Depois que
        # trocou, fica rapido ate sair da rampa.
        if ang_rap <= -20:
            if _t_ingreme is None:
                _t_ingreme = time.time()
            elif time.time() - _t_ingreme >= 0.2 and not _rampa_rapida:
                _rampa_rapida = True
                print(f"[RAMPA] Ingreme ({ang_rap:.1f}) - velocidade de rampa")
        else:
            _t_ingreme = None
        if not _rampa_rapida:
            # Ainda nao ficou mais inclinado que -20 = sobe a 25 (gangorra).
            velocidade_base = VEL_BASE_GANGORRA
            velL = np.clip(velocidade_base + ajuste, VEL_MIN_GANGORRA, VEL_MAX_GANGORRA)
            velR = np.clip(velocidade_base - ajuste, VEL_MIN_GANGORRA, VEL_MAX_GANGORRA)
        else:
            velL = np.clip(velocidade_base + ajuste, VEL_MIN_RAMPA, 80)
            velR = np.clip(velocidade_base - ajuste, VEL_MIN_RAMPA, 80)
    else:
        config.rampa_ativa = False
        velocidade_base = VEL_BASE
        velL = np.clip(velocidade_base + ajuste, config.VEL_MIN, config.VEL_MAX)
        velR = np.clip(velocidade_base - ajuste, config.VEL_MIN, config.VEL_MAX)

    if contador_gangorra == 2:
        # Angulo com filtro leve (angulo_y_rapido) pra perceber a saida rapido;
        # o angulo_y() normal (filtro 0.97) demora pra acompanhar.
        ang_lento = ang_y  # o da "inclinacao" do print (filtro 0.97)
        ang_y = ang_rap
        _pico_rampa = min(_pico_rampa, ang_y)
        # Saiu da rampa: pela inclinacao LENTA (a do print), que quase nao
        # mexe com a abaixadinha depois da lombada - o angulo rapido pulava e
        # fazia ele frear e dar re no meio da rampa. Acabou a rampa quando a
        # inclinacao, depois de ter passado de -18, volta pra mais de -16.
        # Se a subida nunca passou de -18, sai quando ficar quase plano (> -5).
        if (_pico_lento <= -18 and ang_lento > -16) or ang_lento > -5:
            # Gangorra: a inclinacao nunca passou de -20 na subida, OU a camera
            # perdeu a linha (gap) durante a subida - no topo da gangorra ela
            # perde a linha (no log a gangorra chegou em -21 e teve gap antes
            # de sair). Hoje so muda o print - freia + re nas duas.
            gangorra = _pico_lento > -20 or teve_gap_na_subida
            print(f"[RAMPA] Saiu da {'GANGORRA' if gangorra else 'RAMPA'} (inclinacao max {_pico_lento}, gap na subida {teve_gap_na_subida}, pico rapido {_pico_rampa:.1f}, inclinacao agora {ang_lento}) - freando")
            contador_gangorra = 1
            config.rampa_ativa = False
            config.parar()
            time.sleep(1.0)  # parado 1 s antes da re
            # Re devagar ate a camera ver a linha de novo (sair do gap), na rampa
            # e na gangorra. Minimo 0.15 s de re, maximo 3 s (se nunca achar a linha).
            config.mover(-12, -12)  # re devagar (era -20)
            inicio_re = time.time()
            frames_com_linha = 0
            while time.time() - inicio_re < 3.0:
                config.checar_parada()
                ret, frame_re = config.cap.read()
                if not ret:
                    continue
                _, contours_re, _, _ = visao(frame_re)
                if gap_debug(contours_re):
                    frames_com_linha = 0
                else:
                    frames_com_linha += 1
                    # 2 frames seguidos com linha = saiu do gap (frame velho do buffer nao engana)
                    if frames_com_linha >= 2 and time.time() - inicio_re >= 0.15:
                        break
            # Achou a linha: mais 1.0 s de re a 12 (era 0.5).
            t_achou = time.time() - inicio_re
            config.mover(-12, -12)
            fim_extra = time.time() + 1.0
            while time.time() < fim_extra:
                config.checar_parada()
                time.sleep(0.02)
            print(f"[RAMPA] Re por {time.time() - inicio_re:.2f}s ({'achou a linha' if frames_com_linha >= 2 else 'tempo maximo'} em {t_achou:.2f}s, +1.0s)")
            config.parar()
            time.sleep(0.6)  # estabiliza parado (era 0.2)
            velL = np.clip(VEL_BASE + ajuste, config.VEL_MIN, config.VEL_MAX)
            velR = np.clip(VEL_BASE - ajuste, config.VEL_MIN, config.VEL_MAX)


    # Rampa mais inclinada: traseiras fixas em VEL_TRAS_RAMPA_INGREME, frente normal.
    # So nesse comando: qualquer outro (parar, pausa, gap) usa as 4 rodas normal.
    vel_tras = VEL_TRAS_RAMPA_INGREME if (contador_gangorra == 2 and _rampa_rapida) else None
    config.mover(velL, velR, vel_tras)
    ultimo_erro = erro

    ultimo_tempo = tempo_atual
    return ajuste, contours


def check_black(black_around_sign, i, green_box, black_image):
    gb_y = green_box[green_box[:, 1].argsort()]
    marker_height = int(gb_y[-1][1] - gb_y[0][1])
    sign_y = int(gb_y[2][1])
    black_around_sign[i, 4] = sign_y

    y1, y2 = sign_y, min(sign_y + int(marker_height * 0.8), config.camera_y)
    x1, x2 = int(min(gb_y[2][0], gb_y[3][0])), int(max(gb_y[2][0], gb_y[3][0]))
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * LIMIAR_PROPORCAO_PRETO_LADO:
        black_around_sign[i, 0] = 1

    y1, y2 = max(int(gb_y[1][1]) - int(marker_height * 0.8), 0), int(gb_y[1][1])
    x1, x2 = max(int(min(gb_y[0][0], gb_y[1][0])), 0), max(int(max(gb_y[0][0], gb_y[1][0])), 0)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * LIMIAR_PROPORCAO_PRETO_LADO:
        black_around_sign[i, 1] = 1

    gb_x = green_box[green_box[:, 0].argsort()]

    y1, y2 = int(min(gb_x[0][1], gb_x[1][1])), int(max(gb_x[0][1], gb_x[1][1]))
    x1, x2 = max(int(gb_x[1][0]) - int(marker_height * 0.8), 0), int(gb_x[1][0])
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * LIMIAR_PROPORCAO_PRETO_LADO:
        black_around_sign[i, 2] = 1

    y1, y2 = int(min(gb_x[2][1], gb_x[3][1])), int(max(gb_x[2][1], gb_x[3][1]))
    x1, x2 = int(gb_x[2][0]), min(int(gb_x[2][0]) + int(marker_height * 0.8), config.camera_x)
    r = black_image[y1:y2, x1:x2]
    if r.size > 0 and np.count_nonzero(r) > r.size * LIMIAR_PROPORCAO_PRETO_LADO:
        black_around_sign[i, 3] = 1

    return black_around_sign


def check_green(contours_grn, black_image, green_image):
    """So os marcadores VALIDOS (com preto em cima = antes do cruzamento)
    contam. Os que tem a linha embaixo (depois do cruzamento, pra ignorar)
    nao entram nem no lado nem no 180 - antes o verde deles entrava na
    contagem e fazia ele girar 180 ou virar pro lado errado."""
    black_around_sign = np.zeros((len(contours_grn), 5), dtype=np.int16)

    for i, contour in enumerate(contours_grn):
        if cv2.contourArea(contour) <= MIN_GREEN_AREA:
            continue
        green_box = cv2.boxPoints(cv2.minAreaRect(contour))
        black_around_sign = check_black(black_around_sign, i, green_box, black_image)

    roi_w = green_image.shape[1]
    meio = roi_w // 2
    validos_esq = 0  # pixels de marcador valido a esquerda / direita do meio
    validos_dir = 0
    n_esq = n_dir = 0
    for i, contour in enumerate(contours_grn):
        area = cv2.contourArea(contour)
        if area <= MIN_GREEN_AREA:
            continue
        baixo, cima, esq, dir_ = (int(v) for v in black_around_sign[i, :4])
        # Valido = preto EM CIMA (a linha do cruzamento esta depois dele) e
        # SEM preto embaixo. Marcador depois do cruzamento tem a linha
        # embaixo - se tiver preto embaixo e ignorado, mesmo que pareca ter
        # preto em cima.
        valido = cima == 1 and baixo == 0
        print(f"[VERDE-MARCADOR] area={int(area)} preto: cima={cima} baixo={baixo} esq={esq} dir={dir_} -> {'VALIDO' if valido else 'ignora'}")
        if not valido:
            continue
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cx_marcador = M["m10"] / M["m00"]
        if cx_marcador < meio:
            validos_esq += area
            n_esq += 1
        else:
            validos_dir += area
            n_dir += 1

    if n_esq == 0 and n_dir == 0:
        return "RETO"

    print(f"[VERDE-LADO] validos esquerda={n_esq} ({int(validos_esq)}px) direita={n_dir} ({int(validos_dir)}px)")

    # 180: marcador valido dos DOIS lados, ou um blob valido so grande demais
    # (os dois marcadores grudados).
    if (n_esq and n_dir) or (validos_esq + validos_dir) > LIMIAR_PIXELS_VERDE_180:
        return "GIRAR_180"
    return "ESQUERDA" if n_esq else "DIREITA"


def detectar_verde(roi):
    roi_h = roi.shape[0]
    corte_top = int(roi_h * 0.4)

    black_image = cv2.inRange(roi, black_min, black_max_bot)
    black_image[:corte_top, :] = cv2.inRange(roi[:corte_top, :], black_min, black_max_top)
    black_image = cv2.dilate(black_image, kernal, iterations=3)
    black_image = cv2.erode(black_image, kernal, iterations=2)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    green_image = cv2.inRange(hsv, green_min, green_max)
    green_image = cv2.dilate(green_image, kernal, iterations=3)
    green_image = cv2.erode(green_image, kernal, iterations=2)

    contours_grn, _ = cv2.findContours(green_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours_grn) == 0:
        return None

    # DEBUG
    areas = sorted((cv2.contourArea(c) for c in contours_grn), reverse=True)
    validos = sum(1 for a in areas if a > MIN_GREEN_AREA)
    # print(f"[VERDE-BLOBS] total={len(contours_grn)} validos={validos} areas={[int(a) for a in areas[:4]]}")

    acao = check_green(contours_grn, black_image, green_image)
    return acao if acao != "RETO" else None


def verde(roi):
    global _ultima_acao_verde, _repeticoes_acao_verde

    acao_verde = detectar_verde(roi)
    if not acao_verde:
        _ultima_acao_verde = None
        _repeticoes_acao_verde = 0
        return

    if acao_verde == _ultima_acao_verde:
        _repeticoes_acao_verde += 1
    else:
        _ultima_acao_verde = acao_verde
        _repeticoes_acao_verde = 1

    if _repeticoes_acao_verde < CONFIRMACOES_ACAO_VERDE:
        print(f"[VERDE-CONFIRMA] acao={acao_verde} repeticoes={_repeticoes_acao_verde}/{CONFIRMACOES_ACAO_VERDE}, esperando confirmar")
        return

    _repeticoes_acao_verde = 0

    config.parar()

    if acao_verde == "ESQUERDA":
        print("verde_esquerda")
        config.mover(-2, -2)
        time.sleep(0.2)
        config.mover(15, 15)
        time.sleep(0.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        movimentos_bases.guinada2('E', 82, 16)
        config.mover(15, 15)
        time.sleep(0.2)
        config.mover(-2, -2)
        time.sleep(0.1)

    elif acao_verde == "DIREITA":
        print("verde_direita")
        limpar_camera(10)
        config.mover(-2, -2)
        time.sleep(0.2)
        config.mover(15, 15)
        time.sleep(0.3)
        config.mover(-2, -2)
        time.sleep(0.1)
        movimentos_bases.guinada2('D', 82, 16)
        config.mover(15, 15)
        time.sleep(0.2)
        config.mover(-2, -2)
        time.sleep(0.1)

    elif acao_verde == "GIRAR_180":
        print("gira 180")

        movimentos_bases.guinada2("D", 174, 16)
        config.mover(15, 15)
        time.sleep(0.2)
        config.mover(-2, -2)
        
        limpar_camera(10)

    if ALINHAR_APOS_VERDE:
        config.parar()
        time.sleep(0.15)       # robo assentar
        limpar_camera(3)
        alinhar_robo_na_linha()

    if TEMPO_RE_APOS_VERDE > 0:
        config.mover(-VEL_RE_APOS_VERDE, -VEL_RE_APOS_VERDE)
        time.sleep(TEMPO_RE_APOS_VERDE)
        config.parar()

    if TEMPO_SEGUIR_LINHA_APOS_VERDE > 0:
        limpar_camera(2)       # descarta frames velhos do buffer
        inicio_linha = time.time()
        while time.time() - inicio_linha < TEMPO_SEGUIR_LINHA_APOS_VERDE:
            config.checar_parada()
            ret, frame = config.cap.read()
            if not ret:
                continue
            cx, contours, roi_loop, _ = visao(frame)
            controle(cx, contours, roi_loop)


# =========================================================================
# APOIO
# =========================================================================
def gap_debug(contours):
    if len(contours) == 0:
        return True
    maior = max(contours, key=cv2.contourArea)
    area_ = cv2.contourArea(maior)
    if area_ < AREA_MIN:
        if GAP_IGNORA_VERDE and _ultima_roi is not None:
            verde_px = cv2.countNonZero(cv2.inRange(cv2.cvtColor(_ultima_roi, cv2.COLOR_BGR2HSV), green_min, green_max))
            return area_ + verde_px < AREA_MIN
        return True
    return False


DEBUG_ANGULO_GAP = False

ALONGAMENTO_MIN_ANGULO_GAP = 1.5


def angulo_linha_gap(contours):
    if not contours:
        return None
    maior = max(contours, key=cv2.contourArea)

    (_, _), (larg, alt), _ = cv2.minAreaRect(maior)
    maior_lado = max(larg, alt)
    menor_lado = max(min(larg, alt), 1.0)
    if maior_lado / menor_lado < ALONGAMENTO_MIN_ANGULO_GAP:
        return None

    vx, vy, _, _ = cv2.fitLine(maior, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
    angulo = float(np.degrees(np.arctan2(float(vx), float(vy))))

    if angulo > 90:
        angulo -= 180
    elif angulo <= -90:
        angulo += 180
    return angulo


VELOCIDADE_ALINHAR_NA_LINHA = 14


def alinhar_robo_na_linha():
    ret, frame = config.cap.read()
    if not ret:
        print("[ALINHAR] Sem frame da camera")
        return None

    _, contours, roi, mask = visao(frame)
    verde = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), green_min, green_max)
    verde = cv2.dilate(verde, kernal, iterations=2)
    mask_preta = cv2.bitwise_and(mask, cv2.bitwise_not(verde))
    mask_preta = cv2.morphologyEx(mask_preta, cv2.MORPH_OPEN, kernal)
    contours, _ = cv2.findContours(mask_preta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    angulo = angulo_linha_gap(contours)
    if angulo is None:
        print("[ALINHAR] Angulo nao confiavel (contorno pouco alongado), sem corrigir")
        return None

    print(f"[ALINHAR] Corrigindo angulo={angulo:.1f} graus")
    if angulo > 0:
        movimentos_bases.guinada2('E', abs(angulo), VELOCIDADE_ALINHAR_NA_LINHA)
    elif angulo < 0:
        movimentos_bases.guinada2('D', abs(angulo), VELOCIDADE_ALINHAR_NA_LINHA)
    return angulo


def detectar_travamento(roi):
    global _trav_frame_ref, _trav_tempo_ref
    LIMIAR_DIFF = 8
    TEMPO_TRAV = 4.0
    INTERVALO_REF = 2.0

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    agora = time.time()

    if _trav_frame_ref is None:
        _trav_frame_ref = gray.copy()
        _trav_tempo_ref = agora
        return False

    movimento = np.mean(cv2.absdiff(gray, _trav_frame_ref))

    if agora - _trav_tempo_ref >= INTERVALO_REF:
        if movimento >= LIMIAR_DIFF:
            _trav_frame_ref = gray.copy()
            _trav_tempo_ref = agora
            return False
        else:
            tempo_parado = agora - _trav_tempo_ref
            print(f"[TRAV] diff={movimento:.1f} | parado ha {tempo_parado:.1f}s")
            if tempo_parado >= TEMPO_TRAV:
                print("[TRAVAMENTO] Robo travado!")
                _trav_frame_ref = None
                _trav_tempo_ref = agora
                return True
    return False


def detectar_fita_vermelha(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
    )
    h_, w_ = mask.shape
    for y in range(0, h_, 5):
        if cv2.countNonZero(mask[y, :]) > w_ * 0.6:
            print("[VERMELHO] Fita detectada - pausando 6s")
            config.parar()
            time.sleep(6)
            limpar_camera(10)
            return True
    return False


def limpar_camera(n=5):
    for _ in range(n):
        config.cap.read()


def resetar_estado():
    global _trav_frame_ref, _trav_tempo_ref
    global contador_gangorra, ultimo_erro, ultimo_tempo, _mpc_u_ant, _deriv_filtrada, _pico_rampa, _pico_lento, teve_gap_na_subida

    _trav_frame_ref = None
    _trav_tempo_ref = time.time()

    contador_gangorra = 0
    _pico_rampa = 0
    _pico_lento = 0
    teve_gap_na_subida = False
    global _rampa_rapida, _t_ingreme
    _rampa_rapida = False
    _t_ingreme = None
    config.rampa_ativa = False
    ultimo_erro = 0
    _deriv_filtrada = 0.0
    ultimo_tempo = time.time()
    _mpc_u_ant = 0.0
    if USAR_SEGUIDOR_MPC:
        _mpc_obter().reset()

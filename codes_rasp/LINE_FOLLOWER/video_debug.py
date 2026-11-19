import json
import os
import py_compile
import re
import threading
import time
from urllib.parse import urlparse, parse_qs
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PORTA = 8081
QUALIDADE_JPEG = 70
ESCALA = 2  # 1, 2 - tamanho da imagem enviada

_ultimo_jpeg = [None]
_clientes = [0]
_iniciado = [False]
_lock = threading.Lock()

# Setas na pagina: so mexem o robo com ele PAUSADO. Anda enquanto a seta
# estiver apertada, soltou = freia. A pagina reenvia a seta a cada 0.1 s
# enquanto segura; se parar de chegar (aba perdeu foco, wifi caiu) ele freia.
VEL_SETAS_ANDAR = 20
VEL_SETAS_GIRO = 16
_setas = {"dir": None, "t": 0.0}

# Garra (servo pesado, PINO_PESADO) pelo teclado - so com o robo PAUSADO.
# S = sobe, D = desce, continuo enquanto segura; soltou = para onde esta.
# Sobe = angulo maior (150 = levantada no resgate), desce = menor (0 = chao).
VEL_GARRA_GRAUS_S = 90  # graus por segundo subindo enquanto segura a tecla
VEL_GARRA_DESCE_GRAUS_S = 250  # 90 - graus por segundo descendo (nao usado: D vai direto pro GARRA_ANG_BAIXO)
# D = vai direto pra posicao mais baixa. Pode ser negativo (abaixo do 0 do
# resgate) pra testar se o servo desce mais: -6 = pulso de ~433us (o 0 e 500us).
GARRA_ANG_BAIXO = -10  # -6
GARRA_ANG_MIN = 0
GARRA_ANG_MAX = 180
_garra = {"dir": None, "t": 0.0}

# nome da variavel em seguimento_de_linha: (minimo, maximo, passo)
AJUSTES = {
    "Kp": (0.0, 3.0, 0.05),
    "Kd": (0.0, 1.5, 0.05),
    "ALPHA": (0.2, 2.0, 0.05),
    "DERIV_ALPHA": (0.1, 1.0, 0.05),
    "DT_MIN_DERIV": (0.001, 0.06, 0.001),
    "VEL_BASE": (5, 30, 1),
    "DEADZONE": (0, 15, 1),
    "VEL_MAX": (10, 40, 1),
    "MIN_GREEN_AREA": (0, 30000, 100),
}

NO_CONFIG = ("VEL_MAX",)  # essas moram em config.py, nao em seguimento_de_linha

_PAGINA = """<html><head><meta name='viewport' content='width=device-width'></head>
<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>
<button id='bt' style='display:block;width:100%;max-width:960px;margin:8px auto;padding:14px;font-size:18px'>ESPACO = botao do robo</button>
<button id='sv' style='display:block;width:100%;max-width:960px;margin:8px auto;padding:12px;font-size:16px'>SALVAR NO CODIGO</button>
<div style='max-width:960px;margin:auto;padding:4px 8px;color:#aaa'>SETAS (so com o robo PAUSADO): cima/baixo = frente/tras, esquerda/direita = gira. Segura pra andar, solta pra frear. GARRA: S = sobe, D = desce (segura).</div>
<div id='p' style='max-width:960px;margin:auto;padding:8px'></div>
<script>
const bt=document.getElementById('bt');
function botao(){fetch('/botao').then(r=>r.json()).then(d=>{bt.textContent=(d.ativo?'ANDANDO':'PARADO')+' - ESPACO = botao do robo';bt.style.background=d.ativo?'#2a7':'#a33'})}
bt.onclick=botao;
const sv=document.getElementById('sv');
sv.onclick=()=>fetch('/salvar').then(r=>r.json()).then(d=>{sv.textContent=d.salvos.length?'SALVO: '+d.salvos.join(' '):'nada mudou'; setTimeout(()=>sv.textContent='SALVAR NO CODIGO',3000)});
document.addEventListener('keydown',e=>{if(e.code==='Space'&&!e.repeat){e.preventDefault();botao()}});
const SETAS={ArrowUp:'cima',ArrowDown:'baixo',ArrowLeft:'esq',ArrowRight:'dir'};
let seta=null,tSeta=null;
function mandaSeta(d){fetch('/setas?dir='+d).catch(()=>{})}
function solta(){if(seta){seta=null;clearInterval(tSeta);mandaSeta('solta')}}
document.addEventListener('keydown',e=>{const d=SETAS[e.code];if(!d)return;e.preventDefault();if(e.repeat||seta===d)return;
  clearInterval(tSeta);seta=d;mandaSeta(d);tSeta=setInterval(()=>mandaSeta(d),100)});
document.addEventListener('keyup',e=>{if(SETAS[e.code]===seta){e.preventDefault();solta()}});
window.addEventListener('blur',solta);
const GARRA={KeyS:'sobe',KeyD:'desce'};
let garra=null,tGarra=null;
function mandaGarra(d){fetch('/garra?dir='+d).catch(()=>{})}
function soltaGarra(){if(garra){garra=null;clearInterval(tGarra);mandaGarra('solta')}}
document.addEventListener('keydown',e=>{const d=GARRA[e.code];if(!d)return;e.preventDefault();if(e.repeat||garra===d)return;
  clearInterval(tGarra);garra=d;mandaGarra(d);tGarra=setInterval(()=>mandaGarra(d),100)});
document.addEventListener('keyup',e=>{if(GARRA[e.code]===garra){e.preventDefault();soltaGarra()}});
window.addEventListener('blur',soltaGarra);
fetch('/params').then(r=>r.json()).then(d=>{
  const c=document.getElementById('p');
  for(const [n,v] of Object.entries(d)){
    const row=document.createElement('div');
    row.style='display:flex;align-items:center;gap:8px;margin:4px 0';
    row.innerHTML=`<span style='width:110px'>${n}</span><input type=range min=${v.min} max=${v.max} step=${v.step} value=${v.val} style='flex:1'><b style='width:60px'>${v.val}</b>`;
    const i=row.querySelector('input'),b=row.querySelector('b');
    i.oninput=()=>{b.textContent=i.value;fetch('/set?nome='+n+'&valor='+i.value)};
    c.appendChild(row);
  }
});
</script></body></html>""".encode()


def _seg():
    import seguimento_de_linha
    return seguimento_de_linha


def _alvo(nome):
    if nome in NO_CONFIG:
        import config
        return config
    return _seg()


def _fmt(v):
    return str(int(v)) if float(v).is_integer() else f"{v:.4g}"


def _salvar_no_codigo():
    """grava os valores atuais no bloco de config; o valor antigo vai pro comentario"""
    base = os.path.dirname(os.path.abspath(__file__))
    salvos = []
    for nome in AJUSTES:
        alvo = _alvo(nome)
        arq = os.path.join(base, "config.py" if nome in NO_CONFIG else "seguimento_de_linha.py")
        novo = _fmt(getattr(alvo, nome))
        with open(arq, encoding="utf-8") as f:
            linhas = f.read().split("\n")
        pad = re.compile(r"^(" + nome + r"\s*=\s*)(-?[\d.]+)(\s*)(#?)(.*)$")
        for i, ln in enumerate(linhas):
            m = pad.match(ln)
            if not m:
                continue
            antigo = m.group(2)
            if float(antigo) == float(novo):
                break
            resto = m.group(5).strip()
            jatem = [x.strip() for x in resto.split(" - ")[0].split(",")]
            comentario = resto if antigo in jatem[:1] else antigo + (", " + resto if resto else "")
            linhas[i] = f"{m.group(1)}{novo}  #{comentario}"
            tmp = arq + ".tmp.py"
            with open(tmp, "w", encoding="utf-8") as f:
                f.write("\n".join(linhas))
            try:
                py_compile.compile(tmp, doraise=True)
                os.replace(tmp, arq)
                salvos.append(f"{nome}={novo}")
            except Exception:
                os.remove(tmp)
            break
    return salvos


def _aplicar(nome, valor):
    setattr(_alvo(nome), nome, valor)
    if nome == "VEL_MAX":
        import config
        config.VEL_MIN = -valor


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj):
        corpo = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(corpo)))
        self.end_headers()
        self.wfile.write(corpo)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/params":
            dados = {n: {"min": a, "max": b, "step": c, "val": round(float(getattr(_alvo(n), n)), 4)} for n, (a, b, c) in AJUSTES.items()}
            self._json(dados)
            return
        if url.path == "/botao":
            import config
            self._json({"ativo": bool(config.alternar_botao())})
            return
        if url.path == "/garra":
            import config
            d = parse_qs(url.query).get("dir", [""])[0]
            if config.robo_ativo.is_set():
                self._json({"ok": False, "motivo": "robo rodando"})
                return
            if d == "solta":
                _garra["dir"] = None
            elif d == "desce":
                _garra["dir"] = None
                _garra_pra_baixo()
            elif d == "sobe":
                _garra["dir"] = d
                _garra["t"] = time.time()
            self._json({"ok": True})
            return
        if url.path == "/setas":
            import config
            d = parse_qs(url.query).get("dir", [""])[0]
            if config.robo_ativo.is_set():
                self._json({"ok": False, "motivo": "robo rodando"})
                return
            if d == "solta":
                if _setas["dir"] is not None:
                    _setas["dir"] = None
                    config.parar()
            elif d in ("cima", "baixo", "esq", "dir"):
                if d != _setas["dir"]:
                    a, g = VEL_SETAS_ANDAR, VEL_SETAS_GIRO
                    config.mover(*{"cima": (a, a), "baixo": (-a, -a), "esq": (-g, g), "dir": (g, -g)}[d])
                    _setas["dir"] = d
                _setas["t"] = time.time()
            self._json({"ok": True})
            return
        if url.path == "/salvar":
            salvos = _salvar_no_codigo()
            print(f"[AJUSTE] salvo no codigo: {salvos or 'nada mudou'}")
            self._json({"salvos": salvos})
            return
        if url.path == "/set":
            q = parse_qs(url.query)
            try:
                nome = q["nome"][0]
                valor = float(q["valor"][0])
                a, b, _ = AJUSTES[nome]
                valor = min(max(valor, a), b)
                _aplicar(nome, valor)
                print(f"[AJUSTE] {nome} = {valor}")
                self._json({"ok": True})
            except Exception:
                self._json({"ok": False})
            return
        if url.path != "/video":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(_PAGINA)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=quadro")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        with _lock:
            _clientes[0] += 1
        try:
            ultimo = None
            while True:
                jpg = _ultimo_jpeg[0]
                if jpg is None or jpg is ultimo:
                    time.sleep(0.01)
                    continue
                ultimo = jpg
                self.wfile.write(b"--quadro\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
        except Exception:
            pass
        finally:
            with _lock:
                _clientes[0] -= 1


def _vigia_setas():
    """Freia se a seta parou de chegar (pagina fechou/perdeu foco) ou se o
    robo voltou a rodar pelo botao (ai o seguidor manda nos motores)."""
    import config
    while True:
        time.sleep(0.05)
        if _setas["dir"] is None:
            continue
        if config.robo_ativo.is_set():
            _setas["dir"] = None
        elif time.time() - _setas["t"] > 0.35:
            _setas["dir"] = None
            config.parar()


def _garra_pra_baixo():
    """Manda o servo pesado direto pro GARRA_ANG_BAIXO. Abaixo de 0 o
    tx_servo do lgpio nao aceita (minimo 500us), entao usa PWM de 50 Hz com o
    pulso calculado (mesma escala: 0 = 500us, 180 = 2500us)."""
    import config
    import lgpio
    pulso = 500 + (GARRA_ANG_BAIXO / 180.0) * 2000
    if GARRA_ANG_BAIXO >= 0:
        config.mover_servo_direto(config.PINO_PESADO, GARRA_ANG_BAIXO)
    else:
        lgpio.tx_pwm(config.h, config.PINO_PESADO, 50, pulso / 20000 * 100)
        if not hasattr(config.mover_servo, "_angulo_atual"):
            config.mover_servo._angulo_atual = {}
        config.mover_servo._angulo_atual[config.PINO_PESADO] = GARRA_ANG_BAIXO
    print(f"[GARRA] Desceu pra posicao mais baixa: {GARRA_ANG_BAIXO} graus (pulso {pulso:.0f}us)")


def _vigia_garra():
    """Mexe o servo pesado aos poucos enquanto S/D estiver apertado. Para se a
    tecla parou de chegar (aba perdeu foco) ou o robo voltou a rodar."""
    import config
    passo_t = 0.03
    while True:
        time.sleep(passo_t)
        d = _garra["dir"]
        if d is None:
            continue
        if config.robo_ativo.is_set() or time.time() - _garra["t"] > 0.35:
            _garra["dir"] = None
            continue
        atual = getattr(config.mover_servo, "_angulo_atual", {}).get(config.PINO_PESADO, config.SERVO_PESADO_ANGULO_PARADO)
        passo = VEL_GARRA_GRAUS_S * passo_t if d == "sobe" else -VEL_GARRA_DESCE_GRAUS_S * passo_t
        novo = min(max(atual + passo, GARRA_ANG_MIN), GARRA_ANG_MAX)
        if novo != atual:
            config.mover_servo_direto(config.PINO_PESADO, novo)


def iniciar():
    if _iniciado[0]:
        return
    _iniciado[0] = True
    try:
        srv = ThreadingHTTPServer(("0.0.0.0", PORTA), _Handler)
    except OSError as e:
        print(f"[VIDEO] porta {PORTA} indisponivel: {e}")
        return
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=_vigia_setas, daemon=True).start()
    threading.Thread(target=_vigia_garra, daemon=True).start()
    print(f"[VIDEO] imagem em http://localhost:{PORTA}")


def publicar(roi, mask, cx):
    """so gasta CPU se alguem estiver com a pagina aberta"""
    if _clientes[0] <= 0:
        return
    img = roi.copy()
    if mask is not None:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, (0, 0, 255), 1)
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)
    if cx is not None:
        cv2.line(img, (int(cx), 0), (int(cx), h), (0, 255, 0), 2)
        erro = (cx - w / 2) / (w / 2) * 100
        cv2.putText(img, f"cx {int(cx)}  erro {erro:+.0f}%", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    else:
        cv2.putText(img, "sem linha", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    if ESCALA != 1:
        img = cv2.resize(img, (w * ESCALA, h * ESCALA))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, QUALIDADE_JPEG])
    if ok:
        _ultimo_jpeg[0] = buf.tobytes()

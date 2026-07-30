"""
Driver da placa Yahboom de 4 canais (motor DC + encoder). O protocolo e
identico por TX/RX (UART de hardware) ou por USB - so muda a porta serial.

Tentamos primeiro os pinos TX/RX do header (GPIO14/15, /dev/serial0), mas
nos testes de bancada a placa nao respondeu nada por ali (0 bytes, mesmo
com fiacao cruzada e GND comum conferidos) - suspeita de selecao de modo
I2C/UART na placa ou pino errado no conector. Via USB funcionou de cara
($read_vol# respondeu "$Battery:8.5V#"), entao o PORTA_SERIAL abaixo esta
apontando pro USB por enquanto. Ver comentario ao lado da constante pra
trocar de volta pro TX/RX quando/se resolver a causa.

Protocolo (confirmado no repo oficial do fabricante -
github.com/hwthinker/Yahboom-4channel-motor-driver-encoder, pasta
"RPi5/USART" e README principal):
  - Serial 115200 8N1, sem paridade, sem controle de fluxo.
  - Comandos em texto ASCII, formato "$comando:valor#" (ou so "$comando#").
  - Comando de velocidade ($spd) roda em MALHA FECHADA de verdade: a placa
    usa o encoder internamente (PID configuravel) pra manter a velocidade
    pedida em mm/s - nao e so PWM aberto. Pra controle sem encoder (ou
    controle direto de duty cycle) usa $pwm em vez de $spd.
  - M1..M4 no chassi de referencia do FABRICANTE seriam M1=diant.esq,
    M2=tras.esq, M3=diant.dir, M4=tras.dir - mas a fiacao REAL desse robo
    e diferente (conferido fisicamente):
    M1 = roda dianteira DIREITA, M2 = roda traseira ESQUERDA,
    M3 = roda traseira DIREITA,  M4 = roda dianteira ESQUERDA.
    Ou seja: lado ESQUERDO = M2 + M4, lado DIREITO = M1 + M3.
    M1 nao tem encoder (sem telemetria), entao pro lado direito so da
    pra confirmar via M3.

Uso basico:
    with YahboomMotorDriver() as motor:
        motor.aplicar_parametros_motor()   # aplica os presets do MOTOR_TYPE
        motor.velocidade(200, 200, 200, 200)  # mm/s, malha fechada
        time.sleep(2)
        motor.parar()
"""
import re
import time
import serial

# ===================== CONFIG =====================
# Conexao via TX/RX do header (GPIO14/15, /dev/serial0) nao respondeu nos
# testes de bancada (0 bytes, mesmo com fiacao cruzada e GND comum -
# suspeita de selecao de modo I2C/UART na placa ou pino errado no conector).
# Usando USB por enquanto, que funciona (confirmado: $read_vol# respondeu).
#
# A placa usa um adaptador USB-serial CH340 GENERICO, sem numero de serie -
# o mesmo chip do adaptador do Arduino ja usado no robo (so o PID USB
# diferencia: placa Yahboom = 1a86:7522, Arduino = 1a86:7523). Por isso
# NAO da pra confiar em /dev/serial/by-id (pode ser ambiguo entre os dois)
# nem em /dev/ttyUSB0 fixo (indice pode trocar se desconectar/reconectar
# em ordem diferente) - uso o caminho por PORTA FISICA USB, que e estavel
# enquanto o cabo ficar no mesmo conector da Pi.
PORTA_SERIAL = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
# Pra voltar a tentar via TX/RX do header, troque para "/dev/serial0"
BAUDRATE = 115200
TIMEOUT_LEITURA = 0.05  # segundos, leitura nao-bloqueante

# Tipo do motor plugado na placa - ajuste conforme o motor real:
#   1 = 520 (imas permanentes, PH2.0 6pin ligado A->A)
#   2 = 520 ligado com fiacao invertida (A motor -> B placa), tratado como "310"
#   3 = TT com encoder (roda dentada de velocidade)
#   4 = TT sem encoder (so PWM, $spd nao funciona)
#   5 = 520 tipo L
MOTOR_TYPE = 3  # motor real do robo: TT DC 6V, reducao 1:90, encoder 12 PPR, roda 60mm

# Presets por tipo de motor (deadzone, relacao de reducao, linhas do
# encoder, diametro da roda em mm). Tipos 1/2/4/5 sao os padroes de fabrica
# (copiados do exemplo de referencia RPi5/USART/USART.py) - so servem se
# voce tiver esses motores. O tipo 3 foi MEDIDO no motor real do robo (TT
# DC 6V, reducao 1:90, encoder 12 PPR, roda 60mm) - a deadzone e a unica
# que ainda e o valor sugerido pelo fabricante pra TT, pode precisar
# ajustar na pratica (ela nao vem de especificacao, e medida por tentativa).
PRESETS_MOTOR = {
    1: {"mtype": 1, "deadzone": 1600, "mphase": 30, "mline": 11, "wdiameter": 67.00},
    2: {"mtype": 2, "deadzone": 1300, "mphase": 20, "mline": 13, "wdiameter": 48.00},
    3: {"mtype": 3, "deadzone": 1250, "mphase": 90, "mline": 12, "wdiameter": 60.00},
    4: {"mtype": 4, "deadzone": 1000, "mphase": 48, "mline": None, "wdiameter": None},
    5: {"mtype": 1, "deadzone": 1600, "mphase": 40, "mline": 11, "wdiameter": 67.00},
}

VELOCIDADE_MAX_MM_S = 1000  # limite documentado do $spd (-1000~1000)
PWM_MAX = 3600               # limite documentado do $pwm (-3600~3600)

# Correcao de sentido por canal. A fiacao de forca esta correta (vermelho
# positivo, preto negativo, confirmado fisicamente) - o que acontece e que
# o motor de um lado do chassi fica montado espelhado em relacao ao outro,
# entao o mesmo sinal eletrico gira em sentidos opostos. Em vez de mexer
# no fio, corrige aqui: -1 inverte o canal antes de mandar o comando.
# Medido em bancada (comando +200 -> velocidade real lida de volta):
#   M2, M3, M4: giraram negativo com comando positivo -> invertidos aqui
#   M1: sem encoder, entao nunca deu pra confirmar por telemetria - mas o
#   robo girou pra esquerda no proprio eixo com M1 sem correcao (frente-
#   esquerda indo pra tras enquanto o resto ia pra frente), entao invertido
#   aqui tambem por hipotese (mesmo padrao dos outros 3 canais). Se ainda
#   girar depois disso, o problema pode ser desbalanceio de velocidade
#   entre os lados, nao mais polaridade.
POLARIDADE_MOTOR = (-1, -1, -1, -1)  # (M1, M2, M3, M4)


class YahboomMotorDriver:
    """Driver simples pra placa Yahboom de 4 canais via UART TX/RX."""

    _RE_MENSAGEM = re.compile(r"\$([A-Za-z_]+):?([^#]*)#")

    def __init__(self, port=PORTA_SERIAL, baudrate=BAUDRATE, timeout=TIMEOUT_LEITURA):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout,
        )
        self._buffer = ""

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.parar()
        except Exception:
            pass
        self.close()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ===================== BAIXO NIVEL =====================
    def _enviar(self, comando):
        """Manda um comando bruto, ja no formato $...#"""
        try:
            self.ser.write(comando.encode())
            time.sleep(0.01)  # tempo pra placa processar antes do proximo comando
        except serial.SerialException as e:
            print(f"[YAHBOOM][ERRO] Falha ao enviar '{comando}': {e}")

    def processar_recebidos(self):
        """Le o que tiver no buffer serial (sem bloquear) e devolve uma lista
        de mensagens ja parseadas, tipo:
        [{"tipo": "MSPD", "valores": [12.0, 12.0, 0.0, -5.0]}, ...]
        Chame isso periodicamente (ex: a cada iteracao do seu loop de
        controle) pra nao deixar o buffer da serial acumular."""
        resultado = []
        try:
            if self.ser.in_waiting > 0:
                self._buffer += self.ser.read(self.ser.in_waiting).decode(errors="ignore")
        except serial.SerialException as e:
            print(f"[YAHBOOM][ERRO] Falha ao ler serial: {e}")
            return resultado

        for m in self._RE_MENSAGEM.finditer(self._buffer):
            tipo, valores_str = m.group(1), m.group(2)
            item = {"tipo": tipo, "bruto": valores_str}
            if valores_str:
                partes = valores_str.split(",")
                try:
                    item["valores"] = [float(p) if "." in p else int(p) for p in partes]
                except ValueError:
                    item["valores"] = partes
            resultado.append(item)

        # so mantem o que sobrou depois da ultima mensagem completa
        ultimo_fim = 0
        for m in self._RE_MENSAGEM.finditer(self._buffer):
            ultimo_fim = m.end()
        self._buffer = self._buffer[ultimo_fim:]

        return resultado

    # ===================== CONFIGURACAO (salva na flash da placa) =====================
    def configurar_tipo_motor(self, tipo):
        self._enviar(f"$mtype:{tipo}#")

    def configurar_zona_morta(self, valor):
        valor = max(0, min(3600, int(valor)))
        self._enviar(f"$deadzone:{valor}#")

    def configurar_linhas_encoder(self, linhas):
        self._enviar(f"$mline:{int(linhas)}#")

    def configurar_relacao_reducao(self, relacao):
        self._enviar(f"$mphase:{int(relacao)}#")

    def configurar_diametro_roda(self, mm):
        self._enviar(f"$wdiameter:{mm:.2f}#")

    def configurar_pid(self, p, i, d):
        """Cuidado: a placa reinicia e para os motores toda vez que o PID
        muda - normal segundo o fabricante. Na duvida, nao mexe (o default
        P:0.8 I:0.06 D:0.5 costuma servir)."""
        self._enviar(f"$MPID:{p:.2f},{i:.2f},{d:.2f}#")

    def resetar_flash(self):
        """Volta todos os parametros pro padrao de fabrica. A placa reinicia."""
        self._enviar("$flash_reset#")

    def aplicar_parametros_motor(self, tipo=None):
        """Aplica de uma vez os presets (deadzone/reducao/linhas/diametro)
        do MOTOR_TYPE configurado no topo do arquivo (ou do 'tipo' passado).
        So precisa rodar uma vez - a placa salva na flash."""
        tipo = tipo if tipo is not None else MOTOR_TYPE
        preset = PRESETS_MOTOR.get(tipo)
        if preset is None:
            print(f"[YAHBOOM][ERRO] MOTOR_TYPE {tipo} desconhecido, nao apliquei nada")
            return

        self.configurar_tipo_motor(preset["mtype"])
        time.sleep(0.1)
        if preset["mphase"] is not None:
            self.configurar_relacao_reducao(preset["mphase"])
            time.sleep(0.1)
        if preset["mline"] is not None:
            self.configurar_linhas_encoder(preset["mline"])
            time.sleep(0.1)
        if preset["wdiameter"] is not None:
            self.configurar_diametro_roda(preset["wdiameter"])
            time.sleep(0.1)
        self.configurar_zona_morta(preset["deadzone"])
        time.sleep(0.1)

    def configurar_telemetria(self, total=False, tempo_real=False, velocidade_mm_s=False):
        """Liga/desliga o que a placa manda de volta sozinha depois de cada
        $spd: total = encoder acumulado ($MAll), tempo_real = pulsos por
        10ms ($MTEP), velocidade_mm_s = velocidade atual em mm/s ($MSPD)."""
        self._enviar(f"$upload:{int(total)},{int(tempo_real)},{int(velocidade_mm_s)}#")

    # ===================== CONTROLE DE MOTOR =====================
    def velocidade(self, m1, m2, m3, m4):
        """Controle em MALHA FECHADA (mm/s) - a placa usa o encoder e o PID
        interno pra manter essa velocidade sozinha. So funciona com motor
        com encoder (MOTOR_TYPE 1, 2, 3 ou 5). Nao precisa reenviar toda
        hora: a placa mantem a velocidade ate chegar um novo comando.

        Valores aqui sao em termos de "robo andando pra frente" - a
        correcao de POLARIDADE_MOTOR (motor espelhado de um lado do
        chassi) e aplicada por dentro, o chamador nao precisa pensar nisso."""
        valores = (m1, m2, m3, m4)
        m1, m2, m3, m4 = (
            max(-VELOCIDADE_MAX_MM_S, min(VELOCIDADE_MAX_MM_S, int(v * p)))
            for v, p in zip(valores, POLARIDADE_MOTOR)
        )
        self._enviar(f"$spd:{m1},{m2},{m3},{m4}#")

    def pwm(self, m1, m2, m3, m4):
        """Controle direto de PWM (malha aberta, sem encoder). Use pra
        motor sem encoder (MOTOR_TYPE 4) ou pra testar o motor bruto.
        Mesma correcao de POLARIDADE_MOTOR aplicada por dentro."""
        valores = (m1, m2, m3, m4)
        m1, m2, m3, m4 = (
            max(-PWM_MAX, min(PWM_MAX, int(v * p)))
            for v, p in zip(valores, POLARIDADE_MOTOR)
        )
        self._enviar(f"$pwm:{m1},{m2},{m3},{m4}#")

    def parar(self):
        self.velocidade(0, 0, 0, 0)
        self.pwm(0, 0, 0, 0)

    # ===================== LEITURA =====================
    def ler_bateria(self, timeout=1.0):
        """Manda $read_vol# e espera a resposta $Battery:x.xxV#. Retorna o
        valor em volts (float) ou None se nao respondeu a tempo."""
        self._enviar("$read_vol#")
        limite = time.time() + timeout
        while time.time() < limite:
            for msg in self.processar_recebidos():
                if msg["tipo"] == "Battery":
                    valor = msg["bruto"].rstrip("Vv")
                    try:
                        return float(valor)
                    except ValueError:
                        return None
            time.sleep(0.02)
        return None


if __name__ == "__main__":
    # Teste de bancada: liga telemetria de velocidade, aplica os parametros
    # do motor configurado, sobe e desce a velocidade dos 4 motores em
    # rampa e imprime o que a placa manda de volta. Ctrl+C pra parar.
    print(f"Conectando na placa Yahboom em {PORTA_SERIAL} @ {BAUDRATE}...")
    with YahboomMotorDriver() as motor:
        print("Lendo bateria...")
        volts = motor.ler_bateria()
        print(f"Bateria: {volts}V" if volts is not None else "Sem resposta de bateria (confira a fiacao TX/RX)")

        print(f"Aplicando parametros do MOTOR_TYPE={MOTOR_TYPE}...")
        motor.aplicar_parametros_motor()
        motor.configurar_telemetria(velocidade_mm_s=True)

        print("Rampa de velocidade nos 4 motores (Ctrl+C pra parar)...")
        try:
            alvo = 0
            passo = 20
            while True:
                for msg in motor.processar_recebidos():
                    if msg["tipo"] == "MSPD":
                        print("Velocidade atual (mm/s):", msg["valores"])

                alvo += passo
                if alvo > 400 or alvo < -400:
                    passo = -passo
                motor.velocidade(alvo, alvo, alvo, alvo)
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\nParando...")

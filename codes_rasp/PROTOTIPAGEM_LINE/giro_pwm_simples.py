"""
Controle simples via PWM direto (malha aberta, sem encoder/PID) - gira o
robo no proprio eixo por 2s e freia com um pulso curto reverso no final
(em vez de so cortar e deixar a inercia rodar solto).

Mapeamento real desse robo (nao e o padrao de fabrica da Yahboom):
  M1 = dianteira direita (sem encoder, sem telemetria)
  M2 = traseira esquerda
  M3 = traseira direita
  M4 = dianteira esquerda
Todo canal usa sinal invertido nesse robo (motor montado espelhado no
chassi) - ja aplicado direto nos comandos abaixo.

Rode com: python3 giro_pwm_simples.py
Aperta Enter pra girar, Ctrl+C pra sair.
"""
import serial
import time

PORTA = "/dev/serial/by-path/platform-xhci-hcd.0-usb-0:1:1.0-port0"
PWM_GIRO = 900    # forca do giro (faixa valida: -3600 a 3600)
TEMPO_GIRO = 5.0   # segundos girando
PWM_FREIO = 700    # forca do pulso de freio (reverso, mais fraco que o giro)
TEMPO_FREIO = 0.9 # segundos do pulso de freio

ser = serial.Serial(PORTA, 115200, timeout=0.5)


def girar_no_eixo():
    # (M1, M2, M3, M4): esquerda (M2, M4) pra frente + direita (M3, M1)
    # pra tras = gira no proprio eixo.
    ser.write(f"$pwm:{PWM_GIRO},{PWM_GIRO},{PWM_GIRO},{PWM_GIRO}#".encode()) #direita
    time.sleep(TEMPO_GIRO)

    # freio: pulso curto no sentido contrario, reduz a inercia
    ser.write(f"$pwm:{PWM_FREIO},{-PWM_FREIO},{PWM_FREIO},{-PWM_FREIO}#".encode())# esquerda 
    time.sleep(TEMPO_FREIO)

    ser.write(b"$pwm:0,0,0,0#")



try:
    while True:
        input("Enter pra girar 2s (Ctrl+C pra sair)... ")
        girar_no_eixo()
except KeyboardInterrupt:
    ser.write(b"$pwm:0,0,0,0#")
    print("\nParado.")

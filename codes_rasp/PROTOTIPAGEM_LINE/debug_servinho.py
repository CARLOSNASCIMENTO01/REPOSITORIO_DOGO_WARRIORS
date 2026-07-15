"""
Teste isolado: sem câmera, sem YOLO, sem nada além de GPIO.
Roda motor e servo alternadamente, devagar, pra você observar
com a mão / olho em qual componente físico o movimento realmente
acontece a cada mensagem impressa.

Objetivo: confirmar se o cruzamento acontece MESMO sem carga de CPU
(YOLO+OpenCV). Se sim -> é fiação física. Se só cruza com o robô
inteiro rodando (YOLO ativo) -> é disputa de timing por CPU.
"""
import lgpio
import time

IN1, IN2, ENA = 17, 18, 12
IN3, IN4, ENB = 22, 23, 13
PINO_SG90 = 20
PINO_PESADO = 16

h = lgpio.gpiochip_open(0)

for p in [IN1, IN2, ENA, IN3, IN4, ENB, PINO_SG90, PINO_PESADO]:
    lgpio.gpio_claim_output(h, p)

try:
    while True:
        print("\n=== Só MOTOR ESQUERDO pra frente (IN1=17,IN2=18,ENA=12) ===")
        lgpio.gpio_write(h, IN1, 0)
        lgpio.gpio_write(h, IN2, 1)
        lgpio.tx_pwm(h, ENA, 50, 60)
        time.sleep(2)
        lgpio.tx_pwm(h, ENA, 50, 0)
        lgpio.gpio_write(h, IN2, 0)
        time.sleep(1)

        print("\n=== Só SERVO SG90 (GPIO20) indo pra 0 grau ===")
        lgpio.tx_servo(h, PINO_SG90, 500)
        time.sleep(1)
        lgpio.tx_servo(h, PINO_SG90, 0)
        time.sleep(1)

        print("\n=== Só SERVO SG90 (GPIO20) indo pra 180 graus ===")
        lgpio.tx_servo(h, PINO_SG90, 2500)
        time.sleep(1)
        lgpio.tx_servo(h, PINO_SG90, 0)
        time.sleep(1)

        print("\n=== Só SERVO PESADO (GPIO16) indo pra 0 grau ===")
        lgpio.tx_servo(h, PINO_PESADO, 500)
        time.sleep(1)
        lgpio.tx_servo(h, PINO_PESADO, 0)
        time.sleep(1)

        print("\n=== Só MOTOR DIREITO pra frente (IN3=22,IN4=23,ENB=13) ===")
        lgpio.gpio_write(h, IN3, 0)
        lgpio.gpio_write(h, IN4, 1)
        lgpio.tx_pwm(h, ENB, 50, 60)
        time.sleep(2)
        lgpio.tx_pwm(h, ENB, 50, 0)
        lgpio.gpio_write(h, IN4, 0)
        time.sleep(1)

except KeyboardInterrupt:
    pass
finally:
    lgpio.gpiochip_close(h)
    print("Finalizado.")
# ROBO_DOGO_2026

Robô da OBR (seguidor de linha + resgate de vítimas) rodando numa Raspberry Pi 5
com um Arduino Nano como coprocessador de sonares.

Toda a pasta está liberada pra edição — não existe área "read-only" aqui.

## Hardware

| Peça | Ligação |
|---|---|
| Ponte H (motor esq.) | `IN1=17, IN2=18, ENA=12` |
| Ponte H (motor dir.) | `IN3=22, IN4=23, ENB=13` |
| Botão start/stop | GPIO `24` (pull-up, toggle) |
| Sensor TCRT | GPIO `6` |
| Servo garra SG90 | GPIO `5` (`PINO_SG90`) |
| Servo pesado | GPIO `20` (`PINO_PESADO`) |
| MPU6050 | I2C via `smbus2` (thread própria, eixo Y = rampa/gangorra) |
| Arduino Nano | serial FTDI, `PORTA_FIXA` em `/dev/serial/by-id/usb-FTDI_...` |
| Câmera da linha | índice fallback `0` |
| Câmera da vítima | índice fallback `2` |

GPIO é sempre `lgpio` (não RPi.GPIO). PWM dos motores em 50 Hz via `GPIO.tx_pwm`.

## Como roda

`/etc/systemd/system/robo.service` → `env_dogo/bin/python3 codes_rasp/PROTOTIPAGEM_LINE/seguidor_debug2.py`
(`Restart=on-failure`, `WorkingDirectory` = `PROTOTIPAGEM_LINE/`).

O robô é uma **cadeia de scripts que se substituem via `os.execv`**, não um processo só:

```
seguidor_debug2.py  --(area(): achou área de resgate)-->  resgate.py --entregas=N
resgate.py          --(MAX_ENTREGAS atingido / botão)-->  seguidor_debug2.py --auto
```

- `--auto` → já começa com `robo_ativo.set()` (sem esperar o botão).
- `--entregas=N` → carrega quantas vítimas já foram entregues (`ENTREGAS_HERDADAS`).
- Antes de todo `execv`: `parar_motores()`, `cap.release()`, `gpiochip_close(h)`.
  Se esquecer disso, o próximo script não consegue abrir o GPIO/câmera.

## Estrutura

```
codigo_arduino/codigo_arduino.ino    Nano: 2 sonares (TRIG/ECHO 3/2 e 6/5),
                                     manda "F:<dist>,E:<dist>" a cada 10 ms, 115200

codes_rasp/
  upload.sh                          arduino-cli upload (caminho ainda aponta p/ ~/OPENCV_LINE)
  LINE_FOLLOWER/
    line_complet.py                  cópia do seguidor (é o arquivo com alterações
                                     não commitadas no git)
  PROTOTIPAGEM_LINE/                 pasta de trabalho — é daqui que o systemd roda
    seguidor_debug2.py               ** SEGUIDOR PRINCIPAL ** (o que o service executa)
    seguidor_debug.py                versão anterior, sem os servos
    resgate.py                       ** RESGATE PRINCIPAL ** (bola + triângulo + entrega)
    RESGATE_CARLOS.py                variante do resgate, com busca de saída embutida
    proucurar_saida.py               rotina isolada de achar a saída (conta pixel preto)
    inferencia.py                    protótipo enxuto: buscar/centralizar/aproximar saída (YOLO)
    line_overenginer_base.py         código de referência de terceiros, NÃO roda no robô
    contagem_pixel.py / tirar_fotos.py / debug_servo.py / debug_camera
                                     scripts de bancada, um propósito cada
    teste_moondream_webcam.py        teste de VLM (Moondream) — usa env_moondream/, não o env_dogo
    modelon.onnx                     modelo de vítima (VICTIM_MODEL_PATH)
    onnx_saida.onnx                  modelo de saída
    model.onnx                       modelo antigo

env_dogo/                            venv de produção (gitignored)
codes_rasp/PROTOTIPAGEM_LINE/env_moondream/   venv só do Moondream
```

## Padrões que se repetem nos scripts grandes

Os quatro arquivos grandes (`seguidor_debug2`, `line_complet`, `resgate`,
`RESGATE_CARLOS`) são cópias divergentes do mesmo esqueleto — não existe módulo
comum. **`seguidor_debug2.py` é o único seguidor de linha em uso de fato** (é o
que o `robo.service` executa); `line_complet.py` e `seguidor_debug.py` são
cópias antigas/paralelas, não mantidas em sincronia. **Não propague correções
pra elas automaticamente** — só edite `seguidor_debug2.py`, a menos que peçam
explicitamente pra replicar em outro arquivo.

O esqueleto:

1. **`RoboParado` + `checar_parada()`** — botão é toggle. Aperta: `robo_ativo.clear()`,
   e o próximo `checar_parada()` dentro de qualquer `while` levanta a exceção,
   abortando guinada/desvio/resgate na hora. Aperta de novo: retoma.
   Todo `while` que segura o robô mais de um instante **precisa** chamar `checar_parada()`.
2. **`serial_thread()`** — lê `F:,E:` do Nano continuamente; `distancia_frente()` /
   `distancia_esquerda()` só leem o último valor. Tem watchdog de reconexão
   (`WATCHDOG_RECONEXAO = 2.0 s`) e `aguardar_porta()` pra USB que ainda está enumerando.
3. **`_gyro_thread()` + `angulo_y_estavel()`** — MPU integrado em thread; `angulo_y()`
   é filtrado (alpha 0.97) e é o que detecta rampa/gangorra.
4. **`resetar_usb_camera()`** — `ioctl(USBDEVFS_RESET)` quando a câmera trava,
   pra não precisar tirar o cabo na mão. Dispara depois de
   `FALHAS_CAMERA_ANTES_DE_RESETAR = 15` frames perdidos seguidos.
5. **`iniciar_hardware()`** — MPU e YOLO carregam em threads paralelas enquanto a
   serial e a câmera abrem, pra encurtar o boot.
6. O `loop_principal()` nunca morre por exceção: erro é logado, motores param, tenta de novo.

## Pipeline do seguidor (`seguidor_debug2.py`)

`visao(frame)` → ROI inferior, máscara de preto (`black_max_top/bot`), maior contorno → `cx`
→ `controle(cx, ...)` = PID (`Kp=1.8`, `Kd=0.7`, `VEL_BASE=25`, `DEADZONE=5`)
com tratamento especial de gangorra por `angulo_y()`
→ `verde(roi)` decide curva 90°/180° pelos marcadores verdes (`check_green` / `check_black`
/ `determine_turn_direction`)
→ `desvio()` usa o sonar frontal (`<= 6 cm`) pra contornar obstáculo
→ `detectar_travamento()` compara frames (SSIM-like) pra sair de onde encravou
→ `gap_debug()` + sonar esquerdo trata falta de linha; se persistir, chama `area()` → vai pro `resgate.py`
→ `detectar_fita_vermelha()` = fim de percurso.

Guinadas: `guinada2()` (tempo fixo) e `guinada_pid()` (fecha em cima do giroscópio,
`ZONA_CHEGADA` graus de tolerância). `meiavolta()` para 180°.

## Pipeline do resgate (`resgate.py`)

`buscar_bola()` (gira em passos curtos) → `centralizar_bola()` (proporcional, tolerância
em px) → `aproximar_bola()` (para quando `bbox_area / frame_area` passa do limiar) →
garra (`mover_servo(PINO_PESADO, ...)`) → `CLASSE_COR_ENTREGA` mapeia a classe YOLO da
bola pra cor do triângulo → `buscar_triangulo()` / `centralizar_triangulo()` /
`aproximar_triangulo()` (detecção por HSV + `_eh_triangulo()` no contorno) →
`entregar_bola()`. Ao bater `MAX_ENTREGAS`, roda `acionar_saida()` e volta pro seguidor.

## Convenções

- Código e comentários em **português**, sem acento nos comentários novos
  (vários arquivos já têm mojibake de encoding — não piore).
- Constantes de tuning ficam num bloco `# CONFIG` no topo, em MAIÚSCULA, com o valor
  antigo em comentário na mesma linha (`VEL_BASE = 25 #20, 25`). Mantenha esse hábito.
- Índices/paths de câmera e da serial são fixos por `by-id` sempre que possível.
- `.gitignore` já corta `env_dogo/`, `*.jpg`, `*.png`, `*.mp4`, `__pycache__`.
  Os `.onnx` (12 MB cada) **estão** versionados.

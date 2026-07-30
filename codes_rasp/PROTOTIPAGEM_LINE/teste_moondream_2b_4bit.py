"""
Teste do Moondream 2B 4-bit (moondream/moondream-2b-2025-04-14-4bit) via webcam.

Mesma ideia do teste_moondream_webcam.py (que usa o vikhyatk/moondream2
padrao), so que com a versao 4-bit (quantization-aware training, ~2GB de
peso em vez de ~3.7GB). O repo original recomenda device_map cuda - aqui
forcamos cpu porque a Raspberry Pi nao tem GPU. Sem GPU, e normal cada
pergunta demorar varios segundos.

Rode com: env_moondream/bin/python3 teste_moondream_2b_4bit.py
"""
import cv2
import time
from PIL import Image
from transformers import AutoModelForCausalLM

MODEL_ID = "moondream/moondream-2b-2025-04-14-4bit"
# Indice numerico puro (2) e fragil (pode abrir a camera errada ou falhar
# se o barramento USB estiver instavel) - mesmo caminho fixo usado no
# resto do projeto pra essa camera (frontal, Generic_USB_Camera).
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"
INTERVALO_PERGUNTA = 3.0  # segundos entre perguntas (modelo e mais pesado, CPU e lenta)

PERGUNTA = """
Existe uma abertura na parede?

Responda somente:

SIM

ou

NAO
"""

print(f"Carregando {MODEL_ID} (cpu)...")
t0 = time.time()
try:
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_ID,
        trust_remote_code=True,
        device_map={"": "cpu"},
    )
except Exception as e:
    print("[ERRO] Falha ao carregar o modelo:", repr(e))
    print("Se o erro for sobre kernel/dtype do torchao, o int4 desse "
          "checkpoint pode nao ter implementacao pra CPU nessa versao - "
          "nesse caso a alternativa e usar os arquivos .gguf do mesmo repo "
          "via llama.cpp em vez do transformers.")
    raise
print(f"Modelo carregado em {time.time() - t0:.1f}s")

print("Abrindo camera...")
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Erro ao abrir a camera.")
    exit()
print("Camera aberta")

resposta = "..."
ultima_pergunta = 0

print("Entrando no loop...")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    agora = time.time()

    if agora - ultima_pergunta >= INTERVALO_PERGUNTA:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imagem = Image.fromarray(rgb)

        try:
            print("Perguntando ao modelo...")
            t_pergunta = time.time()
            resposta = model.query(imagem, PERGUNTA)["answer"]
            print(f"Modelo respondeu em {time.time() - t_pergunta:.1f}s")
        except Exception as e:
            resposta = str(e)

        print("Resposta:", resposta)
        ultima_pergunta = agora

    cv2.putText(
        frame,
        resposta,
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
    )
    cv2.imshow("Moondream 2B 4-bit", frame)

    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

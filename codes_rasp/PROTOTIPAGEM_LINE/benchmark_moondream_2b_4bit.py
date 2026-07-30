"""
Benchmark rapido: carrega o moondream 2B 4-bit, tira UMA foto da camera
frontal e cronometra uma unica pergunta. Sem loop, sem janela - so pra
medir latencia real de inferencia na CPU da Pi antes de decidir se da
pra usar isso em tempo real no robo.
"""
import cv2
import time
from PIL import Image
from transformers import AutoModelForCausalLM

MODEL_ID = "moondream/moondream-2b-2025-04-14-4bit"
CAMERA_INDEX = "/dev/v4l/by-id/usb-Generic_USB_Camera_200901010001-video-index0"

print(f"Carregando {MODEL_ID} (cpu)...")
t0 = time.time()
model = AutoModelForCausalLM.from_pretrained(
    MODEL_ID,
    trust_remote_code=True,
    device_map={"": "cpu"},
)
print(f"Modelo carregado em {time.time() - t0:.1f}s")

print("Capturando frame...")
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
if not cap.isOpened():
    print("Erro ao abrir a camera.")
    exit(1)
for _ in range(5):
    cap.read()
ret, frame = cap.read()
cap.release()
if not ret:
    print("Erro ao capturar frame.")
    exit(1)

rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
imagem = Image.fromarray(rgb)

print("Perguntando ao modelo (cronometrando)...")
t0 = time.time()
resposta = model.query(imagem, "Existe uma abertura na parede? Responda SIM ou NAO.")["answer"]
dt = time.time() - t0

print(f"\nResposta: {resposta}")
print(f"Tempo da pergunta: {dt:.1f}s")

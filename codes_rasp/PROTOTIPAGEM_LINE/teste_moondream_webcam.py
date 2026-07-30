import cv2
from PIL import Image
from transformers import AutoModelForCausalLM
import time

print("Carregando Moondream...")

model = AutoModelForCausalLM.from_pretrained(
    "vikhyatk/moondream2",
    revision="2025-03-27",
    trust_remote_code=True,
)

print("Modelo carregado!")

print("Abrindo c�mera...")
cap = cv2.VideoCapture(2)
print("C�mera aberta")

print("Entrando no loop...")

if not cap.isOpened():
    print("Erro ao abrir a c�mera.")
    exit()

ultima_pergunta = 0
intervalo = 1.0  # pergunta a cada 1 segundo

while True:

    ret, frame = cap.read()

    if not ret:
        break

    agora = time.time()

    if agora - ultima_pergunta >= intervalo:

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        imagem = Image.fromarray(rgb)

        try:
            print("Perguntando ao modelo...")
            resposta = model.query(
                imagem,
                """
Existe uma abertura na parede?

Responda somente:

SIM

ou

NAO
"""
            )["answer"]
            print("Modelo respondeu!")

        except Exception as e:
            resposta = str(e)

        print("Resposta:", resposta)

        ultima_pergunta = agora

    cv2.putText(
        frame,
        resposta if 'resposta' in locals() else "...",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2
    )

    cv2.imshow("Moondream", frame)

    tecla = cv2.waitKey(1)

    if tecla == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
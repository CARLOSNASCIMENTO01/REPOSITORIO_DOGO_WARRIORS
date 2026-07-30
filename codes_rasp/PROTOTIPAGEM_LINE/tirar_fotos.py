import cv2
import os

# Cria a pasta caso não exista
os.makedirs("fotos_saida", exist_ok=True)

cap = cv2.VideoCapture(2)  # Troque para 1 se sua câmera estiver no índice 1

contador = 0

print("Pressione 'S' para salvar uma foto.")
print("Pressione 'Q' para sair.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erro ao ler a câmera.")
        break

    cv2.imshow("Camera", frame)

    tecla = cv2.waitKey(1) & 0xFF

    if tecla == ord('s'):
        nome = f"fotos_bolas/foto_{contador:04d}.jpg"
        cv2.imwrite(nome, frame)
        print(f"Foto salva: {nome}")
        contador += 1

    elif tecla == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
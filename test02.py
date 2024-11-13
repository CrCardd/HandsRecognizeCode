import cv2
import mediapipe as mp
import math

MAX_DIST_Z = 0.05
MIN_DIST_Z = 0.03

# Inicializando o módulo de mãos do MediaPipe
hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=2)
mp_drawing = mp.solutions.drawing_utils

# Abrindo a captura de vídeo
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        continue

    # Convertendo a imagem para RGB (requisito do MediaPipe)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Processando a imagem para detectar as mãos
    results = Hands.process(frame_rgb)
    handPoints = results.multi_hand_landmarks
    h, w, _ = frame.shape

    if handPoints:
        for points in handPoints:
            mp_drawing.draw_landmarks(frame, points, hands.HAND_CONNECTIONS)

            # Definindo os pontos de referência (pontos 1 e 17)
            x1, y1, z1 = points.landmark[1].x, points.landmark[1].y, points.landmark[1].z
            x17, y17, z17 = points.landmark[17].x, points.landmark[17].y, points.landmark[17].z

            # Convertendo as coordenadas normalizadas (0 a 1) para coordenadas de pixel
            cx1, cy1 = int(x1 * w), int(y1 * h)
            cx17, cy17 = int(x17 * w), int(y17 * h)

            # Calculando a distância Euclidiana 3D
            dist = math.sqrt((x17 - x1) ** 2 + (y17 - y1) ** 2 + (z17 - z1) ** 2)
            
            # Exibindo a distância calculada
            print(f"Distância 3D (tamanho da mão): {dist:.4f}")

            # Desenhando uma linha entre os pontos 1 e 17
            cv2.line(frame, (cx1, cy1), (cx17, cy17), (0, 255, 255), 2)
            cv2.putText(frame, f"{dist:.4f}", (cx1, cy1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Mostrando o vídeo com as mãos detectadas
    cv2.imshow('Imagem', cv2.flip(frame, 1))

    # Esperando uma tecla pressionada para continuar
    cv2.waitKey(1)

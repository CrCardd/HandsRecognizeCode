import cv2
import mediapipe as mp

# Initialize MediaPipe Hands module
hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=2)
mpDwaw = mp.solutions.drawing_utils

# Initialize MediaPipe Drawing module for drawing landmarks
mp_drawing = mp.solutions.drawing_utils

# Open a video capture object (0 for the default camera)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        continue
    

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    results = Hands.process(frame_rgb)
    handPoints = results.multi_hand_landmarks
    h, w, _ = frame.shape
    pontos = []
    polegar_levantado = False
    indicador_levantado = False
    medio_abaixado = False
    anelar_abaixado = False
    minimo_levantado = False
    if handPoints:
        for points in handPoints:
            mpDwaw.draw_landmarks(frame, points,hands.HAND_CONNECTIONS)
            #podemos enumerar esses pontos da seguinte forma
            for id, cord in enumerate(points.landmark):
                cx, cy = int(cord.x * w), int(cord.y * h)
                cv2.putText(frame, str(id), (cx, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                pontos.append((cx,cy))
        print((handPoints[0]))

           
        center_x = int(w // 2)
        center_y = int(h // 2 * (1 + 0.85))
        a = "CENTRO DA MASSA FM É TUDO DE BOM"
        cv2.putText(frame, str(a), (center_x, center_y-10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 0), 1, cv2.LINE_AA)
        # cv2.line(frame, (center_x,center_y), ("FALTA")), (0, 255, 0), 2)

        print(f"Centro da tela: x={center_x}, y={center_y}")

            # dedos = [8,12,16,20]
            # contador = 0
            
            
            # print("\n\n\n\nAQUI IRMAO:")
            # print(pontos)
            # amostradinho = True

            # if pontos:
            #     # Polegar levantado
            #     polegar_levantado = pontos[4][0] > pontos[3][0]
            #     # Indicador levantado
            #     indicador_levantado = pontos[8][1] < pontos[6][1]
            #     # Médio abaixado
            #     medio_abaixado = pontos[12][1] > pontos[10][1]
            #     # Anelar abaixado
            #     anelar_abaixado = pontos[16][1] > pontos[14][1]
            #     # Mínimo levantado
            #     minimo_levantado = pontos[20][1] < pontos[18][1]

    # if polegar_levantado and indicador_levantado and medio_abaixado and anelar_abaixado and minimo_levantado:
    #     cv2.putText(frame, "EU TE AMO", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (155, 25, 255), 3)
    cv2.imshow('Imagem', cv2.flip(frame,1))
    cv2.waitKey(1)

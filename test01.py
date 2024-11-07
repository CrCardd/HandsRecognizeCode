# Fist install python's mediapipe library as  # pip3 install mediapipe

import cv2
import mediapipe as mp


mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


cap = cv2.VideoCapture(0)
with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.6, min_tracking_confidence=0.6) as hands:
  while cap.isOpened():
    success, image = cap.read()
    if not success:
      print("Ignoring empty camera frame.")
      # If loading a video, use 'break' instead of 'continue'.
      continue

    # To improve performance, optionally mark the image as not writeable to
    # pass by reference.
    image.flags.writeable = False
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    points = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    height, width, _ = image.shape
    if points.multi_hand_landmarks:
      for hand_id, hand_landmarks in enumerate(points.multi_hand_landmarks):
        mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
        # for idx, landmark in enumerate(hand_landmarks.landmark):
        #     # Converte as coordenadas normalizadas para coordenadas em pixels
        #     h, w, _ = image.shape
        #     x_pixel = int(landmark.x * w)
        #     y_pixel = int(landmark.y * h)

        #     # Desenha o índice acima de cada ponto
        #     cv2.putText(image, str(idx), (x_pixel, y_pixel - 10), cv2.FONT_HERSHEY_SIMPLEX, 
        #                 0.5, (255, 0, 0), 1, cv2.LINE_AA)

    center_x = int(width // 2)
    center_y = int(height // 2 * (1 + 0.85))
    a = "CENTRO DA MASSA FM É TUDO DE BOM"
    cv2.putText(image, str(a), (center_x, center_y-10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 0), 1, cv2.LINE_AA)
    cv2.line(image, (center_x,center_y), ("FALTA")), (0, 255, 0), 2)

    print(f"Centro da tela: x={center_x}, y={center_y}")

    # Flip the image horizontally for a selfie-view display.
    cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == 27:
      break
      
cap.release()
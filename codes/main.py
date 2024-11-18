import cv2
import mediapipe as mp

MAX_DIST_Z = 0.05
MIN_DIST_Z = 0.03
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
    polegar_levantado = False
    indicador_levantado = False
    medio_abaixado = False
    anelar_abaixado = False
    minimo_levantado = False
    if handPoints:
        center_z_hand = 0

        for points in handPoints:   
            mpDwaw.draw_landmarks(frame, points,hands.HAND_CONNECTIONS)
            # for id, cord in enumerate(points.landmark):
            #     cx, cy = int(cord.x * w), int(cord.y * h)
                # cv2.putText(frame, str(id), (cx, cy + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        center_x_hand = int((w * (handPoints[0].landmark[1].x + handPoints[0].landmark[17].x)) / 2)
        center_y_hand = int((h * (handPoints[0].landmark[1].y + handPoints[0].landmark[17].y)) / 2)


        hand_distance_x = abs(handPoints[0].landmark[1].x - handPoints[0].landmark[17].x)
        hand_distance_y = abs(handPoints[0].landmark[1].y - handPoints[0].landmark[17].y)
        
        hand_size = (hand_distance_x*hand_distance_x + hand_distance_y*hand_distance_y) ** 0.5
        print(hand_size)

        closed_hand = True
        for i in range(4,21,4):
            finger_distance_x = abs(center_x_hand - w * handPoints[0].landmark[i].x)
            finger_distance_y = abs(center_y_hand - h * handPoints[0].landmark[i].y)
            finger_distance = (finger_distance_x*finger_distance_x + finger_distance_y*finger_distance_y) ** 0.5
            finger_distance_color = (100,255,100)
            if not finger_distance <= hand_size / 1.3:
                closed_hand = False 
                finger_distance_color = (100,100,255)
            cv2.line(frame, (int(w * handPoints[0].landmark[i].x), int(h * handPoints[0].landmark[i].y)), (center_x_hand, center_y_hand), finger_distance_color, 1)
            



        if closed_hand:
            cv2.putText(frame, str("MAO NOT ABRIDA"), (20,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            


        cv2.line(frame, (int(handPoints[0].landmark[1].x  * w), int(handPoints[0].landmark[1].y * h)), (int(handPoints[0].landmark[17].x * w), int(handPoints[0].landmark[17].y  * h)), (0, 255, 255), 2)

        center_x = int(w // 2)
        center_y = int(h // 2)
        center = "O"
        cv2.line(frame, (center_x,center_y), (center_x_hand, center_y_hand), (0, 255, 0), 2)

    
    cv2.imshow('Imagem', cv2.flip(frame,1))
    cv2.waitKey(1)

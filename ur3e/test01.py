import cv2
import mediapipe as mp
import URBasic
import time
import math3d as m3d
import math
import rtde_control
import rtde_receive

# Constants for hand tracking
MAX_DIST_Z = 0.24
MIN_DIST_Z = 0.24

# Initialize MediaPipe Hands module
hands = mp.solutions.hands
Hands = hands.Hands(max_num_hands=2)
mpDraw = mp.solutions.drawing_utils

# Initialize robot settings
ROBOT_IP = '169.254.41.22'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

# Initial joint positions (in radians)
joint_position = [-1.7075, -1.4654, -1.5655, -0.1151, 1.5962, -0.0105]

video_resolution = (700, 400)
video_midpoint = (int(video_resolution[0] / 2), int(video_resolution[1] / 2))

vs = cv2.VideoCapture(0)  # OpenCV video capture
vs.set(cv2.CAP_PROP_FRAME_WIDTH, video_resolution[0])
vs.set(cv2.CAP_PROP_FRAME_HEIGHT, video_resolution[1])

# Initialize robot with URBasic
print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP, robotModel=robotModel)

robot.reset_error()
print("robot initialised")
time.sleep(1)

robot.movej(q=joint_position, a=ACCELERATION, v=VELOCITY)

robot_position = [0, 0, 0]
origin = None

robot.init_realtime_control()
time.sleep(1)

# RTDE Setup for monitoring
# rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
# rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

def find_hands(image):
    frame_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = Hands.process(frame_rgb)
    hand_points = results.multi_hand_landmarks
    if hand_points:
        for points in hand_points:
            mpDraw.draw_landmarks(image, points, hands.HAND_CONNECTIONS)
    return hand_points, image

def move_to_hand(hand_landmarks, robot_pos):
    if hand_landmarks:
        hand_center = (
            (hand_landmarks[1].x + hand_landmarks[17].x) / 2, 
            (hand_landmarks[1].y + hand_landmarks[17].y) / 2
        )
        hand_distance_x = abs(hand_landmarks[1].x - hand_landmarks[17].x)
        hand_distance_y = abs(hand_landmarks[1].y - hand_landmarks[17].y)
        hand_size = (hand_distance_x**2 + hand_distance_y**2) ** 0.5

        # Scaling factors
        scale_x = 2.5  # Adjust as necessary
        scale_y = 2.5  # Adjust as necessary
        scale_z = 1    # Use hand_size for z scaling

        # Calculate robot target positions in workspace
        x_pos = hand_center[0] * scale_x
        y_pos = hand_center[1] * scale_y
        z_pos = hand_size * scale_z  # Use hand_size for Z position

        # Update robot target position (not directly modifying joint positions yet)
        robot_target_position = [robot_pos[0] + x_pos, robot_pos[1] + y_pos, robot_pos[2] + z_pos]
        robot_target_position = check_max_xy(robot_target_position)

        # Move robot to the calculated position
        joint_position[0] += x_pos * 0.1
        joint_position[1] += y_pos * 0.1
        joint_position[2] += z_pos * 0.1  # Can be used if needed

        # Move robot joints to new position
        robot.movej(q=joint_position, a=ACCELERATION, v=VELOCITY)

def check_max_xy(robot_target_xy):
    max_dist = 3  # Increase maximum distance for X and Y
    robot_target_xy[0] = max(-max_dist, min(robot_target_xy[0], max_dist))
    robot_target_xy[1] = max(-max_dist, min(robot_target_xy[1], max_dist))
    return robot_target_xy

# Monitor Robot Status using RTDE
# def monitor_robot_status():
#     joint_positions = rtde_r.getActualQ()
#     robot_mode = rtde_r.getRobotMode()
#     safety_status = rtde_r.getSafetyStatus()
#     print(f"Joint Positions: {joint_positions}")
#     print(f"Robot Mode: {robot_mode}")
#     print(f"Safety Status: {safety_status}")

while True:
    ret, frame = vs.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    
    # Detect hand landmarks
    hand_points, frame = find_hands(frame)

    # Process hand landmarks if available
    if hand_points:
        hand_landmarks = hand_points[0].landmark
        move_to_hand(hand_landmarks, robot_position)

    # Display hand tracking on screen
    cv2.imshow("Hand Tracking", frame)

    # Monitor robot status
    # monitor_robot_status()

    # Exit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

vs.release()
cv2.destroyAllWindows()

# Disconnect from robot safely
# rtde_c.disconnect()
# rtde_r.disconnect()

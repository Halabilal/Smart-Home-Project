import os
import cv2
import mediapipe as mp
import math
import numpy as np
import serial
import time

# Suppress TensorFlow logs
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# Solution APIs
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# Webcam Setup
wCam, hCam = 640, 480
cam = cv2.VideoCapture(0)
cam.set(3, wCam)
cam.set(4, hCam)

# Set up the serial connection (adjust 'COM3' to your port)
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=.1)
time.sleep(2)  # Give some time for the serial connection to initialize

# Function to smooth values using a moving average
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

# Initialize lists for smoothing
distance_vals = []
angle_vals = []
window_size = 5

# Mediapipe Hand Landmark Model
with mp_hands.Hands(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5) as hands:

    while cam.isOpened():
        success, image = cam.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Multi-hand landmarks method for finding the position of hand landmarks
        lmList = []
        if results.multi_hand_landmarks:
            myHand = results.multi_hand_landmarks[0]
            for id, lm in enumerate(myHand.landmark):
                h, w, c = image.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmList.append([id, cx, cy])

        # Assigning variables for thumb and index finger position
        if len(lmList) != 0:
            x1, y1 = lmList[4][1], lmList[4][2]
            x2, y2 = lmList[8][1], lmList[8][2]

            # Marking thumb and index finger
            cv2.circle(image, (x1, y1), 15, (255, 255, 255), -1)
            cv2.circle(image, (x2, y2), 15, (255, 255, 255), -1)
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 3)

            # Calculate distance and angle
            length = math.hypot(x2 - x1, y2 - y1)
            angle = math.degrees(math.atan2(y2 - y1, x2 - x1))

            # Smooth the values
            distance_vals.append(length)
            angle_vals.append(angle)

            if len(distance_vals) > window_size:
                distance_vals.pop(0)
            if len(angle_vals) > window_size:
                angle_vals.pop(0)

            smooth_distance = moving_average(distance_vals, window_size)[-1]
            smooth_angle = moving_average(angle_vals, window_size)[-1]

            # Ensure the angle is within 0 to -180 degrees
            if smooth_angle < -180:
                smooth_angle = -180
            elif smooth_angle > 0:
                smooth_angle = 0

            # Print smoothed distance and angle
            print(f'Smooth Distance: {smooth_distance}')
            print(f'Smooth Angle: {smooth_angle} degrees')

            # Send smoothed distance and angle to Arduino
            arduino.write(f"{int(smooth_distance)},{int(smooth_angle)}\n".encode())

            # Display smoothed distance and angle on the webcam feed
            cv2.putText(image, f'Distance: {int(smooth_distance)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(image, f'Angle: {int(smooth_angle)} deg', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        cv2.imshow('handDetector', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release resources
cam.release()
cv2.destroyAllWindows()

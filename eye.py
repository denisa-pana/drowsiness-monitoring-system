import cv2
import dlib
import numpy as np
import requests
from io import BytesIO
import os
import time
import serial 


def shape_to_np(shape, dtype="int"):
    coords = np.zeros((68, 2), dtype=dtype)
    for i in range(0, 68):
        coords[i] = (shape.part(i).x, shape.part(i).y)
    return coords

def eye_on_mask(mask, side):
    points = [shape[i] for i in side]
    points = np.array(points, dtype=np.int32)
    mask = cv2.fillConvexPoly(mask, points, 255)
    return mask

def contouring(thresh, mid, img, right=False):
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    try:
        cnt = max(cnts, key=cv2.contourArea)
        M = cv2.moments(cnt)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        if right:
            cx += mid
        cv2.circle(img, (cx, cy), 4, (0, 0, 255), 2)
    except:
        pass


def calculate_eye_movement(left_eye, right_eye):
    # Calculate the horizontal and vertical distances between left and right pupils
    horizontal_distance = right_eye[0] - left_eye[0]
    vertical_distance = (left_eye[1] + right_eye[1]) // 2  # Average vertical position
    
    return horizontal_distance, vertical_distance
    
detector = dlib.get_frontal_face_detector()
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "shape_predictor_68_face_landmarks.dat")
predictor = dlib.shape_predictor(model_path)

previous_vertical_pos = 0
previous_horizontal_pos = 0

# Initialize variables for tracking eye movement and timing
last_eye_movement_time = time.time()
no_movement_threshold = 5 

left = [36, 37, 38, 39, 40, 41]
right = [42, 43, 44, 45, 46, 47]

# Replace this URL with the URL of your ESP32-CAM's MJPEG stream
stream_url = 'http://192.168.0.101:81/stream'

stream = requests.get(stream_url, stream=True)
bytes_stream = bytes()

cv2.namedWindow('image')
kernel = np.ones((9, 9), np.uint8)

def nothing(x):
    pass

cv2.createTrackbar('threshold', 'image', 0, 255, nothing)

thresh = None  # Initialize thresh variable

arduino_port = 'COM5'
baud_rate = 9600
# Establish serial connection with Arduino Uno
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)

while True:
    bytes_stream += stream.raw.read(1024)
    a = bytes_stream.find(b'\xff\xd8')
    b = bytes_stream.find(b'\xff\xd9')
    if a != -1 and b != -1:
        jpg = bytes_stream[a:b + 2]
        bytes_stream = bytes_stream[b + 2:]
        
        # Check if jpg is not empty before decoding
        if len(jpg) > 0:
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            rects = detector(gray, 1)
            for rect in rects:
                shape = predictor(gray, rect)
                shape = shape_to_np(shape)
                mask = np.zeros(img.shape[:2], dtype=np.uint8)
                mask = eye_on_mask(mask, left)
                mask = eye_on_mask(mask, right)
                mask = cv2.dilate(mask, kernel, 5)
                eyes = cv2.bitwise_and(img, img, mask=mask)
                mask = (eyes == [0, 0, 0]).all(axis=2)
                eyes[mask] = [255, 255, 255]
                mid = (shape[42][0] + shape[39][0]) // 2
                eyes_gray = cv2.cvtColor(eyes, cv2.COLOR_BGR2GRAY)
                threshold = cv2.getTrackbarPos('threshold', 'image')
                _, thresh = cv2.threshold(eyes_gray, threshold, 255, cv2.THRESH_BINARY)
                thresh = cv2.erode(thresh, None, iterations=2)
                thresh = cv2.dilate(thresh, None, iterations=4)
                thresh = cv2.medianBlur(thresh, 3)
                thresh = cv2.bitwise_not(thresh)
                contouring(thresh[:, 0:mid], mid, img)
                contouring(thresh[:, mid:], mid, img, True)
                
                # Get the positions of left and right pupils
                left_pupil = shape[36:42].mean(axis=0).astype(int)
                right_pupil = shape[42:48].mean(axis=0).astype(int)
                
                # Calculate eye movement
                horizontal_distance, vertical_distance = calculate_eye_movement(left_pupil, right_pupil)
                
                if previous_vertical_pos - vertical_distance < -3 or previous_vertical_pos - vertical_distance > 3 or previous_horizontal_pos - horizontal_distance < -3 or previous_horizontal_pos - horizontal_distance > 3:
                    last_eye_movement_time = time.time()
                
                previous_vertical_pos = vertical_distance
                previous_horizontal_pos = horizontal_distance
                
                # Calculate the elapsed time without eye movement
                elapsed_time_without_movement = time.time() - last_eye_movement_time
                cv2.putText(img, f'No_movement: {elapsed_time_without_movement}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                command = f"{1}\n"
                if elapsed_time_without_movement >= no_movement_threshold:
                    command = f"{2}\n"
                    print("No eye movement for 5 seconds - Send Signal!")
                arduino.write(command.encode('utf-8'))
                
        # Check if img is not empty before displaying it
        if img is not None and img.shape[0] > 0 and img.shape[1] > 0:
            cv2.imshow('eyes', img)
        
        # Check if thresh is not empty before displaying it
        if thresh is not None and thresh.shape[0] > 0 and thresh.shape[1] > 0:
            cv2.imshow("image", thresh)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()

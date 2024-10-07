import cv2
import numpy as np
import os
import time

# Checkerboard dimensions
CHECKERBOARD = (6, 8)  # 7x9 board has 6x8 inner corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Set up webcam
cap = cv2.VideoCapture(0)  # Use 0 for default webcam, adjust if necessary

# Set resolution to 640x480
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

current_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
current_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"Current frame size: {current_width}x{current_height}")

# Create directory for saving images
if not os.path.exists('calibration_images'):
    os.makedirs('calibration_images')

image_count = 0
max_images = 15  # Number of images to capture for calibration

print("Press 'c' to capture an image, 'q' to quit and start calibration.")

while image_count < max_images:
    ret, frame = cap.read()
    
    # Ensure the frame is 640x360
    if frame.shape[0] != 360 or frame.shape[1] != 640:
        frame = cv2.resize(frame, (640, 360))
    
    cv2.imshow('Webcam (640x480)', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        # Save the image
        img_name = f'calibration_images/image_{image_count}.jpg'
        cv2.imwrite(img_name, frame)
        print(f"Captured image {image_count + 1}/{max_images}")
        image_count += 1
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Process the captured images
for i in range(image_count):
    img = cv2.imread(f'calibration_images/image_{i}.jpg')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Get new camera matrix
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# Print the calibration results
print("\nCalibration Results:")
print(f"Camera.type: PinHole")
print(f"Camera.fx: {newcameramtx[0, 0]}")
print(f"Camera.fy: {newcameramtx[1, 1]}")
print(f"Camera.cx: {newcameramtx[0, 2]}")
print(f"Camera.cy: {newcameramtx[1, 2]}")
print(f"Camera.k1: {dist[0][0]}")
print(f"Camera.k2: {dist[0][1]}")
print(f"Camera.p1: {dist[0][2]}")
print(f"Camera.p2: {dist[0][3]}")
print(f"Camera.k3: {dist[0][4]}")
print(f"Camera.fps: 30.0")  # Assuming 30 fps, adjust if different
print(f"Camera.RGB: 1")
print(f"Camera.width: 640")
print(f"Camera.height: 360")

print("\nCalibration complete. Use these values to update your ORB-SLAM3 camera.yaml file.")

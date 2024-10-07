import cv2
import numpy as np
import os

# Checkerboard dimensions
CHECKERBOARD = (6, 8)  # 7x9 board has 6x8 inner corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Directory containing the calibration images
input_dir = 'calibration_images'

# Directory to save the annotated images
output_dir = 'annotated_calibration_images'

# Create output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Loop through all images in the input directory
for filename in os.listdir(input_dir):
    if filename.endswith(('.jpg', '.jpeg', '.png')):  # Add or remove file extensions as needed
        # Read the image
        img_path = os.path.join(input_dir, filename)
        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            # Refine the corners
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Draw the corners
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
            
            # Save the annotated image
            output_filename = f'annotated_{filename}'
            output_path = os.path.join(output_dir, output_filename)
            cv2.imwrite(output_path, img)
            
            print(f"Processed and saved: {output_filename}")
        else:
            print(f"Could not find chessboard corners in {filename}")

print("Processing complete. Check the 'annotated_calibration_images' folder for results.")
import cv2
import numpy as np
import glob
import os

# Define the path to the folder containing calibration images
image_folder_path = './calibration_images'
output_image_folder = './temp_images'

# Ensure temp_images folder exists to save extracted images
if not os.path.exists(output_image_folder):
    os.makedirs(output_image_folder)

# Debugging: Check if the image_folder_path exists
if not os.path.exists(image_folder_path):
    print(f"Image folder not found: {os.path.abspath(image_folder_path)}")
else:
    print(f"Image folder found: {os.path.abspath(image_folder_path)}")
    print("Images available:", os.listdir(image_folder_path))

# Calibration settings
checkerboard_size = (24, 32)  # Adjust to your checkerboard pattern size
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the checkerboard size
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Iterate over each image in the folder
for image_file in glob.glob(os.path.join(image_folder_path, '*.*')):
    print(f"Processing image: {image_file}")
    image = cv2.imread(image_file)
    if image is None:
        print(f"Failed to load image: {image_file}")
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
    
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(image, checkerboard_size, corners2, ret)
        cv2.imshow('Checkerboard Corners', image)
        cv2.waitKey(500)
    else:
        print(f"Checkerboard not found in {image_file}")

cv2.destroyAllWindows()

# Calibration step
if objpoints and imgpoints:
    ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    np.savetxt('camera_matrix.txt', camera_matrix, delimiter=',')
    np.savetxt('distortion_coeffs.txt', distortion_coeffs, delimiter=',')
    print("Calibration successful. Camera matrix and distortion coefficients saved.")
else:
    print("Calibration failed. No checkerboard patterns were found.")

# Cleanup temp images
for file in os.listdir(output_image_folder):
    os.remove(os.path.join(output_image_folder, file))

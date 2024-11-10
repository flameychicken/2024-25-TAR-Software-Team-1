import cv2
import numpy as np
import glob
import os
from pdf2image import convert_from_path

# Define the path to the folder containing PDF calibration images
pdf_folder_path = './calibration_images'
output_image_folder = './practice_mission/camera_calibration'

# Ensure temp_images folder exists to save extracted images
if not os.path.exists(output_image_folder):
    os.makedirs(output_image_folder)

# Calibration settings
checkerboard_size = (7, 6)  # Adjust to your checkerboard pattern size
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on the checkerboard size
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Iterate over each PDF in the folder
for pdf_file in glob.glob(os.path.join(pdf_folder_path, '*.pdf')):
    # Convert PDF to images (one per page)
    pages = convert_from_path(pdf_file, dpi=300)
    
    # Process each page image
    for i, page in enumerate(pages):
        # Save the image temporarily for OpenCV to load
        img_path = os.path.join(output_image_folder, f'temp_image_{i}.jpg')
        page.save(img_path, 'JPEG')
        
        # Load the image with OpenCV
        image = cv2.imread(img_path)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Find the checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
        
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # Draw and display the corners
            cv2.drawChessboardCorners(image, checkerboard_size, corners2, ret)
            cv2.imshow('Checkerboard Corners', image)
            cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibration
if objpoints and imgpoints:
    ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    # Save calibration data
    np.savetxt('camera_matrix.txt', camera_matrix, delimiter=',')
    np.savetxt('distortion_coeffs.txt', distortion_coeffs, delimiter=',')

    print("Calibration successful. Camera matrix and distortion coefficients saved.")
else:
    print("Calibration failed. No checkerboard patterns were found.")

# Clean up temporary images
for file in os.listdir(output_image_folder):
    os.remove(os.path.join(output_image_folder, file))

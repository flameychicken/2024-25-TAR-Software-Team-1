import cv2
import numpy as np
from pdf2image import convert_from_path

# Define the dimensions of the checkerboard (update as needed)
CHECKERBOARD = (6, 9)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points based on checkerboard pattern
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Convert PDF pages to images
pdf_path = 'path_to_your_checkerboard_pdf.pdf'
images = convert_from_path(pdf_path, dpi=300)  # Increase DPI for higher resolution images

for i, img in enumerate(images):
    # Convert PIL image to a format compatible with OpenCV
    img = np.array(img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find checkerboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        print(f"Checkerboard detected on page {i + 1}")

# Calibrate the camera and save the calibration parameters
if imgpoints:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    # Save the matrix and distortion coefficients
    np.savetxt('cameraMatrix_webcam.txt', mtx, delimiter=',')
    np.savetxt('cameraDistortion_webcam.txt', dist, delimiter=',')
    print("Calibration completed and saved as 'cameraMatrix_webcam.txt' and 'cameraDistortion_webcam.txt'")
else:
    print("No checkerboard patterns detected in PDF pages.")

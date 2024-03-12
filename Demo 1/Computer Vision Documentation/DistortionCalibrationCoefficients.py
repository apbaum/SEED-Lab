'''
Adam Nussbaum and Zoe Karnisky
EENG 350 - SEED Lab
Calculating the the camera calibration coefficients
'''

import math
from time import sleep
import numpy as np
import cv2


# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.


camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

i = 0

while i < 100: # Taking 100 images
    print("Image Found")
    # Get an image from the camera stream
    ret, image = camera.read()
    
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Display',gray_image)
    ret, corners = cv2.findChessboardCorners(gray_image, (9,6),None)
    
    # If found, add object points, image points (after refining them)
    if ret == True:
        print("Checker Found!")
        i += 1
        
        objpoints.append(objp)
   
        cv2.cornerSubPix(gray_image,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners)
    
        # Draw and display the corners
        cv2.drawChessboardCorners(image, (9,6), corners, ret)
        cv2.imshow('Checkerboard',image)
        

    k = cv2.waitKey(500) & 0xFF
    # Break the loop if 'q' is pressed
    if k == ord('q'):
        break
        
        


    

# Calculating the camera matrix and the camera distortion matrix
camera.release()
cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_image.shape[::-1], None, None)
print(mtx)
np.save('CameraMatrix.npy',mtx)
print(dist)
np.save('CameraDistortion.npy',dist)



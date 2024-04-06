# Demo 2 Computer Vision Code
# Zoe Karnisky and Adam Nussbaum
# SEED Lab Spring 2024
# April 5, 2024

# Import the necessary libraries
import cv2
from cv2 import aruco
import numpy as np

import time
from time import sleep
import board
from smbus2 import SMBus
import struct


# Initialise I2C bus
prevAngle = 0
x = 0
#distance = 0.0
angle = 90.0
#ang_dist = [angle, distance]

# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
offset = 0

# Function to determine the angle of the aruco marker from the camera
def angleBetween(markerCenter,frameWidth):
    HorFOV = 60.0
    relPos = (markerCenter - (frameWidth/2)) / (frameWidth/2)

    angle = -1*relPos * (HorFOV / 2.0)
    #print(angle)
    round(angle,1)
    angle = angle * (10)

    return int(angle)

# Function to calculate the distance of the marker, which was not needed for demo 2, but we kept for use in the final project
def distanceCalc(corners,matrix,distortCoeff):
    distanceConstant = 1.1
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 10, matrix, distortCoeff)
    distance = np.linalg.norm(tvec) / distanceConstant
    distance = distance / 30.48
    distance = round(distance,1)
    return distance
    
# Camera Calibration, which was determined from a separate camera calibration code
matrix = np.array([[673.4502467, 0.000000000000000000e+00, 367.51306664],
[0.000000000000000000e+00, 666.2405825 , 221.70408371],
[0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
distortCoeff = np.array([[ 0.08976875, -0.53677562, -0.01045482,  0.01121468, 0.19515169]])

# Initialize Camera frame
camera = cv2.VideoCapture(0) 
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

# Load the ArUco dictionary
while True:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    ret,image = camera.read() # Takes an image and stores it in frame
    if not ret:
        break
    # sleep(.5) # wait for image to stabilize
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Undistorting the image
    height,width = image.shape[:2]
    camMatrixNew,roi = cv2.getOptimalNewCameraMatrix(matrix, distortCoeff, (width, height), 1, (width,height))
    undistImage = cv2.undistort(grey, matrix, distortCoeff, None, camMatrixNew)
    
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(image=undistImage,dictionary=aruco_dict)
    #print(ids)

    # If id has been read do the following code
    if ids is not None:
        for i in range(len(ids)):

            # Determines the location center of the marker
            centerMarker = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4 

            angle = angleBetween(centerMarker,width)
            #q.put(angle)

            # Checks that the angle is basically straight on using overflow
            if angle < (125) and angle > (-125): 
                # Send distance or run distance function
                #distance = distanceCalc(corners, matrix,distortCoeff)

                # Converts to 8-bit two's complement for negative angles
                if angle < 0:
                    angle = (1 << 8) + angle  # Convert to 8-bit two's complement
                    # Ensure that the value is within the 8-bit range
                    angle &= 0xFF
                print(angle)
                    
                try:
                    # Sends the angle to the arduino
                    i2c.write_byte_data(ARD_ADDR,offset,angle)
                    #i2c.write_byte_data(ARD_ADDR, offset, distance)
                    #data = bytearray(struct.pack("ff", angle, distance))  # '3B' indicates 3 unsigned bytes

                    # Send the data
                    #print(data)
                    #i2c.write_i2c_block_data(ARD_ADDR, offset, data)
                    #i2c.write_byte_data(ARD_ADDR, offset, ang_dist)
                except IOError:
                    print("Could not write data to the Arduino.")

            else:
                distance = 0   
    else:
        # Continue
        # No marker was detected
        print("No marker detected")
        
    # Shows camera image
    cv2.drawMarker(undistImage, (image.shape[1] // 2, image.shape[0] // 2), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
    cv2.imshow("Image",undistImage)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()

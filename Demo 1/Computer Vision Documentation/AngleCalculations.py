'''
Adam Nussbaum and Zoe Karnisky
EENG 350 - SEED Lab
Calculating the angle between the center of the screen and the Aruco Marker
'''

import cv2
from cv2 import aruco
import numpy as np

import time
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

# Initialise I2C bus.
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
prevAngle = 0

lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)
lcd.color = [100,0,100]
lcd.clear()

# Initialise the LCD class
q = queue.Queue()

# Calculating the angle between the marker and the center of the screen
def angleBetween(markerCenter,frameWidth):
    relPos = (markerCenter - (frameWidth/2)) / (frameWidth/2)
    angle = -1*relPos * (60 / 2.0) # Using 60 as that is the HFOV of the camera
    return angle

# Printing to the LCD Display
def LCDdisplay():
    while True:
        if not q.empty():
            angle = q.get()
            lcd.message = "Angle: \n"+ str(angle)
            
#Camera Calibration Coefficients
matrix = np.array([[673.4502467, 0.000000000000000000e+00, 367.51306664],
[0.000000000000000000e+00, 666.2405825 , 221.70408371],
[0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])
distortCoeff = np.array([[ 0.08976875, -0.53677562, -0.01045482,  0.01121468, 0.19515169]])
            
LCDthread = threading.Thread(target = LCDdisplay,args = ())
LCDthread.start()

camera = cv2.VideoCapture(0) 
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

# Load the ArUco dictionary
while True:
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    ret,image = camera.read() # Takes an image and stores it in frame
    if not ret:
        break
    #sleep(.5) # wait for image to stabilize
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #Undistorting the image
    height,width = image.shape[:2]
    camMatrixNew,roi = cv2.getOptimalNewCameraMatrix(matrix, distortCoeff, (width, height), 1, (width,height))
    undistImage = cv2.undistort(grey, matrix, distortCoeff, None, camMatrixNew)
    
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(image=undistImage,dictionary=aruco_dict)
    print(ids)

    if ids is not None:
        for i in range(len(ids)):

            # Finding the center then calculating the angle
            centerMarker = (corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]) / 4
            angle = angleBetween(centerMarker,width)
            print(angle)

            # Only updating angle when the angle changes
            if angle > (prevAngle + .02) or angle < (prevAngle - .02):
                q.put(angle)
            prevAngle = angle
            
            x = 0
            #sleep(1)
            
    else:
        lcd.clear()
        q.put("No marker")
        sleep(1)
        
    cv2.drawMarker(undistImage, (image.shape[1] // 2, image.shape[0] // 2), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
    cv2.imshow("Image",undistImage)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()

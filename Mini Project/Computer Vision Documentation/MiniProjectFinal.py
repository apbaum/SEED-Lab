# Using pictures to determine which quadrant marker is in
import cv2
from cv2 import aruco
import numpy as np
from time import sleep

from smbus2 import SMBus

import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

# Initialise I2C bus.
i2cLCD = board.I2C()  # uses board.SCL and board.SDA
# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)
offset = 0

q = queue.Queue()


def LCDdisplay():
    #Initializes LCD
    # Modify this if you have a different sized Character LCD
    lcd_columns = 16
    lcd_rows = 2
    lcd = character_lcd.Character_LCD_RGB_I2C(i2cLCD, lcd_columns, lcd_rows)
    lcd.clear()

    while True:
        if not q.empty():
            # Printing to the LCD from the queue
            location = q.get()
            lcd.clear()
            lcd.message = "Desired Location:\n"+str(location)
            
# Initializing the threading
LCDthread = threading.Thread(target = LCDdisplay,args = ())
LCDthread.start()



# Initializes the camera and sets the frame size
camera = cv2.VideoCapture(0) 
camera.set(cv2.CAP_PROP_FRAME_WIDTH,640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
quadrant = 0

while(True):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    ret,image = camera.read() # Takes an image and stores it in frame
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Converts image to grayscale for aruco detection
 
    
    # Breaks out of the loop if you press q key
    k = cv2.waitKey(1000) & 0xFF
    if k == ord("q"):
        break


    corners, ids, rejected = cv2.aruco.detectMarkers(image=grey,dictionary=aruco_dict)
    
    if ids is None:
        continue
    else:
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

        # Calculates the center location of the aruco marker
        cx = x_sum*.25 
        cy = y_sum*.25

        # Displays a center crosshair on the image and a center circle in the center of the aruco marker
        cv2.drawMarker(image, (image.shape[1] // 2, image.shape[0] // 2), (0, 255, 0), cv2.MARKER_CROSS, 10, 2)
        cv2.circle(image, (int(cx), int(cy)), 1, (0, 0, 255), 8)

        if cx <= 320:
            if cy <= 240:
                quadrant = 1 # Corresponds to Northwest quadrant
                q.put("[0,1]")
            else:
                quadrant = 2 # Corresponds to Southwest quadrant
                q.put("[1,1]")
        else:
            if cy <= 240:
                quadrant = 0 # Corresponds to Northeast quadrant
                q.put("[0,0]")
            else:
                quadrant = 3 # Corresponds to Southeast quadrant
                q.put("[1,0]")

    print("Quadrant:", quadrant)
    try:
        i2c.write_byte_data(ARD_ADDR,offset,quadrant) # Sends the quadrant location to the arduino
    except IOError:
        print("Could not write data to the Arduino.")
    sleep(.1)

    cv2.imshow("result", image) # Displays the image
    
camera.release()
cv2.destroyAllWindows()

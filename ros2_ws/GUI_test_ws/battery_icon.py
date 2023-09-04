import numpy as np
import cv2 as cv
import time

#Create a blank image
img = np.zeros((512, 512, 3), np.uint8)

#Resize the image to emulate the size of the controller display
img = cv.resize(img, (1024, 600))

#Draw rectangle backdrop
cv.rectangle(img, (850, 10), (950, 50), (0, 255, 0), -1)
#Draw rectangle for battery
cv.rectangle(img, (850, 10), (950, 50), (255, 255, 255), 3)
#Draw terminal nib
cv.rectangle(img, (950, 25), (960, 35), (255, 255, 255), -1)
cv.imshow("Window", img)

cv.waitKey(1)
while True:
    #Show the battery gradient
   # for i in range(99):
   #     cv.rectangle(img, ((852 + i), 11), ((852 + i), 49), (0, 255, 0), -1)
   #     cv.imshow("Window", img)       
   #     cv.waitKey(20)
      
    for i in range(99):
    #255/99 steps = 2.57 - the step to make the color gradient on each increment
        #cv.rectangle(img, (850, 10), (950, 50), (0, 255, 0), -1)
        cv.rectangle(img, ((950 - i), 11), ((950 - i), 49), (0, (255 - (i * 2.57)), 0 + (i * 2.57), -1))
        
        cv.putText(img, str(100 - i), (880, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
        cv.imshow("Window", img)
        cv.waitKey(20)

##Note, when used in the actual submarine, the image will be different each time so the percentages will not overlap each other like they do in this test. It isn't really worth the time to go back and try to make the numbers not overlap since this is just a proof of concept

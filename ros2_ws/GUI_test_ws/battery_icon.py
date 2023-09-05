import numpy as np
import cv2 as cv
import time


##DIMS - THESE CHANGE SO I DONT HAVE TO CONSTANTLY UPDATE OTHER NUMBERS
battery_bottom_x = 900
battery_bottom_y = 10
battery_top_x = 1000
battery_top_y = 50

compass_bottom_x = 300
compass_bottom_y = 10
compass_top_x = 700
compass_top_y = 30
#Create a blank image
img = np.zeros((512, 512, 3), np.uint8)

#Resize the image to emulate the size of the controller display
img = cv.resize(img, (1024, 600))

#Draw rectangle backdrop
cv.rectangle(img, (battery_bottom_x, battery_bottom_y), (battery_top_x, battery_top_y), (0, 255, 0), -1)
#Draw rectangle for battery
cv.rectangle(img, (battery_bottom_x, battery_bottom_y), (battery_top_x, battery_top_y), (255, 255, 255), 3)
#Draw terminal nib
cv.rectangle(img, (battery_top_x, 25), (battery_top_x + 10, 35), (255, 255, 255), -1)

#Draw message console
cv.rectangle(img, (0, 560), (900, 600), (255, 255, 255), -1)
#Draw uptime counter
cv.rectangle(img, (920, 560), (1024, 600), (255, 255, 255), -1)
#Label
cv.putText(img, "Console log", (400, 590), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv.LINE_AA)
cv.putText(img, "00h:00m:00s", (930, 590), cv.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 1, cv.LINE_AA)
cv.imshow("Window", img)

#Compass base
cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (150, 150, 150), -1)
cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
#Draw left box
cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_bottom_x + 50, compass_top_y), (255, 255, 255), 2)
cv.putText(img, "000", (compass_bottom_x + 10, compass_bottom_y + 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
#Draw right box
cv.rectangle(img, (compass_top_x - 50, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
cv.putText(img, "NW", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA) 

#Draw heading pointer
p1 = (500, 34)
p2 = (490, 45)
p3 = (510, 45)
cv.line(img, p1, p2, (255, 255, 255), 2)
cv.line(img, p2, p3, (255, 255, 255), 2)
cv.line(img, p1, p3, (255, 255, 255), 2)

cv.waitKey(1)
while True:
    #Show the battery gradient
   # for i in range(99):
   #     cv.rectangle(img, ((852 + i), 11), ((852 + i), 49), (0, 255, 0), -1)
   #     cv.imshow("Window", img)       
   #     cv.waitKey(20)
      
    for i in range(99):
        #255/99 steps = 2.57 - the step to make the color gradient on each increment
        #Draw red/green bar
        cv.rectangle(img, (battery_bottom_x, battery_bottom_y), (battery_top_x, battery_top_y),  (0, (255 - (i * 2.57)), 0 + (i * 2.57)), -1)
        
        cv.rectangle(img, ((battery_top_x - i), 11), (battery_top_x, 49), (150, 150, 150), -1)
        
        cv.putText(img, str(100 - i).zfill(2), (930, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
        cv.imshow("Window", img)
        cv.waitKey(40)

##Note, when used in the actual submarine, the image will be different each time so the percentages will not overlap each other like they do in this test. It isn't really worth the time to go back and try to make the numbers not overlap since this is just a proof of concept

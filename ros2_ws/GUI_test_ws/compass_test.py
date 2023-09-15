import numpy as np
import cv2 as cv


compass_bottom_x = 300
compass_bottom_y = 10
compass_top_x = 700
compass_top_y = 30
#The length of the compass box - 100 (the size of the two sidebars) is 300/90 (The desired amount of degrees shown)
#After a lot of subjective testing, the above does work, but it doesn't look particularly great. This number makes 
#The headings appear much more nicely than before
scale_deg = 2.15

directions = ["N", "NW", "W", "SW", "S", "SE", "E", "NE"]


#Create a blank image
img = np.zeros((512, 512, 3), np.uint8)

#Resize the image to emulate the size of the controller display
img = cv.resize(img, (1024, 600))

while True:
    #Want to display each heading in the box... Need to make it so that everything gets centered on the point that the arrow is on
        #That way when we're heading North, North is atop the needle... etc. That point is 500
        for deg in range(360):
            #Compass base
            cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (150, 150, 150), -1)
            cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
           
            #Draw left box
            cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_bottom_x + 50, compass_top_y), (255, 255, 255), 2)

            #Draw right box
            cv.rectangle(img, (compass_top_x - 50, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
            
            #Draw heading pointer
            p1 = (500, 34)
            p2 = (490, 45)
            p3 = (510, 45)
            cv.line(img, p1, p2, (255, 255, 255), 2)
            cv.line(img, p2, p3, (255, 255, 255), 2)
            cv.line(img, p1, p3, (255, 255, 255), 2)
                        
            #Make the compass
            for cardinal in range(8): #each direction label
                degree = deg + cardinal*45 #calculate actual degree in clockwise manner
                if degree > 180: #Cut 360 in half so we go from -180- -> 180 (put NW, W, and SW on the left)
                    degree -= 360
                if(degree >= -60 and degree <= 60):
                    cv.putText(img, str(directions[cardinal]), (int(degree*scale_deg + 500), 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            
            #Write the actual heading in the compass right box
            #Will clean this up so that it's a struct or something that I stringify instead. Maybe. Either way there are going to be a bunch of "if" statements checking for the heading
            if(0 < deg < 44) or (deg == 360):
                cv.putText(img, "N", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(45 < deg < 89):
                cv.putText(img, "NE", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(90 < deg < 134):
                cv.putText(img, "E", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(135 < deg < 179):
                cv.putText(img, "SE", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(180 < deg < 224):
                cv.putText(img, "S", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(225 < deg < 269):
                cv.putText(img, "SW", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(270 < deg < 314):
                cv.putText(img, "W", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            if(315 < deg < 359):
                cv.putText(img, "NW", (663, 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)

            #Draw right box
            cv.rectangle(img, (compass_top_x - 50, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)

            #Display degree heading
            cv.putText(img, str(deg), (compass_bottom_x + 10, compass_bottom_y + 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
            cv.rectangle(img, (compass_top_x - 50, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
            cv.imshow("Window", img)
            cv.waitKey(80)

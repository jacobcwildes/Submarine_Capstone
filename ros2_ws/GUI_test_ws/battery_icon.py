import numpy as np
import cv2 as cv
import time
from datetime import datetime


##DIMS - THESE CHANGE SO I DONT HAVE TO CONSTANTLY UPDATE OTHER NUMBERS
battery_bottom_x = 900
battery_bottom_y = 10
battery_top_x = 1000
battery_top_y = 50

compass_bottom_x = 300
compass_bottom_y = 10
compass_top_x = 700
compass_top_y = 30

ballast_cap_l = np.array([[10, 200], [15, 190], [20, 185], [25, 185], [30, 190],  [35, 200]], np.int32)
ballast_cap_l = ballast_cap_l.reshape((-1, 1, 2))
ballast_stern_l = np.array([[10, 300], [15, 310], [30, 310], [35, 300]], np.int32)
ballast_stern_l = ballast_stern_l.reshape((-1, 1, 2))
hull_cap = np.array([[55, 175], [60, 170], [65, 165], [70, 160], [75, 155], [110, 155], [115, 160], [120, 165], [125, 170], [130, 175]], np.int32)
hull_cap = hull_cap.reshape((-1, 1, 2))
hull_stern = np.array([[55, 330], [60, 340], [125, 340], [130, 330]], np.int32)
hull_stern = hull_stern.reshape((-1, 1, 2))
ballast_cap_r = np.array([[150, 200], [155, 190], [160, 185], [165, 185], [170, 190], [175, 200]], np.int32)
ballast_cap_r = ballast_cap_r.reshape((-1, 1, 2))
ballast_stern_r = np.array([[150, 300], [155, 310], [160, 310], [170, 310], [175, 300]], np.int32)
ballast_stern_r = ballast_stern_r.reshape((-1, 1, 2))

#Start the program running counter:
prog_start = datetime.now()

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
    
#Draw left ballast base
cv.fillPoly(img, [ballast_cap_l], (150, 150, 150))
cv.rectangle(img, (10, 200), (35, 300), (150, 150, 150), -1)
cv.fillPoly(img, [ballast_stern_l], (150, 150, 150))

#Draw the left ballast tank outline
cv.polylines(img, [ballast_cap_l], True, (255, 255, 255), 2, cv.LINE_AA)
cv.rectangle(img, (10, 200), (35, 300), (255, 255, 255), 2)
cv.polylines(img, [ballast_stern_l], True, (255, 255, 255), 2, cv.LINE_AA)

#Draw right ballast base
cv.fillPoly(img, [ballast_cap_r], (150, 150, 150))
cv.rectangle(img, (150, 200), (175, 300), (150, 150, 150), -1)
cv.fillPoly(img, [ballast_stern_r], (150, 150, 150))

#Draw the right ballast tank outline
cv.polylines(img, [ballast_cap_r], True, (255, 255, 255), 2, cv.LINE_AA)
cv.rectangle(img, (150, 200), (175, 300), (255, 255, 255), 2)
cv.polylines(img, [ballast_stern_r], True, (255, 255, 255), 2, cv.LINE_AA)

#Draw connector base
cv.rectangle(img, (35, 220), (55, 230), (150, 150, 150), -1)
cv.rectangle(img, (150, 220), (130, 230), (150, 150, 150), -1)
cv.rectangle(img, (35, 245), (55, 255), (150, 150, 150), -1)
cv.rectangle(img, (150, 245), (130, 255), (150, 150, 150), -1)
cv.rectangle(img, (35, 270), (55, 280), (150, 150, 150), -1)
cv.rectangle(img, (150, 270), (130, 280), (150, 150, 150), -1)
#Draw the connector outlines
cv.rectangle(img, (35, 220), (55, 230), (255, 255, 255), 2)
cv.rectangle(img, (150, 220), (130, 230), (255, 255, 255), 2)
cv.rectangle(img, (35, 245), (55, 255), (255, 255, 255), 2)
cv.rectangle(img, (150, 245), (130, 255), (255, 255, 255), 2)
cv.rectangle(img, (35, 270), (55, 280), (255, 255, 255), 2)
cv.rectangle(img, (150, 270), (130, 280), (255, 255, 255), 2)
 
#Draw the hull base
cv.fillPoly(img, [hull_cap], (150, 150, 150))
cv.rectangle(img, (55, 175), (130, 330), (150, 150, 150), -1)
cv.fillPoly(img, [hull_stern], (150, 150, 150))

#Draw the hull outline
cv.polylines(img, [hull_cap], True, (255, 255, 255), 2, cv.LINE_AA)
cv.rectangle(img, (55, 175), (130, 330), (255, 255, 255), 2)
cv.polylines(img, [hull_stern], True, (255, 255, 255), 2, cv.LINE_AA)

#Draw message console
cv.rectangle(img, (0, 560), (900, 600), (255, 255, 255), -1)
#Draw uptime counter
cv.rectangle(img, (920, 560), (1024, 600), (255, 255, 255), -1)
#Label
cv.putText(img, "Console log", (400, 590), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv.LINE_AA)
cv.imshow("Window", img)

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

#The length of the compass box - 100 (the size of the two sidebars) is 300/90 (The desired amount of degrees shown)
#After a lot of subjective testing, the above does work, but it doesn't look particularly great. This number makes 
#The headings appear much more nicely than before
scale_deg = 2.18

directions = ["N", "NW", "W", "SW", "S", "SE", "E", "NE"]

cv.waitKey(1)

#Actual FPS
FPS = 0
#Time
previous_time = 0
#print(previous_time)
while True:
    
    for i in range(101):
        #255/99 steps = 2.57 - the step to make the color gradient on each increment
        #Start tracking FPS
        #Using monotonic time because I don't really care about real timezones. The 
        #monotonic clock naively ticks up - perfect for what I want
        current_time = time.monotonic()
        #print(current_time)


        #test = current_time - previous_time
        #print(test)
        #test = 1/test
        #print(test)
        
        #Draw red/green bar
        cv.rectangle(img, (battery_bottom_x, battery_bottom_y), (battery_top_x, battery_top_y),  (0, (255 - (i * 2.57)), 0 + (i * 2.57)), -1)
        
        cv.rectangle(img, ((battery_top_x - i), 10), (battery_top_x, 50), (150, 150, 150), -1)
        
        cv.putText(img, str(100 - i).zfill(2), (930, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv.LINE_AA)
        
        difference = current_time - previous_time
        FPS = 1/difference
        previous_time = current_time
        
        #This rectangle will not be necessary - the image will always be different so the writing won't get ugly overtop of itself
        cv.rectangle(img, (0,0), (22, 22), (0, 0, 0), -1)
        cv.putText(img, str(int(FPS)), (2, 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
        #This is not a perfect indication of the FPS in this case, but it is the logic that needs to be used in the execution (i.e. it won't be in a bunch of for loops because there won't BE for loops)
        cv.imshow("Window", img)
        cv.waitKey(20)

##Note, when used in the actual submarine, the image will be different each time so the percentages will not overlap each other like they do in this test. It isn't really worth the time to go back and try to make the numbers not overlap since this is just a proof of concept
    
    for deg in range(360):
        current_time = time.monotonic()
        
        #Want to display each heading in the box... Need to make it so that everything gets centered on the point that the arrow is on
        #That way when we're heading North, North is atop the needle... etc. That point is 500
        #Compass base
        cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (150, 150, 150), -1)
        cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
       
        #Draw left box
        cv.rectangle(img, (compass_bottom_x, compass_bottom_y), (compass_bottom_x + 50, compass_top_y), (255, 255, 255), 2)

        #Draw right box
        cv.rectangle(img, (compass_top_x - 50, compass_bottom_y), (compass_top_x, compass_top_y), (255, 255, 255), 2)
        
        #Draw compass headings
        for cardinal in range(8): #each direction label
                degree = deg + cardinal*45 #calculate actual degree in clockwise manner
                if degree > 180: #Cut 360 in half so we go from -180- -> 180 (put NW, W, and SW on the left)
                    degree -= 360
                if(degree >= -60 and degree <= 60):
                    cv.putText(img, str(directions[cardinal]), (int(degree*scale_deg + 495), 25), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
           
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

        difference = current_time - previous_time
        FPS = 1/difference
        previous_time = current_time
        
        #This rectangle will not be necessary - the image will always be different so the writing won't get ugly overtop of itself
        cv.rectangle(img, (0,0), (22, 22), (0, 0, 0), -1)
        cv.putText(img, str(int(FPS)), (2, 15), cv.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv.LINE_AA)
        #This is not a perfect indication of the FPS in this case, but it is the logic that needs to be used in the execution (i.e. it won't be in a bunch of for loops because there won't BE for loops) Also, this will be directly impacted by how long the waitkey is. The waitkey in actual program will not wait at all
        
        prog_current = datetime.now()
        prog_time = prog_current - prog_start
        
        #Remove microseconds from time delta
        prog_time = str(prog_time).split(".")[0]
        #Draw uptime counter - won't be a thing in regular program, just needed for testbench
        cv.rectangle(img, (920, 560), (1024, 600), (255, 255, 255), -1)
        cv.putText(img, str(prog_time), (930, 585), cv.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 1, cv.LINE_AA)
        cv.imshow("Window", img)
        cv.waitKey(20)
    
    #Simulate the ballasts filling
    for i in range(98):
        #Since our ballasts fill from front to back, show fill from top->bottom
        
        #Left ballast
        cv.rectangle(img, (12, 201), (33, 201), (150, 150, 150), -1)
        cv.rectangle(img, (12, 201), (33, 201 + i), (255, 0, 0), -1)
        
        #Right ballast
        cv.rectangle(img, (152, 201), (173, 201), (150, 150, 150), -1)
        cv.rectangle(img, (152, 201), (173, 201 + i), (255, 0, 0), -1)
        
        cv.imshow("Window", img)
        cv.waitKey(20)

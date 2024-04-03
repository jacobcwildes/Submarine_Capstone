import cv2 as cv
import numpy as np

cap = cv.VideoCapture("/home/jacob/Downloads/2024-03-21-192206.mp4")

if not cap.isOpened():
    print("Unable to open video stream")
    
while True:
    
    ret, frame = cap.read()
    
    if ret == True:
        cv.imshow('Frame', frame)
        
        if cv.waitKey(25) & 0xFF == ord('q'):
            break
            
            
    else:
        break
        
cap.release()

cv.destroyAllWindows()

import datetime
from ultralytics import YOLO
import cv2 as cv

#Define some constants
CONFIDENCE_THRESHOLD = 0.4
GREEN = (0, 255, 0)

classNames = ["Fish", "Perch"]

#Load the YOLO model
model = YOLO("runs/segment/yolov8_fish7/weights/best.pt")

#Initiate the video capture object
#video_cap = cv.VideoCapture(0)
video_cap = cv.VideoCapture("/home/jacob/Downloads/2024-03-21-192206.mp4")

if not video_cap.isOpened():
    print("Cannot open camera!")
    exit()
    
while True:

    print("Looping")
    
    #Start time to compute FPS
    start = datetime.datetime.now()    
    
    #Capture each frame
    ret, frame = video_cap.read()
    
    #If the frame was read properly, ret is true
    if not ret:
        print("Frame not received. Did the video end?")
        break
    
    #Turn the image from CV BGR to standard RGB
    #frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
    detections = model(frame, stream=True, vid_stride=15, device="cpu")[0]
    
    #Loop over any detections
    for data in detections.boxes.data.tolist():
        #Extract the confidence associated with each detection
        confidence = data[4]
        
        #Filter out weak detections
        #if float(confidence) < CONFIDENCE_THRESHOLD:
        #    continue
            
        #If the confidence is greater than the minimum confidence, draw the bounding box
        xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
        cv.rectangle(frame, (xmin, ymin), (xmax, ymax), GREEN, 2)
        
    end = datetime.datetime.now()
     
    #Time taken to compute a frame
    total = (end - start).total_seconds()
     
    fps = f"FPS: {1 / total:.2f}"
    cv.putText(frame, fps, (50, 50), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 8)
    
    #Display the resultant frame
    cv.imshow('Video', frame)
    
    cv.waitKey(0)
    #Halt video stream
   # if cv.waitKey(1) & 0xFF == ord('q'):
   #     break
    
#Free up resources
video_cap.release()
cv.destroyAllWindows()

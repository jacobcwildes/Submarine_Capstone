##Pull my custom built dataset from Roboflow
from roboflow import Roboflow


##Comment this after the first run
#rf = Roboflow(api_key="zMt80qeo3jcFrWZ7aZC4")
#project = rf.workspace("recycling-9d51j").project("fish_species-3xdxg")
#dataset = project.version(1).download("yolov5")

##Start YOLO training
from ultralytics import YOLO

#Load a base model
model = YOLO('yolov8n-seg.pt')

#Train!
results = model.train(
    data='data.yaml',
    imgsz=640,
    epochs=25,
    batch=8,
    device=0,
    optimizer='Adam',
    verbose=True,
    cos_lr=True,
    dropout=0.25,
    plots=True,
    name='yolov8_fish'
)

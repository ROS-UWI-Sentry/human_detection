#!/bin/sh

export DISPLAY=:0
gst-launch-1.0 nvarguscamerasrc .........

cd
cd yolov5

ls -ltrh /dev/video* > streams.txt

python2 get_devices.py

python3 detect.py --source streams.txt --classes 0 --nosave --weights yolov5l6.pt --webcamDebounceFrames 5 


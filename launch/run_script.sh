#!/bin/sh

cd
cd yolov5

ls -ltrh /dev/video* > streams.txt

python3 get_devices.py

python3 detect.py --source streams.txt --classes 0 --nosave --weights yolov5l6.pt --webcamDebounceFrames 5


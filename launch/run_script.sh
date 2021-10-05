#!/bin/sh

cd
cd yolov5

ls -ltrh /dev/video* > streams.txt

python2 get_devices_ros.py

python3 detect.py --source streams.txt --classes 0 --nosave --weights yolov5l6.pt --webcamDebounceFrames 5


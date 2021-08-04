#!/bin/sh

cd
cd yolov5

python3 detect.py --source 0 --classes 0 --nosave --weights yolov5l6.pt


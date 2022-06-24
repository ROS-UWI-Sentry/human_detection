# human_detection


This package holds the launch files for running and ending the yolov5 pyhton script. It is needed so that the ROS finite state machine can programatically start and stop the execution of it. Since yolov5 is process heavy and can freeze up, the end script uses pkill to end it.

The launch file starts the run_script.sh bash script. In this script webcamDebounceFrames specifies the amount of frames needed to have a human present in it sequentially before the detector publishes that a human is present. This helps for removing fase positives.  

For running headlessly we had to edit the code, view_img = True:

```
    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = False #check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride)
```

to use multiple usb webcams, in the launch bash script we had to add a few lines before we tried to run yolov5:

```
export DISPLAY=:0
gst-launch-1.0 nvarguscamerasrc .........
```

# human_detection


This package holds the launch files for running and ending the yolov5 pyhton script. It is needed so that the ROS finite state machine can programatically start and stop the execution of it. Since yolov5 is process heavy and can freeze up, the end script uses pkill to end it.

The launch file starts the run_script.sh bash script. In this script webcamDebounceFrames specifies the amount of frames needed to have a human present in it sequentially before the detector publishes that a human is present. This helps for removing fase positives.  

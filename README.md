# human_detection


This package holds the launch files for running and ending the yolov5 pyhton script. It is needed so that the ROS finite state machine can programatically start and stop the execution of it. Since yolov5 is process heavy and can freeze up, the end scrip uses pkill to end it.

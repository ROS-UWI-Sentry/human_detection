#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Bool


# Pin Definitions
output_pin = 18  # BCM pin 18, BOARD pin 12

def callback(data):
    global curr_value

    rospy.loginfo(rospy.get_caller_id() + 'Node heard: %s', data.data)
    if data.data == True:
        curr_value= GPIO.HIGH
        GPIO.output(output_pin, curr_value)
    elif data.data == False:
        curr_value= GPIO.LOW
        GPIO.output(output_pin, curr_value)

def listener():
    global curr_value

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    # set pin as an output pin with optional initial state of HIGH
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    print("Starting demo now! Press CTRL+C to exit")
    curr_value = GPIO.HIGH





    rospy.init_node('lightControlListener', anonymous=True)

    rospy.Subscriber('lightControl', Bool, callback)




    # try:
    #      while not rospy.is_shutdown():
    #             time.sleep(1)
    #             # Toggle the output every second
    #             print("Outputting {} to pin {}".format(curr_value, output_pin))
    #             GPIO.output(output_pin, curr_value)
    #             curr_value ^= GPIO.HIGH
    # finally:
    #     GPIO.cleanup()
    #     print("Exiting")



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    try:
        listener()  
    except KeyboardInterrupt:
        GPIO.cleanup()

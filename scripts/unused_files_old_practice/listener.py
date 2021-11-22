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

## This node recieves and debounces the output of the human detector
#It should be settable.


#put a default value of the debouncing time


import rospy
from std_msgs.msg import Bool

debounceAmount = 5 #amount of frames to debounce for
count = 0
lastVal = False
lastValues = []


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + 'Human detected? %s', data.data)

    global debounceAmount, count, lastVal, lastValues

    rospy.loginfo(rospy.get_caller_id() + 'Human detected? %s', data.data)
       


    #if count < debounceAmount :
    #     lastValues.append(data.data)
    #     count=count+1
    # elif count == debounceAmount:
    #     rospy.loginfo(rospy.get_caller_id() + 'Human detected? %s', all(lastValues))
    #     pub.publish(all(lastValues))
    #     count=0
    #     lastValues.clear()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    

    rospy.Subscriber('chatter', Bool, callback)








    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('listener')

    #INTIALIZE lastVaues array pls for any amount of time

    #create a publisher object and defines which topic it subscribes to
    pub=rospy.Publisher('humanDebounced', Bool, queue_size=100)

    #start the subscribing and publishing process
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
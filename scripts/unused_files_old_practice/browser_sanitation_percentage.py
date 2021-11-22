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

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from multiprocessing import Process, Pipe
import thread

#This node is a combination of a subscriber and publisher

keepCounting = False
percent = 0


#this callback function gets called whenever a message is recieved
#it pushes the message to the terminal for us to confirm what happened
#and it checks the value of the message and performs actions based on it
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
    global keepCounting, percent 

    if (data.data=="Initialize Counter"):
        percent = 0
        keepCounting = False

    elif (data.data=="Turn Off Sentry"):
        keepCounting = False
        percent = 0

    elif (data.data=="Started Sanitizing the Room"):
        if (percent==100):
            percent=0
            keepCounting = True
        else:
            keepCounting = True
    
    elif (data.data=="Paused Sanitizing the Room"):
        keepCounting = False
       
    elif (data.data=="Stopped Sanitizing the Room"):
        keepCounting = False
        percent = 0
        pub.publish(percent)
    else:
        rospy.logwarn("Incrorrect data received.")
        
#this function is for subscribing to messages
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

    global keepCounting, percent

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    rospy.Subscriber('brwsrButtons', String, callback)

    #this value is a sleep value
    rate = rospy.Rate(10) #10Hz

    #while ROS is not shutdown via terminal etc, if the conditions are met, publish counting from 0 to 100
    while not rospy.is_shutdown():
        if (percent < 101 and keepCounting):
            rospy.loginfo("The percent is: " + str(percent))
            pub.publish(percent)
            percent +=1
        rate.sleep()
   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('percentageHandler', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    pub = rospy.Publisher('progress', Int32, queue_size=10)
    
    #start the subscribing and publishing process
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

    




# def callback(data, args):
#     child_conn=args[0]
#     parent_conn=args[1]
#     rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
     
#     if (data.data=="Started Sanitizing the Room"):
#   #      startSanitization()
#         parent_conn.send(0)
#         startCount = Process(target=startStopSanitization, args=(child_conn,))
#         startCount.start()
#         startCount.join()        
#     elif (data.data=="Stopped Sanitizing the Room"):
#         parent_conn.send(1)
#  #        stopSanitization()
        


# def startStopSanitization(child_conn):
#     percent = 0
   
#     cconn=child_conn.recv()
#     while percent < 101 and cconn==0 :
#         rospy.loginfo(percent)
#         pub.publish(percent)
#         percent = percent + 1
#         cconn=child_conn.recv()
#         rate.sleep()
         




# def stopSanitization():
#     percent = 0
#     rospy.loginfo(percent)
#     pub.publish(percent)
        

    

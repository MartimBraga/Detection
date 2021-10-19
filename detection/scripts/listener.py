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
from rospy.core import NullHandler
from std_msgs.msg import Int32
from detection.msg import coordinate  
from sensor_msgs.msg import NavSatFix
from ar_track_alvar_msgs.msg import AlvarMarkers


def callback_rspose(data):
    #rospy.loginfo(rospy.get_caller_id() + 'I heard \n d- %d \n x- %f \n y- %f \n z- %f', data.detection, data.x, data.y, data.z)
    RS_pose = data
    print("stored d-  ", RS_pose.detection)
    print("stored x-  ", RS_pose.x)
    print("stored y-  ", RS_pose.y)
    print("stored z-  ", RS_pose.z)
def callback_jackalpose(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard latitude- %s ', data)
    jackal_location = data
    print("stored lat-  ", jackal_location.latitude)
    print("stored lon-  ", jackal_location.longitude)
    print("stored alt-  ", jackal_location.altitude)

def callback_artagpose(data):
    for ar_tag_pose in data.markers:

            marker_id = ar_tag_pose.id

            if marker_id == 1:

                marker_pose = ar_tag_pose.pose.pose

                pos = marker_pose.position

                #ori = marker_pose.orientation
                print("id -  ", marker_id)
                print("tag x-  ", pos.x)
                print("tag y-  ", pos.y)
                print("tag z-  ", pos.y)
                print("--------------------------")
            if ar_tag_pose == []:
                print("not detected")

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('detect_locate', coordinate, callback_rspose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() 
def listener1():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('jackal0/navsat/fix', NavSatFix, callback_jackalpose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
def listener2():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback_artagpose)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    

if __name__ == '__main__':
    listener2()

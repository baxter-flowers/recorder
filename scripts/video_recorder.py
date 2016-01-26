#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from os import system

rospy.init_node('video_recorder')

def record_device(device, sound, path):
    name = device.split('/')[-1]
    if sound:
        video = "avconv -f video4linux2 -video_size 1280x720 -i {} -f alsa -i hw:3 -acodec libvo_aacenc -vcodec libx264 {}/{}.avi".format(device, path, name)
    else:
        video = "avconv -f video4linux2 -video_size 1280x720 -i {} -acodec none -vcodec libx264 {}/{}.avi".format(device, path, name)
    system(video)

sound = rospy.get_param(rospy.get_name()+"/sound", default=False)
path = rospy.get_param(rospy.get_name()+"/path")
device = rospy.get_param(rospy.get_name()+"/device")

rospy.loginfo("Creating new dataset under {}".format(path))
mkdir = system("mkdir -p {}".format(path))

if mkdir == 0:
    record_device(device, sound, path)
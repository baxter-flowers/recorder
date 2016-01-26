#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import tf
from sys import argv
from os import system

class TFRecorder:
    def __init__(self, path, frames=None, rate=20., timeout=1):
        self.tfl = tf.TransformListener(True, rospy.Duration(timeout))
        self.transforms = []
        self.start_time = None
        self.path = path
        self.frames = frames
        self.world = "base"
        self.rate = rospy.Rate(rate)

    def run(self):
        self.start_time = rospy.Time.now()
        rospy.loginfo("Starting recording {}...".format(str(self.frames) if self.frames is not None else "all frames"))
        while not rospy.is_shutdown():
            for frame in self.frames:
                try:
                    lct = self.tfl.getLatestCommonTime(self.world, frame)
                    transform = self.tfl.lookupTransform(self.world, frame, lct)
                except tf.Exception, e:
                    print e.message
                    pass
                else:
                    time = (lct - self.start_time).to_sec()
                    sample = {"time": time, "object": frame, "pose": transform}
                    self.transforms.append(sample)
            self.rate.sleep()
        
        rospy.loginfo("Generating JSON file of /tf...")
        self.dump()
        rospy.loginfo("JSON generated at {}".format(self.path))

    def dump(self):
        data = {"metadata": {"world": self.world, "objects": self.frames, "timestamp" : self.start_time.to_sec()}, "transforms": self.transforms}
        with open(self.path, 'w') as f:
            json.dump(data, f)


rospy.init_node('tf_recorder')

try:
    frames = [frame for frame in rospy.get_param("/recorder/frames") if not frame.startswith("__")]
except KeyError:
    if len(argv) > 1:
        frames = argv[1:]
    else:
        raise RuntimeError("Please specify the frames to record in /record/frames or in argument")

path = rospy.get_param(rospy.get_name()+"/path")

rospy.loginfo("Creating new video dataset under {}".format(path))
mkdir = system("mkdir -p {}".format(path))

if mkdir == 0:
    TFRecorder(path+'/frames.json', frames).run()

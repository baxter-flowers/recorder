#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import json
import tf
from sys import argv
from os import system
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from threading import RLock
from copy import deepcopy
from cv_bridge.core import CvBridge
from cv2 import VideoWriter
from cv2.cv import CV_FOURCC

class Recorder:
    def __init__(self, path, frames=None, rate=20, timeout=1):
        self.tfl = tf.TransformListener(True, rospy.Duration(timeout))
        self.transforms = []
        self.start_time = None
        self.path = path
        self.frames = frames
        self.world = "base"
        self.rate_hz = rate
        self.rate = rospy.Rate(rate)
        self.microrate = rospy.Rate(1000)
        self.bridge = CvBridge()

        self.clock_lock = RLock()
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.cb_clock, queue_size=1)
        self.right_image_sub = rospy.Subscriber('/cameras/right_hand_camera/image', Image, self.cb_image_right, queue_size=1)
        self.left_image_sub = rospy.Subscriber('/cameras/left_hand_camera/image', Image, self.cb_image_left, queue_size=1)
        self.head_image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.cb_image_head, queue_size=1)
        self.image = {'left': None, 'right': None, 'head': None}
        self.locks = {'left': RLock(), 'right': RLock(), 'head': RLock()}
        self.four_cc = CV_FOURCC('F' ,'M','P', '4')
        self.writers = {'left': None, 'right': None, 'head': None}

        # Recording is triggered when all components are ready
        self.readiness_lock = RLock()
        self.components_ready = {'left': False, 'right': False, 'head': False, 'clock': not rospy.get_param('use_sim_time', default=False)}
        self.ready = False  # True when all components are ready

    def _update_readiness(self, component):
        self.components_ready[component] = True
        with self.readiness_lock:
            self.ready = len([component for component in self.components_ready if not self.components_ready[component]]) == 0

    def cb_clock(self, clock):
        self.clock_sub.unregister()
        self._update_readiness('clock')

    def cb_image_right(self, image):
        with self.locks['right']:
            self.image['right'] = image
        self.open_writer('right')
        self._update_readiness('right')

    def cb_image_left(self, image):
        with self.locks['left']:
            self.image['left'] = image
        self.open_writer('left')
        self._update_readiness('left')

    def cb_image_head(self, image):
        with self.locks['head']:
            self.image['head'] = image
        self.open_writer('head')
        self._update_readiness('head')

    def open_writer(self, camera):
        if not self.writers[camera]:
            self.writers[camera] = VideoWriter(path + '/' + camera + '.avi',
                                               self.four_cc, self.rate_hz,
                                               (self.image[camera].width, self.image[camera].height))
    def save_transforms(self):
        transformations = {}
        for frame in self.frames:
            try:
                lct = self.tfl.getLatestCommonTime(self.world, frame)
                transform = self.tfl.lookupTransform(self.world, frame, lct)
            except tf.Exception, e:
                transformations[frame] = {"visible": False}
            else:
                transformations[frame] = {"visible": True, "pose": transform}
        sample = {"time": (rospy.Time.now() - self.start_time).to_sec(), "objects": transformations}
        self.transforms.append(sample)

    def save_image(self, camera):
        with self.locks[camera]:
            ros_image = deepcopy(self.image[camera])
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        self.writers[camera].write(cv_image)

    def dump(self):
        rospy.loginfo("Generating JSON file of /tf...")
        file = self.path + '/frames.json'
        data = {"metadata": {"world": self.world, "objects": self.frames, "timestamp" : self.start_time.to_sec()}, "transforms": self.transforms}
        with open(file, 'w') as f:
            json.dump(data, f)
        rospy.loginfo("JSON generated at {}".format(file))
        for camera in ['right', 'left', 'head']:
            rospy.loginfo("Generating video file, side {}...".format(camera))
            if self.writers[camera]:
                self.writers[camera].release()
                rospy.loginfo("Video file side {} generated".format(camera))
            else:
                rospy.logwarn("Cannot generate file {} or no data to save".format(camera))

    def wait_subscribers(self):
        rospy.loginfo('Waiting all subscribers to be ready... if it never ends check that all cameras are publishing on their topics')
        while not self.ready and not rospy.is_shutdown():
            self.microrate.sleep()

    def run(self):
        self.wait_subscribers()
        raw_input("Subscribers ready, press <Enter> to start recording...")
        self.start_time = rospy.Time.now()
        rospy.loginfo("Starting recording {}...".format(str(self.frames) if self.frames is not None else "all frames"))
        try:
            while not rospy.is_shutdown():
                self.save_transforms()
                self.save_image('right')
                self.save_image('left')
                self.save_image('head')
                self.rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
        finally:
            self.dump()

if __name__=='__main__':
    rospy.init_node('tf_recorder')

    try:
        frames = rospy.get_param("/recorder/frames")
    except KeyError:
        if len(argv) > 1:
            frames = [frame for frame in argv[1:] if not frame.startswith("__")]
        else:
            raise RuntimeError("Please specify the frames to record in /recorder/frames or in argument")

    path = rospy.get_param("/recorder/path", "/tmp")

    rospy.loginfo("Creating new empty dataset under {}".format(path))
    mkdir = system("mkdir -p {}".format(path))

    if mkdir == 0:
        Recorder(path, frames).run()
    else:
        rospy.logerr("Unable to create dataset path {}, mkdir returned {}".format(path, mkdir))

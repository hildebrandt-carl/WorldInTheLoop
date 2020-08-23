#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import colors

import sys
import rospy
import cv2
import yaml
import os


class ColorThreshold:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('ColorThresholdNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)
        self._quit = False

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Create the bridge
        self.bridge = CvBridge()

        # Read the config file
        self._readConfig()

        # Define the upper and lower bound
        lb = rospy.get_param(rospy.get_name() + '/lower_bound', {'H': 0, 'S': 0, 'V': 40})
        ub = rospy.get_param(rospy.get_name() + '/upper_bound', {'H': 20, 'S': 200, 'V': 200})
        self.lower_bound = (lb["H"], lb["S"], lb["V"])
        self.upper_bound = (ub["H"], ub["S"], ub["V"])
        
        # Get the name of the topic we want to be using
        topic_out = rospy.get_param(rospy.get_name() + '/output_topic', "/color_threshold/output")

        # Init all the publishers and subscribers
        self.image_sub = rospy.Subscriber(self.config['camera_overlay_source'], Image, self._getImage)

        # Publish the image with the center
        self.img_pub = rospy.Publisher(topic_out, Image, queue_size=10)

    def _readConfig(self):
        home_dir = os.path.abspath(os.curdir)[:-4]
        with open(home_dir + '/Desktop/MixedRealityTesting/monocular_avoidance_ws/src/mixed_reality/config/config.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _getImage(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Convert to hsv
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Get the pixels that are within the range
        mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)
        result = cv2.bitwise_and(img, img, mask=mask)
           
        # Convert back to ros message and publish
        image_message = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
        self.img_pub.publish(image_message)

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit:
            pass
            r.sleep()


if __name__ == "__main__":
    try:
        x = ColorThreshold()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from time import sleep
from std_msgs.msg import Int16
# from sensor_msgs.msg import Image
from drone_controller.msg import Move
from darknet_ros_msgs.msg import BoundingBoxes
import numpy as np
from utility import DroneState

import sys
import rospy
import cv2
import time
from cv_bridge import CvBridge


class SpinningYaw:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('YawNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._inyawmode = False

        # Init all the publishers and subscribers
        self.move_pub = rospy.Publisher("uav1/input/move", Move, queue_size=10)
        self.drone_state_sub = rospy.Subscriber("uav1/input/state", Int16, self._getstate)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getstate(self, msg):
        if msg.data == DroneState.YAWNAVIGATION.value:
            self._inyawmode = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit:
            if self._inyawmode:
                
                msg = Move()
                msg.left_right = 0
                msg.front_back = 0
                msg.yawl_yawr = 75
                msg.up_down = 0
                self.move_pub.publish(msg)
            
            r.sleep()

if __name__ == "__main__":
    try:
        x = SpinningYaw()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

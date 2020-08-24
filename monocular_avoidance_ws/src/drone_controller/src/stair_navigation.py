#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from time import sleep
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from drone_controller.msg import Move
import numpy as np
from utility import DroneState
from cv_bridge import CvBridge
from matplotlib.colors import hsv_to_rgb
from matplotlib import colors
from scipy import ndimage

import sys
import rospy
import cv2
import time
import math


class GateNavigation:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('GateNavigationNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Move down counter
        self.move_down = self.rate * 1

        # Init the drone and program state
        self._quit = False 
        self._innavigationmode = False     
     
        # Init all the publishers and subscribers
        self.move_pub = rospy.Publisher("/uav1/input/beforeyawcorrection/move", Move, queue_size=10)
        self.drone_state_sub = rospy.Subscriber("uav1/input/state", Int16, self._getstate)


    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getstate(self, msg):
        if msg.data == DroneState.STAIRNAVIGATION.value:
            self._innavigationmode = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        down_counter = 0

        while not self._quit:
            if self._innavigationmode:

                if down_counter < self.move_down:
                    # Move down as low to the ground as it can go
                    direction_y = 0
                    direction_x = 1
                    direction_backforward = 0
                    # Add to the down counter so that it eventually stops moving down
                    down_counter += 1
                else:
                    # Move towards the staircase
                    direction_y = 0
                    direction_x = 0
                    direction_backforward = -1           

                msg = Move()
                msg.left_right = int(direction_y * 25)
                msg.front_back = int(direction_backforward * 10)
                msg.yawl_yawr = 0
                msg.up_down = int(direction_x * 25)
                self.move_pub.publish(msg)
            
            r.sleep()

if __name__ == "__main__":
    try:
        x = GateNavigation()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

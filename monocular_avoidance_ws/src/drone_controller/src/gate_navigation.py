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

        # Init the drone and program state
        self._quit = False
        self._innavigationmode = False
        self._gatefound = True

        # Holds the center of the image:
        self.camera_center_x = None
        self.camera_center_y = None

        # Used to keep track of the object
        self.gate_center_x_arr = None
        self.gate_center_y_arr = None
        self.object_arr_length = 10

        # Create the bridge
        self.bridge = CvBridge()

        # Define the upper and lower bound
        self.lower_bound = (0, 0, 40)
        self.upper_bound = (20, 200, 200)

        # Init all the publishers and subscribers
        self.move_pub = rospy.Publisher("uav1/input/move", Move, queue_size=10)
        self.drone_state_sub = rospy.Subscriber("uav1/input/state", Int16, self._getstate)
        self.image_sub = rospy.Subscriber("/mixer/sensors/camera", Image, self._getImage)

        # Publish the image with the center
        self.img_pub = rospy.Publisher("/gate_navigation/sensors/camera", Image, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getstate(self, msg):
        if msg.data == DroneState.GATENAVIGATION.value:
            self._innavigationmode = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _getImage(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Save the size of the image
        if self.camera_center_x is None or self.camera_center_y is None:
            self.camera_center_x = msg.width / 2.0
            self.camera_center_y = msg.height / 2.0

        # Convert to hsv
        rbg_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        hsv_img = cv2.cvtColor(rbg_img, cv2.COLOR_RGB2HSV)

        # Compute where the center is
        mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound)
        gate_centre_y, gate_centre_x = ndimage.measurements.center_of_mass(mask)

        if math.isnan(gate_centre_x) or math.isnan(gate_centre_y):
            if self._gatefound:
                self._log("No gate found.")
                self._gatefound = False
            return
        else:
            self._gatefound = True

        # Save the results
        if self.gate_center_x_arr is None:
            self.gate_center_x_arr = np.full(self.object_arr_length, gate_centre_x)
            self.gate_center_y_arr = np.full(self.object_arr_length, gate_centre_y)
        else:
            self.gate_center_x_arr = np.roll(self.gate_center_x_arr, 1)
            self.gate_center_y_arr = np.roll(self.gate_center_y_arr, 1)
            self.gate_center_x_arr[0] = gate_centre_x
            self.gate_center_y_arr[0] = gate_centre_y

        # Publish the image with the dot
        center_coordinates = (int(round(gate_centre_x, 0)), int(round(gate_centre_y, 0)))
        radius = 2
        color = (255, 0, 0) 
        thickness = 2
        rbg_img = cv2.circle(rbg_img, center_coordinates, radius, color, thickness) 

        # Convert back to ros message and publish
        image_message = self.bridge.cv2_to_imgmsg(rbg_img, encoding="rgb8")
        self.img_pub.publish(image_message)

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        stop_counter = 0

        while not self._quit:
            if self._innavigationmode:
                # Calculate what the drones current direction
                try:
                    cen_x = np.average(self.gate_center_x_arr)
                    cen_y = np.average(self.gate_center_y_arr)
                except:
                    continue

                percentage_updown       = (cen_x - self.camera_center_x) / self.camera_center_x
                percentage_leftright    = (cen_y - self.camera_center_y) / self.camera_center_y

                direction_y = percentage_updown
                direction_x = percentage_leftright
                direction_backforward = -1
 
                if self._gatefound:
                    stop_counter = 0
                else: 
                    stop_counter += 1
                    # Wait 2 seconds before stopping your action
                    if stop_counter <= self.rate * 1.5:
                        direction_y = 0
                        direction_x = 0
                        direction_backforward = -1
                    else:
                        direction_y = 0
                        direction_x = 0
                        direction_backforward = 0
                    

                direction_y = 0
                direction_x = 0
                direction_backforward = 0

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

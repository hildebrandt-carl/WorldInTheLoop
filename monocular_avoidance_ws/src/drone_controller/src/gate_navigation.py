#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from matplotlib.colors import hsv_to_rgb
from cv_bridge import CvBridge
from matplotlib import colors
from scipy import ndimage
from time import sleep

from pid_class import PID
from utility import DroneState

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from drone_controller.msg import Move

import numpy as np

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
        self.rate = 30.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._innavigationmode = False
        self._gatefound = True

        # Used to compute the total mass of gate in each part of the image
        self.upper_left_mass  = 0
        self.upper_right_mass = 0
        self.lower_left_mass  = 0
        self.lower_right_mass = 0

        # Used to switch navigation mode from full gate to close gate navigation
        self.close_gate_navigation = False

        # Holds the center of the image:
        self.camera_center_x = None
        self.camera_center_y = None

        # Used to keep track of the object
        self.gate_center_x_arr = None
        self.gate_center_y_arr = None
        self.object_arr_length = 10

        # PID Class to navigate to the gate
        gains = rospy.get_param(rospy.get_name() + '/gains', {'p': 1.0, 'i': 0.0, 'd': 0.0})
        Kp, Ki, Kd = gains['p'], gains['i'], gains['d']
        self.z_controller = PID(Kp, Ki, Kd, self.rate)
        self.y_controller = PID(Kp, Ki, Kd, self.rate)

        # Display the variables
        self._log(": p - " + str(Kp))
        self._log(": i - " + str(Ki))
        self._log(": d - " + str(Kd))

        # Create the bridge
        self.bridge = CvBridge()

        # Variable to check if movement is allowed
        self.allow_movement = True

        # Variable to tell the algorithm if we can see the whole gate or only partial gate
        self.full_gate_found = False
        self.full_gate_ever_found = False

        # Used to store the current gates area (used as a metric for distance to gate)
        self._gate_area = 0

        # Define the upper and lower bound
        lb = rospy.get_param(rospy.get_name() + '/lower_bound', {'H': 0, 'S': 0, 'V': 40})
        ub = rospy.get_param(rospy.get_name() + '/upper_bound', {'H': 20, 'S': 200, 'V': 200})
        self.lower_bound = (lb["H"], lb["S"], lb["V"])
        self.upper_bound = (ub["H"], ub["S"], ub["V"])

        # Init all the publishers and subscribers
        self.move_pub           = rospy.Publisher("/uav1/input/beforeyawcorrection/move", Move, queue_size=10)
        self.drone_state_sub    = rospy.Subscriber("/uav1/input/state", Int16, self._getstate)
        self.image_sub          = rospy.Subscriber("/mixer/sensors/camera", Image, self._getImage)
        self.allow_movement_sub = rospy.Subscriber("/uav1/status/yaw_out_of_range", Bool, self._getOutOfRange)

        # Publish the image with the center
        self.img_pub = rospy.Publisher("/gate_navigation/sensors/camera", Image, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getOutOfRange(self, msg):
        # Set the allow_movement flag to the incoming message
        self.allow_movement = msg.data

    def _getstate(self, msg):
        # Check if we are in navigation mode
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

        # Make sure we are looking at a full gate and not only an image
        thresh = cv2.adaptiveThreshold(mask, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contour_list = []
        area_list = [0]
        for contour in contours:
            area = cv2.contourArea(contour)
            # Filter based on length and area
            if (area > 20000):
                contour_list.append(contour)
                area_list.append(area)
        
        # Get the largest gate area
        self._gate_area = max(area_list)

        # Check if there are any full gates found:
        if len(contour_list) <= 0:
            self.full_gate_found = False
        else:
            self.full_gate_found = True
            self.full_gate_ever_found = True

        rbg_img = cv2.drawContours(rbg_img, contour_list,  -1, (0, 255, 0), 2)

        # Draw a horizontal and vertical line through the image
        point1_vertical = (int(rbg_img.shape[1]/2), 0)
        point2_vertical = (int(rbg_img.shape[1]/2), img.shape[0])
        point1_horizontal = (0, int(rbg_img.shape[0]/2))
        point2_horizontal = (img.shape[1], int(rbg_img.shape[0]/2))
        rbg_img = cv2.line(rbg_img, pt1=point1_vertical, pt2=point2_vertical, color=(0, 0, 255), thickness=2)
        rbg_img = cv2.line(rbg_img, pt1=point1_horizontal, pt2=point2_horizontal, color=(0, 0, 255), thickness=2)

        # If we are too close for full navigation
        if self.close_gate_navigation:
            # Reset the center point
            gate_centre_x = self.camera_center_x
            gate_centre_y = self.camera_center_y

            # Compute the total mass of gate in each section of the image
            mask_horizontal_center = int(mask.shape[0] / 2)
            mask_vertical_center = int(mask.shape[1] / 2)
            self.upper_left_mass  = np.sum(mask[0:mask_horizontal_center, 0:mask_vertical_center])
            self.upper_right_mass = np.sum(mask[0:mask_horizontal_center, mask_vertical_center:])
            self.lower_left_mass  = np.sum(mask[mask_horizontal_center:, 0:mask_vertical_center])
            self.lower_right_mass = np.sum(mask[mask_horizontal_center:, mask_vertical_center:])

            # Check if there is gate on the upper left
            if self.upper_left_mass >= 1000:
                gate_centre_x += 0.025 * mask.shape[1] 
                gate_centre_y += 0.025 * mask.shape[0] 
            # Check if there is gate on the upper right
            if self.upper_right_mass >= 1000:
                gate_centre_x -= 0.025 * mask.shape[1] 
                gate_centre_y += 0.025 * mask.shape[0] 
            # Check if there is gate on the lower left
            if self.lower_left_mass >= 1000:
                gate_centre_x += 0.025 * mask.shape[1] 
                gate_centre_y -= 0.025 * mask.shape[0] 
            # Check if there is gate on the lower right
            if self.lower_right_mass >= 1000:
                gate_centre_x -= 0.025 * mask.shape[1] 
                gate_centre_y -= 0.025 * mask.shape[0]

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
        color = (0, 255, 0) 
        thickness = 2
        rbg_img = cv2.circle(rbg_img, center_coordinates, radius, color, thickness) 

        # Convert back to ros message and publish
        image_message = self.bridge.cv2_to_imgmsg(rbg_img, encoding="rgb8")
        self.img_pub.publish(image_message)

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        # Hold samples for the last 2 seconds
        total_samples = int(self.rate * 2)
        # These arrays hold the values used to line up with the gate.
        line_up_window_y = np.full((total_samples,), 10)
        line_up_window_z = np.full((total_samples,), 10)

        move_forward = False
        stop_counter = 0
        
        while not self._quit:
            if self._innavigationmode:
                # Calculate what the drones current direction
                try:
                    cen_x = np.average(self.gate_center_x_arr)
                    cen_y = np.average(self.gate_center_y_arr)
                except:
                    continue

                # Calculate the error
                percentage_leftright = (cen_x - self.camera_center_x) / self.camera_center_x
                percentage_updown    = (cen_y - self.camera_center_y) / self.camera_center_y

                # Put the error into the corresponding PID controller
                leftright   = self.y_controller.get_output(0, -1 * percentage_leftright)
                updown      = self.z_controller.get_output(0, -1 * percentage_updown)
                
                # Set the output
                direction_y = leftright
                direction_z = updown
                direction_backforward = 0

                # Record the values and check if on average we have lined up
                if (self._gatefound): 
                    line_up_window_y = np.roll(line_up_window_y, 1)
                    line_up_window_z = np.roll(line_up_window_z, 1)
                    line_up_window_y[0] = abs(direction_y)
                    line_up_window_z[0] = abs(direction_z)

                # Check if we have lined up
                avg_y = np.mean(line_up_window_y) 
                avg_z = np.mean(line_up_window_z)
                # if (avg_y <= 1) and (avg_z <= 1) and (not move_forward):
                if (avg_y <= 1.5) and (avg_z <= 1.5) and (not move_forward):
                    self._log("Moving forward with visual navigation")
                    move_forward = True
                # While you are moving towards the gate
                elif ((avg_y >= 2) or (avg_z >= 2)) and (move_forward):
                # elif ((avg_y >= 2) or (avg_z >= 2)) and (move_forward):
                    # Only pause if we have found the gate and its not too close
                    if self._gatefound and self.full_gate_found and self._gate_area <= 180000:
                        self._log(self._gate_area)
                        self._log("Pausing for re-alignment")
                        move_forward = False
                        # Send quick burst to kill forward momemtum
                        direction_backforward = 100

                # If we want to start moving forward, as the gate is now lined up
                if move_forward == True and not self.close_gate_navigation:
                    direction_backforward = -1
                    # If we cant see the gate anymore and were moving forward, we need to switch to move forward only mode
                    if not self.full_gate_found:
                        self._log("Gate too close, switching to quadrant navigation")
                        self.close_gate_navigation = True

                # Fly forward to make it through the gate
                if self.close_gate_navigation:
                    if self._gatefound:
                        # The center of mass is now calculated different and so we can use it to do the final navigation
                        direction_y = leftright
                        direction_z = -1 * updown
                        direction_backforward = -5
                        # If we cant see any gate go straight
                    else:
                        direction_y = 0
                        direction_z = 0
                        direction_backforward = -7

                # Move backwards so you can see the gate again
                if (not self.full_gate_found) and (not self.close_gate_navigation):
                    direction_backforward = 2

                # If the full gate has never been found slowly move forward until you find it
                if (self.full_gate_ever_found == False):
                    direction_backforward = -1

                # Allow movement is triggered if the yaw is off
                if not self.allow_movement:
                    direction_y = 0
                    direction_z = 0
                    direction_backforward = 0

                # Make sure the commands are int's between -100 and 100
                direction_y = int(round(max(min(direction_y, 100), -100),0))
                direction_z = int(round(max(min(direction_z, 100), -100),0))
                direction_backforward = int(round(max(min(direction_backforward, 100), -100),0))

                # If you can't see a gate anymore reset PID and send stop command
                if self._gatefound:
                    stop_counter = 0
                else: 
                    self.z_controller.remove_buildup()
                    self.y_controller.remove_buildup()
                    stop_counter += 1
                    # If we havent seen the gate for 2 seconds stop
                    if stop_counter >= self.rate * 2:
                        direction_y = 0
                        direction_z = 0
                        direction_backforward = 0


                # Publish the message
                msg = Move()
                msg.left_right = direction_y
                msg.up_down = direction_z
                msg.front_back = direction_backforward
                msg.yawl_yawr = 0
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

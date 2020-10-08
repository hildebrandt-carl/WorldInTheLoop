#!/usr/bin/env python3
from __future__ import print_function, absolute_import

import sys
import rospy
import cv2
import time

import numpy as np

from time import sleep
from utility import DroneState
from cv_bridge import CvBridge
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from drone_controller.msg import Move
from darknet_ros_msgs.msg import BoundingBoxes


class AvoidanceNavigation:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('AvoidanceNavigationNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._inavoidancemode = False

        # Used to keep track of the object
        self.object_center_x_arr = None
        self.object_center_y_arr = None
        self.object_size_arr = None
        self.object_arr_length = 10

        # Init all the publishers and subscribers
        self.move_pub           = rospy.Publisher("/uav1/input/beforeyawcorrection/move", Move, queue_size=10)
        self.drone_state_sub    = rospy.Subscriber("uav1/input/state", Int16, self._getstate)
        self.bounding_sub       = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self._getbounding)
        self.debug_area_pub     = rospy.Publisher("/debug/second_drone_area", Float32 , queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getstate(self, msg):
        if msg.data == DroneState.AVOIDING.value:
            self._inavoidancemode = True

    def _getbounding(self, msg):
        # For each bounding box
        for box in msg.bounding_boxes:
            # If it is an airplane
            if box.Class == "aeroplane":
                len_side_x = box.xmax - box.xmin
                len_side_y = box.ymax - box.ymin

                obj_centre_x = (len_side_x / 2.0) + box.xmin
                obj_centre_y = (len_side_y / 2.0) + box.ymin

                obj_area = len_side_x * len_side_y

                if self.object_center_x_arr is None:
                    self.object_center_x_arr = np.full(self.object_arr_length, obj_centre_x)
                    self.object_center_y_arr = np.full(self.object_arr_length, obj_centre_y)
                    self.object_size_arr = np.full(self.object_arr_length, obj_area)
                else:
                    self.object_center_x_arr = np.roll(self.object_center_x_arr, 1)
                    self.object_center_y_arr = np.roll(self.object_center_y_arr, 1)
                    self.object_size_arr = np.roll(self.object_size_arr, 1)
                    self.object_center_x_arr[0] = obj_centre_x
                    self.object_center_y_arr[0] = obj_centre_y
                    self.object_size_arr[0] = obj_area

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        movement_f = None
        movement_x = None
        movement_y = None
        compare_back_distance = 30

        while not self._quit:
            if self._inavoidancemode:

                # Init variables
                moving_towards = False
                direction_x = 0
                direction_y = 0
                direction_z = 0

                # Calculate what the drones current points
                try:
                    area  = np.average(self.object_size_arr)
                    cen_x = np.average(self.object_center_x_arr)
                    cen_y = np.average(self.object_center_y_arr)
                except:
                    continue

                # Keep a list of previous recordings to allow a compare
                if movement_f is None:
                    movement_f = np.full(compare_back_distance, area)
                    movement_x = np.full(compare_back_distance, cen_x)
                    movement_y = np.full(compare_back_distance, cen_y)
                else:
                    movement_f = np.roll(movement_f, 1)
                    movement_x = np.roll(movement_x, 1)
                    movement_y = np.roll(movement_y, 1)
                    movement_f[0] = area
                    movement_x[0] = cen_x
                    movement_y[0] = cen_y


                # If the current area is 2 times larger than the previous one, the drone is moving towards us
                if movement_f[-1] * 2.5 < movement_f[0]:
                    moving_towards = True
                
                # If it is moving towards us
                if moving_towards:
                    # Always move up for now
                    direction_z = -0.3
                    direction_str = "up"
                    
                if moving_towards:
                    self._log("Object moving towards drone detected! Moving: " + str(direction_str))

                # Scale between up to 100
                direction_y = direction_y * 100
                direction_x = direction_x * 100
                direction_z = direction_z * 100

                # Make sure the commands are int's between -100 and 100
                direction_y = int(round(max(min(direction_y, 100), -100),0))
                direction_z = int(round(max(min(direction_z, 100), -100),0))
                direction_x = int(round(max(min(direction_x, 100), -100),0))

                # Publish the move commands
                msg = Move()
                msg.left_right = direction_y
                msg.up_down = direction_z
                msg.front_back = direction_x
                msg.yawl_yawr = 0
                self.move_pub.publish(msg)

                # Publish the recorded area for debugging
                self.debug_area_pub.publish(Float32(area))
            
            r.sleep()

if __name__ == "__main__":
    try:
        x = AvoidanceNavigation()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

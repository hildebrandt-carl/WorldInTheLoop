#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from time import sleep
from std_msgs.msg import Int16
# from sensor_msgs.msg import Image
from drone_controller.msg import Move
from darknet_ros_msgs.msg import BoundingBoxes
import numpy as np

import sys
import rospy
import cv2
import time
from cv_bridge import CvBridge


class Avoidance:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('AvoidanceNode')
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
        self.move_pub = rospy.Publisher("uav1/input/move", Move, queue_size=10)
        self.avoidance_sub = rospy.Subscriber("uav1/input/state", Int16, self._setstate)
        # self.image_sub = rospy.Subscriber("darknet_ros/detection_image", Image, self._getimagedetails)
        self.bounding_sub = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self._getbounding)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _setstate(self, msg):
        if msg.data == 3:
            self._inavoidancemode = True

    def _getbounding(self, msg):
        for box in msg.bounding_boxes:
            if box.id == 4:
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
                

    def _setstate(self, msg):
        if msg.data == 3:
            self._inavoidancemode = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        movement_f = None
        movement_x = None
        movement_y = None
        delay = 5

        while not self._quit:
            if self._inavoidancemode:
                # Calculate what the drones current points
                try:
                    area  = np.average(self.object_size_arr)
                    cen_x = np.average(self.object_center_x_arr)
                    cen_y = np.average(self.object_center_y_arr)
                except:
                    continue

                if movement_f is None:
                    movement_f = np.full(delay, area)
                    movement_x = np.full(delay, cen_x)
                    movement_y = np.full(delay, cen_y)
                else:
                    movement_f = np.roll(movement_f, 1)
                    movement_x = np.roll(movement_x, 1)
                    movement_y = np.roll(movement_y, 1)
                    movement_f[0] = area
                    movement_x[0] = cen_x
                    movement_y[0] = cen_y

                moving_towards = False
                direction_x = 0
                direction_y = 0

                if movement_f[-1] * 1.1 < movement_f[0]:
                    moving_towards = True
                
                
                if moving_towards:
                    # if movement_x[-1] < movement_x[0]:
                    #     direction_y = -1
                    #     direction_y_str = "left"
                    # else:
                    #     direction_y = 1
                    #     direction_y_str = "right"
                    # if movement_y[-1] < movement_y[0]:
                    #     direction_x = -1
                    #     direction_x_str = "up"
                    # else:
                    #     direction_x = 1
                    #     direction_x_str = "down"

                    # Always move up for now
                    direction_x = -1
                    direction_x_str = "up"
                    direction_y = 0
                    
                if moving_towards:
                    # self._log("Object moving towards drone detected! Moving: " + str(direction_y_str) + ", " + str(direction_x_str))
                    self._log("Object moving towards drone detected! Moving: " + str(direction_x_str))


                msg = Move()
                msg.left_right = direction_y * 100
                msg.front_back = 0
                msg.yawl_yawr = 0
                msg.up_down = direction_x * 100
                self.move_pub.publish(msg)
            
            r.sleep()

if __name__ == "__main__":
    try:
        x = Avoidance()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

#!/usr/bin/env python3
from __future__ import print_function, absolute_import
from std_msgs.msg import Int16
# from sensor_msgs.msg import Image
from drone_controller.msg import Move
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image
import numpy as np
from utility import DroneState
from pid_class import PID
import math
from std_msgs.msg import Float32

import sys
import rospy
import cv2
from cv_bridge import CvBridge


class PersonNavigation:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('PersonNavigationNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._infollowmode = False

        # Used to keep track of the object
        self.object_center_x_arr = None
        self.object_center_y_arr = None
        self.object_size_arr = None
        self.object_arr_length = 10

        self.image_size = None

        # PID Class to keep the person in view
        gains = rospy.get_param(rospy.get_name() + '/gains', {'p': 1.0, 'i': 0.0, 'd': 0.0})
        Kp, Ki, Kd = gains['p'], gains['i'], gains['d']
        self.distance_controller = PID(Kp, Ki, Kd, self.rate)

        # Display the variables
        self._log(": p - " + str(Kp))
        self._log(": i - " + str(Ki))
        self._log(": d - " + str(Kd))

        # Init all the publishers and subscribers
        self.move_pub           = rospy.Publisher("/uav1/input/beforeyawcorrection/move", Move, queue_size=10)
        self.drone_state_sub    = rospy.Subscriber("uav1/input/state", Int16, self._getstate)
        self.bounding_sub       = rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self._getbounding)
        self.camera_sub         = rospy.Subscriber("/mixer/sensors/camera", Image, self._getimagesize)
        self.vicon_yaw_pub      = rospy.Publisher("/uav1/input/vicon_setpoint/yaw/rate", Float32, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getstate(self, msg):
        if msg.data == DroneState.FOLLOWPERSON.value:
            self._infollowmode = True

    def _getimagesize(self, msg):
        data_size = (msg.width, msg.height)
        # Get the size of the image
        if self.image_size is None:
            self.image_size = data_size
        else:
            if self.image_size != data_size:
                self.image_size = data_size

    def _getbounding(self, msg):

        # Arrays to store the bounding box information
        person_data = []

        # Go through each bounding box
        for box in msg.bounding_boxes:
            # If it is a person
            if box.Class == "person" and box.probability >= 0.9:
                print("Person found")
                person_data.append([box.xmin, box.xmax, box.ymin, box.ymax])

        # If we have people data
        if len(person_data) <= 0:
            return

        else:
            # If there is more than one person
            if len(person_data) > 1:
                max_area = 0
                max_data = []
                # Find the largest person
                for person in person_data:
                    area = (person[1] - person[0]) * (person[3] - person[2])
                    if area > max_area:
                        max_area = area
                        max_data = person

                xmin = max_data[0]
                xmax = max_data[1]
                ymin = max_data[2]
                ymax = max_data[3]
            else:
                xmin = person_data[0][0]
                xmax = person_data[0][1]
                ymin = person_data[0][2]
                ymax = person_data[0][3]

            # Compute the area and the center of the object
            len_side_x = xmax - xmin
            len_side_y = ymax - ymin

            obj_centre_x = (len_side_x / 2.0) + xmin
            obj_centre_y = (len_side_y / 2.0) + ymin

            obj_area = len_side_x * len_side_y

            # If we have never computed a center before
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

        while not self._quit:
            if self._infollowmode:

                area = 0
                cen_x = 0
                cen_y = 0

                # Calculate what the drones current points
                try:
                    area  = np.average(self.object_size_arr)
                    cen_x = np.average(self.object_center_x_arr)
                    cen_y = np.average(self.object_center_y_arr)

                    print("Human center: " + str(cen_x))
                    print("Image size: " + str(self.image_size[0]))

                    # Get a percentage away from the center lines
                    cen_x = (cen_x - self.image_size[0] / 2.0) / (self.image_size[0] / 2.0)
                    cen_y = (cen_y - self.image_size[1] / 2.0) / (self.image_size[1] / 2.0)

                    # How much of the screen you want to take up (i.e. 30th of the image)
                    percentage_covered = 1 / 30.0
                    desired_area = (self.image_size[0] * self.image_size[1]) * percentage_covered
                    area = (area - desired_area) / desired_area

                    forward_backward = self.distance_controller.get_output(0, -1 * area)
                    forward_backward = min(max(forward_backward, -1), 1)

                except (TypeError):
                    continue

                direction_yaw = 0
                direction_x = 0
                direction_y = 0
                direction_z = forward_backward

                # Dont move forward for initial tests
                direction_z = 0

                msg = Move()
                msg.up_down     = int(round(direction_x * 100, 0))
                msg.left_right  = int(round(direction_y * 100, 0))
                msg.front_back  = int(round(direction_z * 100, 0))
                msg.yawl_yawr   = int(round(cen_x * 100, 0))

                print("Cent X: " + str(cen_x))
                # Compute how much to change the yaw in degrees
                yaw_rate = math.radians(-cen_x * 20)
                print("Yaw Rate: " + str(yaw_rate))
                # yaw_rate = 0
                
                self.vicon_yaw_pub.publish(Float32(yaw_rate))
                print("----------------")
                
                # Publish the move command
                self.move_pub.publish(msg)

            r.sleep()

if __name__ == "__main__":
    try:
        x = PersonNavigation()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

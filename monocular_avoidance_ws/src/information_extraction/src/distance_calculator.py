#!/usr/bin/env python3
from __future__ import print_function, absolute_import

import rospy
import sys
import time
import copy
import math
import csv

import numpy as np

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class DistanceCalculator:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('DistanceCalculatorNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the drone positions
        self.drone1_position   = np.zeros(3)
        self.drone2_position   = np.zeros(3)

        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Init the program state
        self._quit = False

        # Init all the publishers and subscribers
        self.drone1_sub = rospy.Subscriber("ground_truth/uav1/pose", PoseStamped, self.pose1_callback)
        self.drone2_sub = rospy.Subscriber("ground_truth/uav2/pose", PoseStamped, self.pose2_callback)

        self.distance_pub = rospy.Publisher("ground_truth/uav1/uav2/distance", Float32, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def pose1_callback(self, msg):
        # Get the current position
        self.drone1_position[0] = copy.deepcopy(msg.pose.position.x)
        self.drone1_position[1] = copy.deepcopy(msg.pose.position.y)
        self.drone1_position[2] = copy.deepcopy(msg.pose.position.z)

    def pose2_callback(self, msg):
        # Get the current position
        self.drone2_position[0] = copy.deepcopy(msg.pose.position.x)
        self.drone2_position[1] = copy.deepcopy(msg.pose.position.y)
        self.drone2_position[2] = copy.deepcopy(msg.pose.position.z)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))
        
    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit:
            
            # Compute the distance between the two drones
            dx = self.drone1_position[0] - self.drone2_position[0]
            dy = self.drone1_position[1] - self.drone2_position[1]
            dz = self.drone1_position[2] - self.drone2_position[2]
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2))

            # Publish the distance
            self.distance_pub.publish(Float32(distance))

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    # Run the node
    try:
        x = DistanceCalculator()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)




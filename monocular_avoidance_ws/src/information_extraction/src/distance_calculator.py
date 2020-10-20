#!/usr/bin/env python3
from __future__ import print_function, absolute_import

import rospy
import sys
import time
import copy
import math
import csv

import numpy as np

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32


class DistanceCalculator:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('DistanceCalculatorNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the drone positions
        self.object1_position   = np.zeros(3)
        self.object2_position   = np.zeros(3)

        # Get the topic names
        self.object1_topic      = rospy.get_param(rospy.get_name() + '/object1_topic', "ground_truth/uav1/pose")
        self.object2_topic      = rospy.get_param(rospy.get_name() + '/object2_topic', "ground_truth/uav2/pose")
        self.output_topic_base  = rospy.get_param(rospy.get_name() + '/output_base', "ground_truth/uav1uav2")

        # Print out the parameters
        self._log("Object1 topic: " + str(self.object1_topic))
        self._log("Object2 topic: " + str(self.object2_topic))
        self._log("Output topic base: " + str(self.output_topic_base))

        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Compute the min distance 
        self.minimum_distance = 100

        # Init the program state
        self._quit = False

        # Init all the publishers and subscribers
        self.object1_sub = rospy.Subscriber(self.object1_topic, PoseStamped, self.pose1_callback)
        self.object2_sub = rospy.Subscriber(self.object2_topic, PoseStamped, self.pose2_callback)
        self.distance_pub = rospy.Publisher(self.output_topic_base + "/distance", Float32, queue_size=10)
        self.misallignment_pub = rospy.Publisher(self.output_topic_base + "/misalignment", Vector3, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def pose1_callback(self, msg):
        # Get the current position
        self.object1_position[0] = copy.deepcopy(msg.pose.position.x)
        self.object1_position[1] = copy.deepcopy(msg.pose.position.y)
        self.object1_position[2] = copy.deepcopy(msg.pose.position.z)

    def pose2_callback(self, msg):
        # Get the current position
        self.object2_position[0] = copy.deepcopy(msg.pose.position.x)
        self.object2_position[1] = copy.deepcopy(msg.pose.position.y)
        self.object2_position[2] = copy.deepcopy(msg.pose.position.z)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))
        
    def _mainloop(self):

        r = rospy.Rate(self.rate)

        # Make sure we wait a few seconds before recording the min
        counter = 0
        
        while not self._quit:
            
            print(self.object1_position[0])
            print(self.object2_position[0])
            print("---------------------")
            # Compute the distance between the two drones
            dx = self.object1_position[0] - self.object2_position[0]
            dy = self.object1_position[1] - self.object2_position[1]
            dz = self.object1_position[2] - self.object2_position[2]
            distance = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2) + math.pow(dz, 2))

            # Publish the distance
            self.misallignment_pub.publish(Vector3(dx, dy, dz))
            self.distance_pub.publish(Float32(distance))

            # Save minimum distance
            if (distance < self.minimum_distance) and (counter > (self.rate * 5)):
                self.minimum_distance = distance
            else:
                counter += 1

            # Mantain the rate
            r.sleep()
        
        # Print the min distance
        self._log("Minimum distance: " + str(self.minimum_distance))

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




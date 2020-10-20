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


class ForceCalculator:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('ForceCalculatorNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the drone positions
        self.current_position   = np.zeros(3)
        self.current_time   = 0

        # Set the rate
        self.rate = 3.0
        self.dt = 1.0 / self.rate

        # Get the parameters for this
        self.mass       = rospy.get_param(rospy.get_name() + '/drone_mass', 0.32)
        self.topic_name = rospy.get_param(rospy.get_name() + '/topic_name', '/ground_truth/uav1')

        # Print out the parameters
        self._log("Drone mass: " + str(self.mass) + "kg")
        self._log("Topic name: " + str(self.topic_name))

        # Init the program state
        self._quit = False

        # Init all the publishers and subscribers
        self.drone_sub = rospy.Subscriber(self.topic_name + "/pose", PoseStamped, self.pose_callback)
        self.force_on_drone         = rospy.Publisher(self.topic_name + "/force", Float32, queue_size=10)
        self.velocity_on_drone      = rospy.Publisher(self.topic_name + "/velocity", Float32, queue_size=10)
        self.acceleration_on_drone  = rospy.Publisher(self.topic_name + "/acceleration", Float32, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def pose_callback(self, msg):
        # Get the current position
        self.current_time = time.time()
        self.current_position[0] = copy.deepcopy(msg.pose.position.x)
        self.current_position[1] = copy.deepcopy(msg.pose.position.y)
        self.current_position[2] = copy.deepcopy(msg.pose.position.z)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))
        
    def _mainloop(self):

        r = rospy.Rate(self.rate)

        previous_position   = np.zeros(3)
        previous_velocity   = np.zeros(3)
        previous_time       = 0

        while not self._quit:
            
            # Compute change in position
            dx = self.current_position[0] - previous_position[0]
            dy = self.current_position[1] - previous_position[1]
            dz = self.current_position[2] - previous_position[2]

            # Compute change in time
            dt = self.current_time - previous_time
            
            # Compute velocity in the x,y,z direction
            if dt != 0:
                v_x = dx / dt 
                v_y = dy / dt 
                v_z = dz / dt 
            else:
                v_x = 0
                v_y = 0
                v_z = 0

            # Compute overall velcity
            current_velocity = math.sqrt(math.pow(v_x, 2) + math.pow(v_y, 2) + math.pow(v_z, 2))

            # Compute change in velocity
            dv_x = v_x - previous_velocity[0]
            dv_y = v_y - previous_velocity[1]
            dv_z = v_z - previous_velocity[2]

            # Compute the acceleration
            if dt != 0:
                a_x = dv_x / dt
                a_y = dv_y / dt
                a_z = dv_z / dt
            else:
                a_x = 0
                a_y = 0
                a_z = 0

            # Compute the acceleration on the drone
            acceleration = math.sqrt(math.pow(a_x, 2) + math.pow(a_y, 2) + math.pow(a_z, 2))

            # Compute the force
            f_x = self.mass * a_x
            f_y = self.mass * a_y
            f_z = self.mass * a_z

            # Compute the force on the drone
            force = math.sqrt(math.pow(f_x, 2) + math.pow(f_y, 2) + math.pow(f_z, 2))

            # Publish the froce on the drone
            self.velocity_on_drone.publish(Float32(current_velocity))
            self.acceleration_on_drone.publish(Float32(acceleration))
            self.force_on_drone.publish(Float32(force))

            # Set the previous time and data
            previous_time       = copy.deepcopy(self.current_time)
            previous_position   = copy.deepcopy(self.current_position)
            previous_velocity   = np.array([v_x, v_y, v_z])

            # Mantain the rate
            r.sleep()

if __name__ == "__main__":
    # Run the node
    try:
        x = ForceCalculator()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)




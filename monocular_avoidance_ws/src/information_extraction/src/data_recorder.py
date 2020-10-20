#!/usr/bin/env python3
from __future__ import print_function, absolute_import

import rospy
import sys
import time
import copy
import math
import csv

import numpy as np
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Float32, Bool


class DataRecorder:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('DataRecorderNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the different metrics we want to capture and record
        self.current_time                   = 0
        self.drone1_position                = np.zeros(3)
        self.drone1_orientation             = np.zeros(3)
        self.drone1_velocity                = 0
        self.drone1_acceleration            = 0
        self.drone1_force                   = 0
        self.drone1_collision               = False
        self.drone2_position                = np.zeros(3)
        self.drone2_collision               = False
        self.person_position                = np.zeros(3)
        self.drone1_drone2_distance         = 0
        self.drone1_drone2_misallignment    = np.zeros(3)
        self.drone1_person_distance         = 0
        self.drone1_person_misallignment    = np.zeros(3)

        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Get the parameters for saving the file
        self.save_name = rospy.get_param(rospy.get_name() + '/save_name', "output.csv")

        # Print out the parameters
        self._log("Save name: " + str(self.save_name))

        # Init the program state
        self._quit = False

        # Init all the subscribers
        self.drone1_position_sub                = rospy.Subscriber("ground_truth/uav1/pose"                 , PoseStamped   , self.drone1_position_callback)
        self.velocity_sub                       = rospy.Subscriber("ground_truth/uav1/velocity"             , Float32       , self.drone1_velocity_callback)
        self.acceleration_sub                   = rospy.Subscriber("ground_truth/uav1/acceleration"         , Float32       , self.drone1_acceleration_callback)
        self.force_sub                          = rospy.Subscriber("ground_truth/uav1/force"                , Float32       , self.drone1_force_callback)
        self.drone1_collision_sub               = rospy.Subscriber("ground_truth/uav1/collision"            , Bool          , self.drone1_collision_callback)
        self.drone2_position_sub                = rospy.Subscriber("ground_truth/uav2/pose"                 , PoseStamped   , self.drone2_position_callback)
        self.drone2_collision_sub               = rospy.Subscriber("ground_truth/uav2/collision"            , Bool          , self.drone2_collision_callback)
        self.person_position_sub                = rospy.Subscriber("ground_truth/person/pose"               , PoseStamped   , self.person_position_callback)
        self.drone1_drone2_distance_sub         = rospy.Subscriber("ground_truth/uav1uav2/distance"         , Float32       , self.drone1_drone2_distance_callback)
        self.drone1_drone2_misallignment_sub    = rospy.Subscriber("ground_truth/uav1uav2/misalignment"     , Vector3       , self.drone1_drone2_misallignment_callback)
        self.drone1_person_distance_sub         = rospy.Subscriber("ground_truth/uav1person/distance"       , Float32       , self.drone1_person_distance_callback)
        self.drone1_person_misallignment_sub    = rospy.Subscriber("ground_truth/uav1person/misalignment"   , Vector3       , self.drone1_person_misallignment_callback)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def drone1_position_callback(self, msg):
        self.drone1_position[0] = copy.deepcopy(msg.pose.position.x)
        self.drone1_position[1] = copy.deepcopy(msg.pose.position.y)
        self.drone1_position[2] = copy.deepcopy(msg.pose.position.z)
        rot = msg.transform.rotation
        euler_rot = euler_from_quaternion(quaternion=(rot.x, rot.y, rot.z, rot.w))
        self.drone1_orientation[0] = copy.deepcopy(euler_rot[0])
        self.drone1_orientation[1] = copy.deepcopy(euler_rot[1])
        self.drone1_orientation[2] = copy.deepcopy(euler_rot[2])

    def drone1_velocity_callback(self, msg):
        self.drone1_velocity = copy.deepcopy(msg.data)

    def drone1_acceleration_callback(self, msg):
        self.drone1_acceleration = copy.deepcopy(msg.data)

    def drone1_force_callback(self, msg):
        self.drone1_force = copy.deepcopy(msg.data)

    def drone1_collision_callback(self, msg):
        self.drone1_collision = copy.deepcopy(msg.data)

    def drone2_position_callback(self, msg):
        self.drone2_position[0] = copy.deepcopy(msg.pose.position.x)
        self.drone2_position[1] = copy.deepcopy(msg.pose.position.y)
        self.drone2_position[2] = copy.deepcopy(msg.pose.position.z)

    def drone2_collision_callback(self, msg):
        self.drone2_collision = copy.deepcopy(msg.data)    

    def person_position_callback(self, msg):
        self.person_position[0] = copy.deepcopy(msg.pose.position.x)
        self.person_position[1] = copy.deepcopy(msg.pose.position.y)
        self.person_position[2] = copy.deepcopy(msg.pose.position.z)

    def drone1_drone2_distance_callback(self, msg):
        self.drone1_drone2_distance = copy.deepcopy(msg.data)

    def drone1_drone2_misallignment_callback(self, msg):
        self.drone1_drone2_misallignment[0] = copy.deepcopy(msg.x)
        self.drone1_drone2_misallignment[1] = copy.deepcopy(msg.y)
        self.drone1_drone2_misallignment[2] = copy.deepcopy(msg.z)

    def drone1_person_distance_callback(self, msg):
        self.drone1_person_distance = copy.deepcopy(msg.data)

    def drone1_person_misallignment_callback(self, msg):
        self.drone1_person_misallignment[0] = copy.deepcopy(msg.x)
        self.drone1_person_misallignment[1] = copy.deepcopy(msg.y)
        self.drone1_person_misallignment[2] = copy.deepcopy(msg.z)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))
        
    def _mainloop(self):

        r = rospy.Rate(self.rate)

                self.current_time = time.time()

        if self.save_name != "" and len(self.save_name) > 0:
            # Open the csv file for writing
            csvfile = open(self.save_name, 'w') 
            # Create the header
            fieldnames = ["timestamp", "drone1_pos_x", "drone1_pos_y", "drone1_pos_z", "drone1_ori_x", "drone1_ori_y", "drone1_ori_z", "drone1_velocity", "drone1_acceleration", "drone1_force", "drone1_collision", "drone2_pos_x", "drone2_pos_y", "drone2_pos_z", "drone2_collision", "person_pos_x", "person_pos_y", "person_pos_z", "uav1uav2_distance", "uav1uav2_misallignment_x", "uav1uav2_misallignment_y", "uav1uav2_misallignment_z", "drone1person_distance", "drone1person_misallignment_x", "drone1person_misallignment_y", "drone1person_misallignment_z"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

        while not self._quit:
            # Write the data to the csv file
            data = {}
            data['timestamp']                       = time.time()
            data['drone1_pos_x']                    = self.drone1_position[0]
            data['drone1_pos_y']                    = self.drone1_position[1]
            data['drone1_pos_z']                    = self.drone1_position[2]
            data['drone1_ori_x']                    = self.drone1_orientation[0]
            data['drone1_ori_y']                    = self.drone1_orientation[1]
            data['drone1_ori_z']                    = self.drone1_orientation[2]
            data['drone1_velocity']                 = self.drone1_velocity 
            data['drone1_acceleration']             = self.drone1_acceleration 
            data['drone1_force']                    = self.drone1_force 
            data['drone1_collision']                = self.drone1_collision 
            data['drone2_pos_x']                    = self.drone1_velocity[0]
            data['drone2_pos_y']                    = self.drone1_velocity[1]
            data['drone2_pos_z']                    = self.drone1_velocity[2]
            data['drone2_collision']                = self.drone2_collision
            data['person_pos_x']                    = self.person_position[0]
            data['person_pos_y']                    = self.person_position[1]
            data['person_pos_z']                    = self.person_position[2]
            data['uav1uav2_distance']               = self.drone1_drone2_distance 
            data['uav1uav2_misallignment_x']        = self.drone1_drone2_misallignment[0] 
            data['uav1uav2_misallignment_y']        = self.drone1_drone2_misallignment[1]
            data['uav1uav2_misallignment_z']        = self.drone1_drone2_misallignment[2] 
            data['drone1person_distance']           = self.drone1_person_distance 
            data['drone1person_misallignment_x']    = self.drone1_person_misallignment[0]
            data['drone1person_misallignment_y']    = self.drone1_person_misallignment[1] 
            data['drone1person_misallignment_z']    = self.drone1_person_misallignment[2] 

            writer.writerow(data)

            # Mantain the rate
            r.sleep()

        # # Close the csv file
        if self.save_name != "":
            csvfile.close()

if __name__ == "__main__":
    # Run the node
    try:
        x = DataRecorder()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)




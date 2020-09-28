#!/usr/bin/env python3
from __future__ import print_function, absolute_import

import rospy
import sys
import time
import re
import yaml
import math
import os

from geometry_msgs.msg import PoseStamped
from subprocess import PIPE, Popen
from threading  import Thread
import numpy as np

from queue import Queue, Empty
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from drone_controller.msg import Attitude


class InformationExtraction:

    def __init__(self, attitude_source, position_source):
        # Init the ROS node
        rospy.init_node('InformationExtractionNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the drone positions
        self.drone1 = PoseStamped()
        self.drone2 = PoseStamped()

        # Set the rate
        self.rate = 25.0
        self.dt = 1.0 / self.rate

        # Init the program state
        self._quit = False

        # Init all the publishers and subscribers
        self.drone1_pub = rospy.Publisher("ground_truth/uav1/pose", PoseStamped, queue_size=10)
        self.drone2_pub = rospy.Publisher("ground_truth/uav2/pose", PoseStamped, queue_size=10)

        self.att_sub = None
        self.pos_sub = None

        # None Vicon related topics
        if (attitude_source != "") and ("vicon" not in attitude_source):
            self.att_sub = rospy.Subscriber(attitude_source, Attitude , self.attitude_callback)
        if (position_source != "") and ("vicon" not in position_source):
            self.pos_sub = rospy.Subscriber(position_source, Point , self.position_callback)

        # Vicon related topics
        self.vicon_attitude = ("vicon" in attitude_source.lower())
        self.vicon_position = ("vicon" in position_source.lower())
        if self.vicon_attitude or self.vicon_position:
            self.vicon_sub = rospy.Subscriber("/vicon/ANAFI/ANAFI", TransformStamped , self.vicon_callback)

        time.sleep(0.5)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def vicon_callback(self, msg):
        if self.vicon_attitude:
            euler = euler_from_quaternion(quaternion=(msg.transform.rotation.x,
                                                      msg.transform.rotation.y,
                                                      msg.transform.rotation.z,
                                                      msg.transform.rotation.w))
            att1[0] = float(euler[0])
            att1[1] = float(euler[1])
            att1[2] = float(euler[2])
        if self.vicon_position:
            pos1[0] = float(msg.transform.translation.x)
            pos1[1] = float(msg.transform.translation.y)
            pos1[2] = float(msg.transform.translation.z)  

    def attitude_callback(self, msg):
        att1[0] = float(msg.roll)
        att1[1] = float(msg.pitch)
        att1[2] = float(-1 * msg.yaw + math.radians(90))

    def position_callback(self, msg):
        pos1[0] = float(msg.x)
        pos1[1] = float(msg.y)
        pos1[2] = float(msg.z)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))
        
    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit:
            try:  
                line = q.get_nowait()
            except Empty:
                # Clear out the queue
                q.queue.clear()

            # Drone 1
            self.drone1.header.stamp = rospy.Time.now()
            self.drone1.header.frame_id = "/map"
            self.drone1.pose.position.x = pos1[0]
            self.drone1.pose.position.y = pos1[1]
            self.drone1.pose.position.z = pos1[2]
            # Get drones attitude
            (x, y, z, w) = quaternion_from_euler(att1[0], att1[1], att1[2])
            self.drone1.pose.orientation.x = x
            self.drone1.pose.orientation.y = y
            self.drone1.pose.orientation.z = z
            self.drone1.pose.orientation.w = w

            # Drone 2
            self.drone2.header.stamp = rospy.Time.now()
            self.drone2.header.frame_id = "/map"
            self.drone2.pose.position.x = pos2[0]
            self.drone2.pose.position.y = pos2[1]
            self.drone2.pose.position.z = pos2[2]
            # Get drones attitude
            (x, y, z, w) = quaternion_from_euler(att2[0], att2[1], att2[2])
            self.drone2.pose.orientation.x = x
            self.drone2.pose.orientation.y = y
            self.drone2.pose.orientation.z = z
            self.drone2.pose.orientation.w = w

            # publish the topics
            self.drone1_pub.publish(self.drone1)
            self.drone2_pub.publish(self.drone2)

            # Mantain the rate
            r.sleep()


# Process the output from the file
def process_output(out, queue, att_source, pos_source):
    for line in iter(out.readline, b''):
        line = str(line)
        if pos_source == "":
            if "_first.worldPosition" in line:
                number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
                if ".x" in line:
                    pos1[0] = float(number)
                if ".y" in line:
                    pos1[1] = float(number)
                if ".z" in line:
                    pos1[2] = float(number)
        if "_second.worldPosition" in line:
            number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
            if ".x" in line:
                pos2[0] = float(number)
            if ".y" in line:
                pos2[1] = float(number)
            if ".z" in line:
                pos2[2] = float(number)

        if att_source == "":
            if "_first.worldAttitude" in line:
                number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
                if ".x" in line:
                    att1[0] = float(number)
                if ".y" in line:
                    att1[1] = float(number)
                if ".z" in line:
                    att1[2] = float(number)
        if "_second.worldAttitude" in line:
            number = re.findall(r"[-+]?\d*\.\d+|\d+", line)[0]
            if ".x" in line:
                att2[0] = float(number)
            if ".y" in line:
                att2[1] = float(number)
            if ".z" in line:
                att2[2] = float(number)

        queue.put(line)
    out.close()


if __name__ == "__main__":
    pos1 = np.zeros(3)
    pos2 = np.zeros(3)
    att1 = np.zeros(3)
    att2 = np.zeros(3)
    q = Queue()

    # Get the source of the attitude from the config file
    home_dir = os.path.abspath(os.curdir)[:-4]
    with open(home_dir + '/Desktop/MixedRealityTesting/monocular_avoidance_ws/src/mixed_reality/config/config.yaml') as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    att_source = config["attitude_source"]
    pos_source = config["position_source"]

    # Run the command
    ON_POSIX = 'posix' in sys.builtin_module_names
    command = "tlm-data-logger -r 0 inet:127.0.0.1:9060"
    p = Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
    
    # Create a thread which dies with main program
    t = Thread(target=process_output, args=(p.stdout, q, att_source, pos_source))
    t.daemon = True 
    t.start()

    # Run the node
    try:
        x = InformationExtraction(att_source, pos_source)
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)


#!/usr/bin/env python3
# TODO Set the IP as a param
# TODO set the waypoint as a subscribe
from __future__ import print_function, absolute_import
from geometry_msgs.msg import PoseStamped
from subprocess import PIPE, Popen
from threading  import Thread
import rospy
import sys
import time
import numpy as np
import re
from queue import Queue, Empty
from tf.transformations import quaternion_from_euler


class InformationExtraction:

    def __init__(self):
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
        time.sleep(0.5)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

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
def process_output(out, queue):
    for line in iter(out.readline, b''):
        line = str(line)
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

    # Run the command
    ON_POSIX = 'posix' in sys.builtin_module_names
    command = "tlm-data-logger -r 0 inet:127.0.0.1:9060"
    p = Popen(command, stdout=PIPE, bufsize=1, close_fds=ON_POSIX, shell=True)
    
    # Create a thread which dies with main program
    t = Thread(target=process_output, args=(p.stdout, q))
    t.daemon = True 
    t.start()

    # Run the node
    try:
        x = InformationExtraction()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)


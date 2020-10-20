#!/usr/bin/env python3
import rospy
import zmq
import json
import cv2
import math
import time

import numpy as np

from std_msgs.msg import Bool 
from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

class UnityRosConnection():

    def __init__(self):

        # Run the shutdown sequence on shutdown
        rospy.on_shutdown(self.shutdown_sequence)

        # Getting the PID parameters
        port = rospy.get_param(rospy.get_name() + '/port', '5555')

        # Determines whether we send or recieve drone positions
        self.update_unity     = rospy.get_param(rospy.get_name() + '/update_drone_position_in_unity' , True)

        # Determines whether or not we publish the returned values (and what to call them)
        self.pos_topic_name     = rospy.get_param(rospy.get_name() + '/position_topic_name'   , "/ground_truth/uav1/pose")
        self.img_topic_name     = rospy.get_param(rospy.get_name() + '/image_topic_name'      , "")
        self.person_topic_name  = rospy.get_param(rospy.get_name() + '/person_topic_name'     , "")
        self.col_topic_name     = rospy.get_param(rospy.get_name() + '/collision_topic_name'  , "")

        # Set the rate
        self.rate = 100.0
        self.dt = 1.0 / self.rate

        # Declare the drone position
        self.drone_pos = np.zeros(3)
        self.drone_ori = np.zeros(3)

        # Display incoming parameters
        rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
        rospy.loginfo(str(rospy.get_name()) + ": port - " + str(port))
        rospy.loginfo(str(rospy.get_name()) + ": Send drone position to Unity - " + str(self.update_unity))
        rospy.loginfo(str(rospy.get_name()) + ": Position topic name - " + str(self.pos_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": Image topic name - " + str(self.img_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": Person topic name - " + str(self.person_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": Collision topic name - " + str(self.col_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

        # Init all subs and pubs
        self.pos_sub        = None
        self.pos_pub        = None
        self.img_pub        = None
        self.person_pub     = None
        self.collision_pub  = None

        # If we want to send the data we need to subscribe to it
        if self.update_unity:
            self.pos_sub = rospy.Subscriber(self.pos_topic_name, PoseStamped , self.position_callback)
        # Else we want to publish the returned position
        else:
            self.pos_pub = rospy.Publisher(self.pos_topic_name, PoseStamped, queue_size=10)

        # Publish the image
        if self.img_topic_name != "":
            self.bridge = CvBridge()
            self.img_pub = rospy.Publisher(self.img_topic_name, Image, queue_size=10)

        # Publish the person
        if self.person_topic_name != "":
            self.person_pub = rospy.Publisher(self.person_topic_name, PoseStamped, queue_size=10)

        # Publish the collision
        if self.col_topic_name != "":
            self.collision_pub = rospy.Publisher(self.col_topic_name, Bool, queue_size=10)

        # Connect to the TCP connection
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:" + str(port))

        # Run the communication node
        self.MainLoop()


    # This is the main loop of this class
    def MainLoop(self):
        # Set the rate
        rate = rospy.Rate(self.rate)

        # Create a dummy return image
        return_image = [0.0]

        # While running
        while not rospy.is_shutdown():
            # Start the timer
            start = time.time()

            # Create the TCP message witht he current drone position and orientation
            msg = {
                "collision": False,

                "drone_pose_x": self.drone_pos[0],
                "drone_pose_y": self.drone_pos[1],
                "drone_pose_z": self.drone_pos[2],
                "drone_orientation_x": self.drone_ori[0],
                "drone_orientation_y": self.drone_ori[1],
                "drone_orientation_z": self.drone_ori[2],

                "person_pose_x": 0,
                "person_pose_y": 0,
                "person_pose_z": 0,
                "person_orientation_x": 0,
                "person_orientation_y": 0,
                "person_orientation_z": 0,

                "image": return_image
            }

            # Wait for a message from Unity
            message = self.socket.recv()
            data = json.loads(message)

            # If we want to publish the drones position
            if not self.update_unity:
                # Create the drones pose
                drone_msg = PoseStamped()
                drone_msg.pose.position.x = data["drone_pose_z"]
                drone_msg.pose.position.y = -1 * data["drone_pose_x"]
                drone_msg.pose.position.z = data["drone_pose_y"]
                euler_ang = (data["drone_orientation_x"], data["drone_orientation_y"], data["drone_orientation_z"])
                # quart = quaternion_from_euler(ai=euler_ang[0], aj=euler_ang[1], ak=euler_ang[2])
                euler_ang = (0, 0 ,0)
                drone_msg.pose.orientation.x = quart[0]
                drone_msg.pose.orientation.y = quart[1]
                drone_msg.pose.orientation.z = quart[2]
                drone_msg.pose.orientation.w = quart[3]

                # Publish the drone position
                self.pos_pub.publish(drone_msg)

            # If we want to get the image
            if self.img_topic_name != "":
                # Get the image
                img_bytes = np.array(data["image"], dtype=np.uint8)
                if img_bytes.size > 1:
                    img_np = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
                    # Convert he image to a msg
                    try:
                        image_message = self.bridge.cv2_to_imgmsg(img_np, "bgr8")
                    except CvBridgeError as e:
                        pass
                    # Publish the new image
                    self.img_pub.publish(image_message)

            # If we want to publish collision information
            if self.col_topic_name != "":
                self.collision_pub.publish(Bool(data["collision"]))

            # If we want to publish the person information
            if self.person_topic_name != "":
                # Create the person pose
                person_msg = PoseStamped()
                person_msg.pose.position.x = data["person_pose_z"]
                person_msg.pose.position.y = -1 * data["person_pose_x"]
                person_msg.pose.position.z = data["person_pose_y"]
                # euler_ang = (data["person_orientation_y"], -1 * data["person_orientation_z"], -1 * data["person_orientation_x"])
                euler_ang = (0, 0 ,0)
                quart = quaternion_from_euler(ai=euler_ang[0], aj=euler_ang[1], ak=euler_ang[2])
                person_msg.pose.orientation.x = quart[0]
                person_msg.pose.orientation.y = quart[1]
                person_msg.pose.orientation.z = quart[2]
                person_msg.pose.orientation.w = quart[3]

                # Publish the person position
                self.person_pub.publish(person_msg)

            # Reply to unity with drone new position
            msg_json = json.dumps(msg)
            msg_json = msg_json.encode('utf8') 
            self.socket.send(msg_json)

            # Sleep away any remaining time
            rate.sleep()

    # Save the new attitude readings
    def position_callback(self, msg):
        self.drone_pos[0] = -1 * msg.pose.position.y
        self.drone_pos[1] = msg.pose.position.z
        self.drone_pos[2] = msg.pose.position.x
        rot = msg.pose.orientation
        euler = euler_from_quaternion(quaternion=(rot.x, rot.y, rot.z, rot.w))
        self.drone_ori[0] = math.degrees(euler[1])
        self.drone_ori[1] = math.degrees(-1 * euler[2])
        self.drone_ori[2] = math.degrees(-1 * euler[0])

    # Called on ROS shutdown
    def shutdown_sequence(self):
        rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")

if __name__ == '__main__':
    rospy.init_node('UnityRosCommunication')
    try:
        coms = UnityRosConnection()
    except rospy.ROSInterruptException:
        pass
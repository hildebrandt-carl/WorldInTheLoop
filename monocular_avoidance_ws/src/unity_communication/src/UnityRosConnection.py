#!/usr/bin/env python3
import rospy
import zmq
import json
import cv2
import math
import time

import matplotlib
import matplotlib.pyplot as plt

import numpy as np

from geometry_msgs.msg import PoseStamped 
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError

class UnityRosConnection():

    def __init__(self):

        # Run the shutdown sequence on shutdown
        rospy.on_shutdown(self.shutdown_sequence)

        # Getting the PID parameters
        port = rospy.get_param(rospy.get_name() + '/port', '5555')
        self.pos_topic_name = rospy.get_param(rospy.get_name() + '/position_topic_name', "/ground_truth/uav1/pose")
        self.img_topic_name = rospy.get_param(rospy.get_name() + '/image_topic_name', "")

        # Set the rate
        self.rate = 100.0
        self.dt = 1.0 / self.rate

        # Decarle the drone position
        self.drone_pos = np.zeros(3)
        self.drone_ori = np.zeros(3)

        # Display incoming parameters
        rospy.loginfo(str(rospy.get_name()) + ": Launching with the following parameters:")
        rospy.loginfo(str(rospy.get_name()) + ": port - " + str(port))
        rospy.loginfo(str(rospy.get_name()) + ": position topic name - " + str(self.pos_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": image topic name - " + str(self.img_topic_name))
        rospy.loginfo(str(rospy.get_name()) + ": rate - " + str(self.rate))

        # Publishers and Subscribers
        self.pos_sub = rospy.Subscriber(self.pos_topic_name, PoseStamped , self.position_callback)
        if self.img_topic_name != "":
            self.img_pub = rospy.Publisher(self.img_topic_name, Image, queue_size=10)

        # Create the CV Bridge so we can publish images
        self.bridge = CvBridge()

        # Connect to the TCP connection
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind("tcp://*:" + str(port))

        # # Used to plot fps
        # self.y = []
        # self.average_fps = []

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

            # Create the TCP message
            msg = {
                "collision": False,
                "pose_x": self.drone_pos[0],
                "pose_y": self.drone_pos[1],
                "pose_z": self.drone_pos[2],
                "orientation_x": self.drone_ori[0],
                "orientation_y": self.drone_ori[1],
                "orientation_z": self.drone_ori[2],
                "image": return_image
            }

            # Wait for a message from Unity
            message = self.socket.recv()
            data = json.loads(message)
            # data = message
            img_bytes = np.array(data["image"], dtype=np.uint8)

            # Get the image
            if img_bytes.size > 1 and self.img_topic != "":
                img_np = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)

                # Convert he image to a msg
                try:
                    image_message = self.bridge.cv2_to_imgmsg(img_np, "bgr8")
                except CvBridgeError as e:
                    pass
            
                # Publish the new image
                self.img_pub.publish(image_message)

            # Reply to unity with drone new position
            msg_json = json.dumps(msg)
            msg_json = msg_json.encode('utf8') 
            self.socket.send(msg_json)

            time_taken = time.time() - start
            fps = 1.0 / time_taken
            # print("Current FPS: {}".format(fps))         
            # if len(self.average_fps) > 10:
            #     self.average_fps[:-1]

            # self.average_fps.append(fps)
            # self.y.append(np.average(self.average_fps))

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
        self.drone_ori[1] = math.degrees(euler[2])
        self.drone_ori[2] = math.degrees(-1 * euler[0])

    # Called on ROS shutdown
    def shutdown_sequence(self):
        rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")
        # fig = plt.figure()
        # ax = plt.subplot(111)
        # ax.plot(self.y)
        # plt.title('Unity Editor - Camera Size(2048, 1536)')
        # # plt.title('External - Camera Size(2048, 1536)')
        # plt.xlabel('TCP Message')
        # plt.ylabel('Average FPS (Window Size 10)')
        # fig.savefig('/home/carl/Documents/results/example2.png')


if __name__ == '__main__':
    rospy.init_node('UnityRosCommunication')
    try:
        coms = UnityRosConnection()
    except rospy.ROSInterruptException:
        pass
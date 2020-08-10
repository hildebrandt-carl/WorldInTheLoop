#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from random import random
from time import sleep
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from drone_controller.msg import Move
from utility import DroneState

import threading
import pygame
import sys
import rospy
import cv2
import time
from cv_bridge import CvBridge
import numpy as np

from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingState import AltitudeChanged
import olympe


class MainDroneController:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.1"
    LAND_TAKEOFF_TIME = 4.0

    def __init__(self, speed=65, refresh_move=0.2):
        # Init the ROS node
        rospy.init_node('MainDroneControllerNode')
        self._log("Node Initialized")

        # State we only want warnings
        olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 10.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._state = DroneState.INACTIVE

        # Set the move goal
        self._movegoal = np.zeros(4, dtype=int)

        # Get the IP address
        self.simulated_param = rospy.get_param(rospy.get_name() + '/simulated_ip', True)

        if self.simulated_param == True:
            publish_topic_base = "/uav1/sphinx_sensors/"
        else:
            publish_topic_base = "/uav1/physical_sensors/"

        # Init all the publishers and subscribers
        self.state_sub = rospy.Subscriber("/uav1/input/state", Int16, self._setstate)
        self.move_sub = rospy.Subscriber("/uav1/input/move", Move, self._setmove)
        self.camera_pub = rospy.Publisher(publish_topic_base + "camera", Image, queue_size=10)
        self.altitude_pub = rospy.Publisher(publish_topic_base + "altitude", Float64, queue_size=10)

        # Init the drone variables
        self.drone_speed = min([speed, 100])
        self.drone_mtime = min([refresh_move, 1]) # move time
        if self.simulated_param == True:
            self.drone = olympe.Drone(MainDroneController.SIMULATED_IP)
        else:
            self.drone = olympe.Drone(MainDroneController.PHYSICAL_IP)

        # Create the bridge used to send the ROS message
        self.bridge = CvBridge()

        # Connect to the drone
        self.drone.connection()
        time.sleep(0.5)
        # Setup the callback function to get the live video
        self.drone.set_streaming_callbacks(raw_cb=self.yuv_frame_cb)
        # Start video streaming
        self.drone.start_video_streaming()
        # Start piloting
        self.drone.start_piloting()


    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _setstate(self, msg):
        self._state = DroneState(int(msg.data))

    def _setmove(self, msg):
        accepted = False
        if -100 <= msg.left_right <= 100 and -100 <= msg.front_back <= 100 and -100 <= msg.yawl_yawr <= 100 and -100 <= msg.up_down <= 100:
            accepted = True
        if accepted:
            self._movegoal[0] = int(msg.left_right)
            self._movegoal[1] = int(msg.front_back)
            self._movegoal[2] = int(msg.yawl_yawr)
            self._movegoal[3] = int(msg.up_down)

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _close_conn(self):
        self.drone.stop_video_streaming()
        self.drone.stop_piloting()
        self.drone.disconnection()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful information
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # Send the image on ROS
        image_message = self.bridge.cv2_to_imgmsg(cv2frame, encoding="bgr8")
        self.camera_pub.publish(image_message)


    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)

        while not self._quit:
            # Inactive state
            if self._state == DroneState.INACTIVE:
                if previous_state != self._state or previous_state is None:
                    self._log("Drone Inactive")
                    previous_state = self._state

            # Takeoff state
            elif self._state == DroneState.TAKEOFF:
                if previous_state != self._state:
                    self._log("Takeoff Initiated")
                    self._takeoff()
                    previous_state = self._state

            # Landing state
            elif self._state == DroneState.LANDING:
                if previous_state != self._state:
                    self._log("Landing Initiated")
                    self._land()
                    previous_state = self._state

            # Hovering state
            elif self._state == DroneState.HOVERING:
                if previous_state != self._state:
                    self._log("Hovering Initiated")
                    previous_state = self._state
                self._move(0, 0, 0, 0)
                
            # Avoding state
            elif self._state == DroneState.AVOIDING:
                if previous_state != self._state:
                    self._log("Obstacle Avoidance Initiated")
                    previous_state = self._state
                self._move(self._movegoal[0], self._movegoal[1], self._movegoal[2], self._movegoal[3])

            # Navigation state
            elif self._state == DroneState.GATENAVIGATION:
                if previous_state != self._state:
                    self._log("Gate Navigation Initiated")
                    previous_state = self._state
                self._move(self._movegoal[0], self._movegoal[1], self._movegoal[2], self._movegoal[3])

            # Going towards stairs
            elif self._state == DroneState.STAIRNAVIGATION:
                if previous_state != self._state:
                    self._log("Stair Navigation Initiated")
                    previous_state = self._state
                self._move(self._movegoal[0], self._movegoal[1], self._movegoal[2], self._movegoal[3])
                
            # Going yaw test mode
            elif self._state == DroneState.YAWNAVIGATION:
                if previous_state != self._state:
                    self._log("Stair Navigation Initiated")
                    previous_state = self._state
                self._move(self._movegoal[0], self._movegoal[1], self._movegoal[2], self._movegoal[3])

            # Going person following mode
            elif self._state == DroneState.FOLLOWPERSON:
                if previous_state != self._state:
                    self._log("Fallow Person Mode Initiated")
                    previous_state = self._state
                self._move(self._movegoal[0], self._movegoal[1], self._movegoal[2], self._movegoal[3])
            # Mantain the rate
            r.sleep()
                  
        # Make sure we have initiated landing
        self._land()
        self._close_conn()  # closes the connection

    def _takeoff(self):
        self.drone(
            TakeOff(_no_expect=True)
            & FlyingStateChanged(state="hovering", _timeout=5, _policy="check_wait")
        ).wait()

    def _land(self):
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        ).wait()

        front_back = 0

    def _move(self, left_right_in, front_back_in, yawl_yawr_in, up_down_in):
        
        # movements must be in [-100:100]
        # negative does the first variable name 
        # positive does the second variable name
        left_right = left_right_in
        front_back = front_back_in
        yawl_yawr  = yawl_yawr_in
        up_down    = up_down_in

        self.drone.piloting_pcmd(
            left_right, -front_back, yawl_yawr, -up_down,
            self.drone_mtime
        )

        # Get the current altitude
        altitude_dict = self.drone.get_state(AltitudeChanged)
        alt = Float64()
        alt.data = altitude_dict['altitude']
        self.altitude_pub.publish(alt)


if __name__ == "__main__":
    try:
        x = MainDroneController()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

#!/usr/bin/env python3
# TODO Set the IP as a param
# TODO set the waypoint as a subscribe
from __future__ import print_function, absolute_import
from std_msgs.msg import Int16
import rospy
import time
import olympe 
import sys
import copy
import numpy as np

from geometry_msgs.msg import PoseStamped
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from utility import DroneState


class SecondDroneController:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.1.1"
    LAND_TAKEOFF_TIME = 4.0

    def __init__(self, drone=None):
        # Init the ROS node
        rospy.init_node('SecondDroneControllerNode')
        self._log("Node Initialized")

        # State we only want warnings
        olympe.log.update_config({"loggers": {"olympe": {"level": "WARNING"}}})

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Used to save the drone positions
        self.drone_position = np.zeros(3)

        # Set the rate
        self.rate = 5.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False
        self._state = DroneState.INACTIVE

        # Get the IP address and speed param
        self.simulated_param    = rospy.get_param(rospy.get_name() + '/simulated_ip', True)
        self.fast_param         = rospy.get_param(rospy.get_name() + '/fast', False)

        # Init all the publishers and subscribers
        self.state_sub = rospy.Subscriber("/uav2/input/state", Int16, self._setstate)
        self.drone2_sub = rospy.Subscriber("/ground_truth/uav2/pose", PoseStamped, self.pose_callback)

        # Connect to the drone
        if self.simulated_param == True:
            self.drone = olympe.Drone(SecondDroneController.SIMULATED_IP)
        else:
            print(str(rospy.get_name()) + ": Note you are launching the actual drone!")
            self.drone = olympe.Drone(SecondDroneController.PHYSICAL_IP)
        
        self.drone.connection()
        time.sleep(0.5)
        self.drone.start_piloting()
        time.sleep(0.5)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def pose_callback(self, msg):
        self.drone_position[0] = copy.deepcopy(msg.pose.position.x)
        self.drone_position[1] = copy.deepcopy(msg.pose.position.y)
        self.drone_position[2] = copy.deepcopy(msg.pose.position.z)

    def _setstate(self, msg):
        self._state = DroneState(int(msg.data))

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _close_conn(self):
        self.drone.stop_piloting()
        self.drone.disconnection()

    def _mainloop(self):

        previous_state = None
        r = rospy.Rate(self.rate)
        start_time = 0
        duration = 0
        if (not self.fast_param):
            duration = 6
        else:
            duration = 3

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
            elif self._state == DroneState.WAYPOINT:
                if previous_state != self._state:
                    self._log("Waypoint Sent")
                    previous_state = self._state
                    start_time = time.time()
                if (time.time() - start_time < duration):
                    self._fly(stop=False)
                else:
                    self._fly(stop=True)

            # Mantain the rate
            r.sleep()
                
        # Make sure we have initiated landing
        self._land()
        # Closes the connection
        self._close_conn()  

    def _takeoff(self):
        self.drone(TakeOff()).wait()

    def _land(self):
        self.drone(Landing()).wait()

    def _fly(self, stop=False):
        
        # Dont move
        front_back = 0
        left_right = 0
        yawl_yawr = 0
        up_down = 0

        # Only allow movement if the drone's x position is greater than 1
        if self.drone_position[0] > -0.5:
            if self.fast_param:
                front_back = 20
            else:
                front_back = 10

        # If stop we dont want to move
        if stop:
            front_back = 0

        # Send command
        self.drone.piloting_pcmd(
            left_right, -front_back, yawl_yawr, -up_down, 0.2
        )
        

if __name__ == "__main__":
    try:
        x = SecondDroneController()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

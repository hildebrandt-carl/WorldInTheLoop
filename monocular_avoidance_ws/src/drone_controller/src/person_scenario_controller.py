#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import

from std_msgs.msg import Int16
from std_msgs.msg import Bool
from utility import ProgramState
from utility import DroneState

import sys
import rospy
import time


class ProgramController:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('ProgramControllerNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Keep track of the program state
        self._state = ProgramState.INACTIVE
        self._quit = False

        # Set the rate
        self.rate = 1.0
        self.dt = 1.0 / self.rate

        # Holds the manual override value
        self.manual_override = False

        # Init all the publishers and subscribers
        self.main_drone_pub     = rospy.Publisher("uav1/input/state", Int16, queue_size=10)
        self.manual_move_sub    = rospy.Subscriber("/uav1/input/manual_overide", Bool, self._setManualOverride)

    # Manual override callback
    def _setManualOverride(self, msg):
        if self.manual_override == False and msg.data == True:
            self._log("Manual Override Requested")
            self.manual_override = True
            msg = Int16(DroneState.MANUAL.value)
            self.main_drone_pub.publish(msg)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)
        previous_state = None

        while not self._quit:
            if self._state == ProgramState.INACTIVE:
                if previous_state is None:
                    self._log("Scenario Inactive")
                    previous_state = self._state
                    self._state = ProgramState.STARTING

            elif self._state == ProgramState.STARTING:
                if previous_state != self._state:
                    self._log("Scenario Started")
                    previous_state = self._state
                    self._startscenario()
                    
            elif self._state == ProgramState.ENDING:
                if previous_state != self._state:
                    self._log("Scenario Ending")
                    previous_state = self._state
                    self._endscenario()
                   
            elif self._state == ProgramState.SCENARIO:
                if previous_state != self._state:
                    self._log("Main Scenario Initiated")
                    previous_state = self._state
                    self._mainscenario()

            # Mantain the rate
            r.sleep()

    def _startscenario(self):
        msg = Int16(DroneState.TAKEOFF.value)
        self.main_drone_pub.publish(msg)
        time.sleep(0.2)
        self._state = ProgramState.SCENARIO

    def _endscenario(self):
        msg = Int16(DroneState.LANDING.value)
        self.main_drone_pub.publish(msg)
        time.sleep(5)
        self._state = ProgramState.INACTIVE

    def _mainscenario(self):
        msg = Int16(DroneState.HOVERING.value)
        self.main_drone_pub.publish(msg)
        time.sleep(0.1)
        msg = Int16(DroneState.FOLLOWPERSON.value)
        self.main_drone_pub.publish(msg)
        time.sleep(60)
        self._state = ProgramState.ENDING

if __name__ == "__main__":
    try:
        x = ProgramController()
        time.sleep(5)
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

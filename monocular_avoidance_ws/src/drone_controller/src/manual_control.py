#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from drone_controller.msg import Move
from utility import DroneState

import sys
import rospy


class ManualControl:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('ManualControlNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # used to control the main loop
        self._quit = False

        # Used to control the drone
        self.takeoff = 0
        self.land = 0
        self.move = Move()

        # Used to keep track of the drone state
        self.drone_state = DroneState.INACTIVE

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Init all the publishers and subscribers
        self.move_pub = rospy.Publisher("uav1/input/move", Move, queue_size=10)
        self.drone_state_pub = rospy.Publisher("uav1/input/state", Int16, queue_size=10)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self._getCommand)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getCommand(self, msg):
        self.takeoff            = msg.buttons[3]
        self.land               = msg.buttons[0]
        self.move.left_right    = int(round(-1 * msg.axes[0] * 100, 0))
        self.move.front_back    = int(round(-1 * msg.axes[1] * 100, 0))
        self.move.up_down       = int(round(-1 * msg.axes[4] * 100, 0))
        self.move.yawl_yawr     = int(round(-1 * msg.axes[3] * 100, 0))

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)
        takeoff_counter = 0

        while not self._quit:

            # If we have landed or haven't ever taken off
            if ((self.drone_state == DroneState.INACTIVE) or (self.drone_state == DroneState.LANDING)):
                if self.takeoff == 1:
                    self.drone_state = DroneState.TAKEOFF
                    self.drone_state_pub.publish(Int16(self.drone_state.value))

            # If we have taken off wait a while before allowing new commands
            if (self.drone_state == DroneState.TAKEOFF):
                takeoff_counter += 1
                if takeoff_counter > self.rate * 5:
                    self.drone_state = DroneState.MANUAL
                    self.drone_state_pub.publish(Int16(self.drone_state.value))
            else:
                takeoff_counter = 0

            # If we are in manual mode
            if (self.drone_state == DroneState.MANUAL):
                self.move_pub.publish(self.move)

            # If we are flying
            if ((self.drone_state != DroneState.INACTIVE) and (self.drone_state != DroneState.LANDING)):
                if self.land == 1:
                    self.drone_state = DroneState.LANDING
                    self.drone_state_pub.publish(Int16(self.drone_state.value))
            
            r.sleep()

if __name__ == "__main__":
    try:
        x = ManualControl()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)
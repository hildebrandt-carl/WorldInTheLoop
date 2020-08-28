#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from drone_controller.msg import Move
from std_msgs.msg import Int8

import sys
import copy
import rospy


class YawCorrection:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('YawCorrectionNode')
        self._log("Node Initialized")      

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the move goal
        self._yaw = 0

        # Param to use the corrrection or not
        self._use_yaw_correction = rospy.get_param(rospy.get_name() + '/yaw_correction', True)

        # Display the variables
        self._log(": Yaw Correction - " + str(self._use_yaw_correction))

        # Set the rate
        self.rate = 30.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False

        # Create the publishers and subscribers
        self.yaw_sub = rospy.Subscriber("/uav1/input/yawcorrection", Int8, self._setyaw)
        self.move_sub = rospy.Subscriber("/uav1/input/beforeyawcorrection/move", Move, self._setmove)
        self.move_pub = rospy.Publisher("uav1/input/move", Move, queue_size=10)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _setmove(self, msg):
        msg_out = Move()
        msg_out.left_right = int(round(msg.left_right, 0))
        msg_out.front_back = int(round(msg.front_back, 0))
        msg_out.up_down = int(round(msg.up_down, 0))

        if self._use_yaw_correction:
            msg_out.yawl_yawr = self._yaw
        else:
            msg_out.yawl_yawr = int(round(msg.yawl_yawr, 0))
            
        self.move_pub.publish(msg_out)

    def _setyaw(self, msg):
        self._yaw = copy.deepcopy(int(round(msg.data, 0)))

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit:            
            r.sleep()

if __name__ == "__main__":
    try:
        x = YawCorrection()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

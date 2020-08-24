#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from drone_controller.msg import Move
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

import sys
import copy


class YawCorrection:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('YawCorrectionNode')
        self._log("Node Initialized")      

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the move goal
        self._true_yaw = 0
        self._desired_yaw = 0

        # Param to use the corrrection or not
        self._use_yaw_correction = rospy.get_param(rospy.get_name() + '/yaw_correction', False)

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Init the drone and program state
        self._quit = False

        # Create the publishers and subscribers
        self.yaw_pub = rospy.Publisher("/uav1/input/yawcorrection", Int8, queue_size=10)
        self.true_yaw = rospy.Subscriber("/uav1/input/beforeyawcorrection/move", Move, self._getmove)
        self.desired_yaw = rospy.Subscriber("/vicon/anafi", TransformStamped, self._getvicon)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getmove(self, msg):
        self._desired_yaw = msg.yawl_yawr

    def _getvicon(self, msg):
        self._true_yaw = euler_from_quaternion(msg.transform.rotation)[2]

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        while not self._quit: 
            PID CONTROLLER           
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

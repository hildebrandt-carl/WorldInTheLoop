#!/usr/bin/env python3
# TODO Set the IP as a param
from __future__ import print_function, absolute_import
from drone_controller.msg import Move
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from pid_class import PID 
from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Bool

import sys
import copy
import rospy
import math


class ViconYawControl:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('ViconYawControlnNode')
        self._log("Node Initialized")      

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the move goal
        self._true_yaw = 0
        self._desired_yaw = None
        self._desired_yaw_wait_count = 1

        # Set the rate
        self.rate = 15.0
        self.dt = 1.0 / self.rate

        # Out of range param
        self.out_of_range_value = rospy.get_param(rospy.get_name() + '/out_of_range_yaw', 30)

        # Get the PID
        gains = rospy.get_param(rospy.get_name() + '/gains', {'p': 10.0, 'i': 0.0, 'd': 0.0})
        Kp, Ki, Kd = gains['p'], gains['i'], gains['d']
        self.yaw_controller = PID(Kp, Ki, Kd, self.rate)

        # Display the variables
        self._log(": p - " + str(Kp))
        self._log(": i - " + str(Ki))
        self._log(": d - " + str(Kd))
        self._log(": Out of range - " + str(self.out_of_range_value))

        # Init the drone and program state
        self._quit = False

        # Create the publishers and subscribers
        self.yaw_pub            = rospy.Publisher("/uav1/input/yawcorrection", Int8, queue_size=10)
        self.debug_desired_pub  = rospy.Publisher("/debug/DesiredYaw", Float32 , queue_size=10)
        self.debug_actual_pub   = rospy.Publisher("/debug/ActualYaw", Float32 , queue_size=10)
        self.out_of_range_pub   = rospy.Publisher("/uav1/status/yaw_out_of_range", Bool , queue_size=10)
        self.true_yaw_sub       = rospy.Subscriber("/vicon/ANAFI/ANAFI", TransformStamped, self._getvicon)

    def start(self):
        self._mainloop()

    def stop(self):
        self._quit = True

    def _getvicon(self, msg):
        rot = msg.transform.rotation
        self._true_yaw = euler_from_quaternion(quaternion=(rot.x, rot.y, rot.z, rot.w))[2]

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):

        r = rospy.Rate(self.rate)

        count = 0

        while not self._quit: 
            # get the initial position of the drone
            if self._desired_yaw is None:
                if count < self._desired_yaw_wait_count:
                    count += 1
                else:
                    self._desired_yaw = self._true_yaw
            # We have our initial position
            else:

                # Debug the true and actual yaw
                desired_yaw_msg = Float32()
                desired_yaw_msg.data = self._desired_yaw
                self.debug_desired_pub.publish(desired_yaw_msg)
                actual_yaw_msg = Float32()
                actual_yaw_msg.data = self._true_yaw
                self.debug_actual_pub.publish(actual_yaw_msg)

                # Check if we are out of range
                range_msg = Bool()
                range_msg.data = False
                if abs(self._desired_yaw - self._true_yaw) < math.radians(self.out_of_range_value):
                    range_msg.data = True
                self.out_of_range_pub.publish(range_msg)
                    
                # Compute the output
                output = self.yaw_controller.get_output(self._desired_yaw, self._true_yaw)
                output = -1 * output

                msg = Int8()
                msg.data = int(min(max(round(output, 0), -100), 100))

                self.yaw_pub.publish(msg)
            r.sleep()

if __name__ == "__main__":
    try:
        x = ViconYawControl()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

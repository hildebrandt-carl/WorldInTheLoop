#!/usr/bin/env python3

from sensor_msgs.msg import Image

import rospy
import yaml
import sys


class Mixer:

    def __init__(self):
        # Init the ROS node
        rospy.init_node('MixerNode')
        self._log("Node Initialized")

        # On shutdown do the following 
        rospy.on_shutdown(self.stop)

        # Set the rate
        self.rate = 30.0
        self.dt = 1.0 / self.rate

        # Init the variables
        self._quit = False

        # Init the config
        self.config = None

        # Init all the subscribers
        self.parrot_phy_image_sub = rospy.Subscriber("/uav1/physical_sensors/camera", Image, self._getPhysicalParrotImage)
        self.parrot_sim_image_sub = rospy.Subscriber("/uav1/simulated_sensors/camera", Image, self._getSimulatedParrotImage)
        self.unity_image_sub      = rospy.Subscriber("/uav1/unity_sensors/camera", Image, self._getUnityImage)

        # Init all the publishers
        self.img_pub = rospy.Publisher("/mixer/sensors/camera", Image, queue_size=10)

        # Start
        self.start()

    def start(self):
        self._readConfig()
        self._mainloop()

    def stop(self):
        self._quit = True            

    def _log(self, msg):
        print(str(rospy.get_name()) + ": " + str(msg))

    def _mainloop(self):
        # Set the rate and do nothing
        
        r = rospy.Rate(self.rate)
        self._log("Starting Main Loop")
        while not self._quit:           
            r.sleep()

    def _readConfig(self):
        with open('/home/carl/Desktop/MixedRealityTesting/monocular_avoidance_ws/src/mixed_reality_control/config/config.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

    def _getPhysicalParrotImage(self, msg):
        if self._processIncomingMessage('camera', '/uav1/physical_sensors/camera'):
            # Send on this message
            self.img_pub.publish(msg)

    def _getSimulatedParrotImage(self, msg):
        if self._processIncomingMessage('camera', '/uav1/simulated_sensors/camera'):
            # Send on this message
            self.img_pub.publish(msg)

    def _getUnityImage(self, msg):
        if self._processIncomingMessage('camera', '/uav1/unity_sensors/camera'):
            # Send on this message
            self.img_pub.publish(msg)

    def _processIncomingMessage(self, sensor, topic):
        try:
            if self.config[sensor] != topic:
                return False
        except KeyError as e:
            self._log("Key Error")
            return False
        except TypeError as e:
            self._log("Type Error")
            return False

        # Return true if all conditions are met
        return True


if __name__ == "__main__":
    try:
        x = Mixer()
        x.start()
    except KeyboardInterrupt:
        print("Manually Aborted")
        x.stop()

    print("System Exiting\n")
    sys.exit(0)

#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

import rospy
import yaml
import sys
import copy
import cv2
import os

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

        # Create the bridge
        self.bridge = CvBridge()

        # Used to save any overlay sensors
        self.overlay = {}

        # Init all the subscribers
        self.parrot_phy_image_sub = rospy.Subscriber("/uav1/physical_sensors/camera", Image, self._getPhysicalParrotImage)
        self.parrot_sim_image_sub = rospy.Subscriber("/uav1/sphinx_sensors/camera", Image, self._getSimulatedParrotImage)
        self.unity_image_sub      = rospy.Subscriber("/uav1/unity_sensors/camera", Image, self._getUnityImage)
        self.overlay_image_sub    = rospy.Subscriber("/uav1/overlay/camera", Image, self._getOverlayImage)

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
        home_dir = os.path.abspath(os.curdir)[:-4]
        with open(home_dir + '/Desktop/MixedRealityTesting/monocular_avoidance_ws/src/mixed_reality/config/config.yaml') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

    def _getPhysicalParrotImage(self, msg):
        base_sensor, overlay_sensor = self._processIncomingMessage('camera', '/uav1/physical_sensors/camera')
        result = self._computeOutgoingMessage('camera', base_sensor, overlay_sensor, msg)
        if result is not None:
            self.img_pub.publish(result)

    def _getSimulatedParrotImage(self, msg):
        base_sensor, overlay_sensor = self._processIncomingMessage('camera', '/uav1/sphinx_sensors/camera')
        result = self._computeOutgoingMessage('camera', base_sensor, overlay_sensor, msg)
        if result is not None:
            self.img_pub.publish(result)

    def _getUnityImage(self, msg):
        base_sensor, overlay_sensor = self._processIncomingMessage('camera', '/uav1/unity_sensors/camera')
        result = self._computeOutgoingMessage('camera', base_sensor, overlay_sensor, msg)
        if result is not None:
            self.img_pub.publish(result)

    def _getOverlayImage(self, msg):
        base_sensor, overlay_sensor = self._processIncomingMessage('camera', '/uav1/overlay/camera')
        result = self._computeOutgoingMessage('camera', base_sensor, overlay_sensor, msg)
        if result is not None:
            self.img_pub.publish(result)

    def _computeOutgoingMessage(self, sensor, base_bool, overlay_bool, msg_in):
        # Get the data 
        msg = self.bridge.imgmsg_to_cv2(msg_in, desired_encoding='passthrough')

        # Check if there is an overlay, and if there is, add it to the current message
        if base_bool:
            try:
                if np.size(self.overlay[sensor]) > 0:
                    # reshape images to match sizes
                    height, width = msg.shape[:2]
                    overlay_img = cv2.resize(self.overlay[sensor], (width, height), interpolation = cv2.INTER_CUBIC)

                    # Overlay the images
                    img2gray = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2GRAY)
                    ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
                    mask_inv = cv2.bitwise_not(mask)
                    img1_bg = cv2.bitwise_and(msg, msg, mask=mask_inv)
                    img2_fg = cv2.bitwise_and(overlay_img, overlay_img, mask=mask)
                    msg = cv2.add(img1_bg,img2_fg)

            except KeyError as e:
                pass
            # Return the data in the original form
            output = self.bridge.cv2_to_imgmsg(msg, encoding="bgr8")
            return output

        if overlay_bool:
            # Save the image to be used later
            self.overlay[sensor] = copy.deepcopy(msg)
            return None


    def _processIncomingMessage(self, sensor, topic):
        base_bool, overlay_bool = False, False
        try:
            # Check if this is an overlay or base sensor
            if self.config[sensor + "_base"] == topic:
                base_bool = True
            if (self.config[sensor + "_overlay"] == True) and (topic == '/uav1/overlay/' + sensor):
                overlay_bool = True
            # Return if they are
            return base_bool, overlay_bool
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

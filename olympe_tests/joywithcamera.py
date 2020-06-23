from __future__ import print_function, absolute_import
from random import random
from time import sleep

import threading
import pygame
import sys
import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile

try:
    from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
    from olympe.messages.ardrone3.Piloting import TakeOff, Landing
    from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
    import olympe
except ImportError:
    print("Olympe has not been loaded yet! Cannot run the app", file=sys.__stderr__)
    sys.exit(0)


class DroneController:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.1"
    LAND_TAKEOFF_TIME = 4.0


    def __init__(self, drone=None, speed=65, refresh_move=0.2):
        """"""
        try:
            self._quit_pressed = None

            # Create drone parameters and object
            self.drone_speed = min([speed, 100])
            self.drone_mtime = min([refresh_move, 1]) # move time
            self.drone = drone

            # Launch pygame controller
            pygame.init()
            self.joy = pygame.joystick.Joystick(0)  

            # Set up video capture
            self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
            print("Olympe streaming example output dir: {}".format(self.tempd))
            self.h264_frame_stats = []
            self.h264_stats_file = open(os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
            self.h264_stats_writer = csv.DictWriter(self.h264_stats_file, ['fps', 'bitrate'])
            self.h264_stats_writer.writeheader()

            # Connect to drone
            self.drone.connection()

            # Setup your callback functions to do some live video processing
            self.drone.set_streaming_callbacks(
                raw_cb=self.yuv_frame_cb,
                h264_cb=self.h264_frame_cb
            )

            # Start video streaming
            self.drone.start_video_streaming()

            # Turn drone into piloting mode
            self.drone.start_piloting()

        # No controller connected
        except pygame.error as e:
            print(e)
            print("\n(There is no joystick connected to the system)")
            sys.exit(0)


    def start(self):
        # Initialize the joystick
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))

        # Start the flying loop
        self._mainloop()


    def stop(self):
        self._quit_pressed = True


    def _close_conn(self):
        # Properly stop the video stream, piloting and disconnect
        self.drone.stop_video_streaming()
        self.drone.stop_piloting()
        self.drone.disconnection()

        # Close the stats file
        self.h264_stats_file.close()


    def _get_joy_values(self):
        pygame.event.pump()

        #Read input from the two joysticks and take only the ones we need
        out_joys = [self.joy.get_axis(i) for i in [0,1,3,4]]

        return out_joys


    def _is_takeoff_pressed(self):
        return self.joy.get_button(3) == 1


    def _is_landed_pressed(self):
        return self.joy.get_button(0) == 1


    def _check_quit_pressed(self):
        self._quit_pressed = self.joy.get_button(8) == 1


    def _mainloop(self):
        """"""
        while not self._quit_pressed:
            sleep(0.2)
            joy_values = self._get_joy_values()
            
            if self._is_takeoff_pressed():
                print("Pressed takeoff button!")
                self._takeoff()
            elif self._is_landed_pressed():
                print("Pressed landing button!")
                self._land()
            else:
                print(joy_values)
                self.move(joy_values)
            
            self._check_quit_pressed()

        print("\n============")
        print("Pressed QUIT button (X)")
        print("============\n")

        self._land()
        self._close_conn()


    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.
            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful informations
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        cv2frame = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        # Use OpenCV to show this frame
        cv2.imshow("Olympe Streaming Example", cv2frame)
        cv2.waitKey(1)  # please OpenCV for 1 ms...


    def h264_frame_cb(self, h264_frame):
        """
        This function will be called by Olympe for each new h264 frame.
            :type yuv_frame: olympe.VideoFrame
        """

        # Get a ctypes pointer and size for this h264 frame
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()

        # For this example we will just compute some basic video stream stats
        # (bitrate and FPS) but we could choose to resend it over an another
        # interface or to decode it with our preferred hardware decoder..

        # Compute some stats and dump them in a csv file
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})


    def _takeoff(self):
        """"""
        print("Takeoff if necessary...")
        self.drone(
            TakeOff(_no_expect=True)
            & FlyingStateChanged(state="hovering", _timeout=5, _policy="check_wait")
        ).wait()


    def _land(self):
        """Lands the drone"""
        print("Landing...")
        self.drone(
            Landing()
            >> FlyingStateChanged(state="landed", _timeout=5)
        ).wait()




    def move(self, joy_values):
        """
        Move in the desired direction given the (normalized) Joystick values:
        [LeftThumbXAxis, LeftThumbYAxis, RightThumbXAxis, RightThumbYAxis, Select/Quit]
        """
        # movements must be in [-100:100]
        left_right, front_back, turning, up_down = [
            int(j * self.drone_speed) for j in joy_values
        ]

        self.drone.piloting_pcmd(
            roll=left_right,
            pitch=-front_back,
            yaw=turning,
            gaz=-up_down,
            piloting_time=self.drone_mtime
        )


if __name__ == "__main__":
    drone = olympe.Drone(DroneController.SIMULATED_IP)

    try:
        x = DroneController(drone)
        x.start()
    except KeyboardInterrupt:
        x.stop()

    print("Teleoperating stopped\n")
    sys.exit(0)

from __future__ import print_function, absolute_import
from random import random
from time import sleep

import threading
import pygame
import sys

try:
    from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
    from olympe.messages.ardrone3.Piloting import TakeOff, Landing
    from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
    import olympe
except ImportError:
    print("Olympe has not been loaded yet! Cannot run the app", file=sys.__stderr__)
    sys.exit(0)


class JoystickTeleop:
    PHYSICAL_IP = "192.168.42.1"
    SIMULATED_IP = "10.202.0.2"
    LAND_TAKEOFF_TIME = 4.0

    def __init__(self, drone=None, speed=65, refresh_move=0.2):
        """"""
        try:
            self._quit_pressed = None
            self.drone_speed = min([speed, 100])
            self.drone_mtime = min([refresh_move, 1]) # move time

            pygame.init()
            self.joy = pygame.joystick.Joystick(0)  

            self.drone = drone
            self.drone.connection()
            self.drone.start_piloting()
        except pygame.error as e:
            print(e)
            print("\n(There is no joystick connected to the system)")
            sys.exit(0)

    def start(self):
        self.joy.init()
        print("Initialized Joystick: {}".format(self.joy.get_name()))
        self._mainloop()

    def stop(self):
        self._quit_pressed = True

    def _close_conn(self):
        self.drone.stop_piloting()
        self.drone.disconnection()
        
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
                # print(joy_values)
                self.move(joy_values)
            
            self._check_quit_pressed()

        print("\n============")
        print("Pressed QUIT button (X)")
        print("============\n")
        self._land()
        self._close_conn()  # closes the connection

    def _takeoff(self):
        """"""
        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=5, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=5, _policy="check_wait")
                )
            )
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
            left_right, -front_back, turning, -up_down,
            self.drone_mtime
        )


if __name__ == "__main__":
    drone = olympe.Drone(JoystickTeleop.SIMULATED_IP)

    try:
        x = JoystickTeleop(drone)
        x.start()
    except KeyboardInterrupt:
        x.stop()

    print("Teleoperating stopped\n")
    sys.exit(0)

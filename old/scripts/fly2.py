import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing

drone = olympe.Drone("10.202.1.1")
drone.connection()
drone(TakeOff()).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(moveBy(0,  1, 0, 0)).wait()
drone(moveBy(0, -1, 0, 0)).wait()
drone(Landing()).wait()
drone.disconnection()

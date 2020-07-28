#!/usr/bin/env python3
from enum import Enum

class ProgramState(Enum):
    INACTIVE    = 0
    STARTING    = 1
    SCENARIO    = 2
    ENDING      = 3

class DroneState(Enum):
    INACTIVE        = 0
    TAKEOFF         = 1
    HOVERING        = 2
    AVOIDING        = 3
    WAYPOINT        = 4
    GATENAVIGATION  = 5
    LANDING         = 6
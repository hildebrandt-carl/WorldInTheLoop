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
    MANUAL          = 3
    AVOIDING        = 4
    WAYPOINT        = 5
    GATENAVIGATION  = 6
    STAIRNAVIGATION = 7
    FOLLOWPERSON    = 8
    LANDING         = 9
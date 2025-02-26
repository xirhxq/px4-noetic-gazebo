#!/usr/bin/env python3

from enum import Enum

class State(Enum):
    INIT = 0
    TAKEOFF = 1
    PREPARE = 2
    GUIDANCE = 3
    BACK = 4
    LAND = 5
    END = 6
    THROTTLE_TEST = 7
    ATTITUDE_BIAS_TEST = 8
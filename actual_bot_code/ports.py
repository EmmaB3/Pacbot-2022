from enum import Enum


# these are bonnie's numbers; jumbot's are F0, L1, R2
class TcaPort(Enum):
    LEFT_DIST = 0
    RIGHT_DIST = 2
    FRONT_DIST = 1
    BACK_DIST = 3

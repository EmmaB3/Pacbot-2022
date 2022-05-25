# NAME: ports.py
# PURPOSE: all-in-one place for storing the port numbers of sensors/motors on a 
#          bot, such that the same codebase can work on different bots with 
#          only this file changed
# AUTHORS: Emma Bethel

from enum import Enum


# these are bonnie's numbers; jumbot's are F0, L1, R2
class TcaPort(Enum):
    LEFT_DIST = 0
    RIGHT_DIST = 2
    FRONT_DIST = 1
    BACK_DIST = 3

#!/usr/bin/env python3

import os
import robomodules as rm
from variables import *
from .messages import MsgType, message_buffers, PacmanDirection

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 1000

# once a second, tells robot to either go up (in grid) or stop (for testing)
class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = []
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.last_command = None

    def msg_received(self, msg, msg_type):
       pass

    def tick(self):
        msg = PacmanDirection()
        if self.last_command == PacmanDirection.STOP:
            msg.direction = PacmanDirection.W
        else:
            msg.direction = PacmanDirection.STOP

        self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)


def main():
    module = MotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

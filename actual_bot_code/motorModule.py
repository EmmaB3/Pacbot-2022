#!/usr/bin/env python3

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60


class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = []
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.current_direction = None

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_DIRECTION:
            self.desired_direction = msg.direction

    def tick(self):
        pass


def main():
    module = MotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

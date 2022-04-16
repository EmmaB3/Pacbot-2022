#!/usr/bin/env python3

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers, PacmanDirection

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 0

# drive robot based on command line input
class TestCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = []
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.last_command = None

    def msg_received(self, msg, msg_type):
       pass

    def tick(self):
        msg = self._get_direction()

        self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)

    def _get_direction(self):
        print('enter robot direction (w/a/s/d/stop):')
        command = input()
        if command == 'w':
            return PacmanDirection.Direction.W
        elif command == 'a':
            return PacmanDirection.Direction.A
        elif command == 's':
            return PacmanDirection.Direction.S
        elif command == 'd':
            return PacmanDirection.Direction.D
        elif command == 'stop':
            return PacmanDirection.Direction.STOP
        else:
            print('invalid input. try again.')


def main():
    module = TestCommandModule(ADDRESS, PORT)
    while True:
        module.tick()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3



# this is just for receiving and printing messages bc it's not working

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60

# drive robot based on command line input
class TestCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_DIRECTION]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None

    def msg_received(self, msg, msg_type):
       print(msg_type, msg)

    def tick(self):
        pass


def main():
    module = TestCommandModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

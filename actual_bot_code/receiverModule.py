#!/usr/bin/env python3

# NAME: recieverModule.py
# PURPOSE: module for printing all PACMAN_DIRECTION messages sent over the 
#          local robomodules server (for use in testing/troubleshooting)
# AUTHORS: Emma Bethel
# NOTES: can be easily adapted to work for other message types

import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 60


class TestCommandModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_DIRECTION]  # insert other message types here
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

#!/usr/bin/env python3

import os, sys
import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, PacmanState
from Node import manhattanDist
from pacbot.variables import *
from pacbot.grid import *
import numpy as np

###### AI INPUT STUFF ########
import random
from time import sleep
import smarterInput
##############################

ADDRESS = os.environ.get("BIND_ADDRESS", "localhost")    # address of game engine server
# ADDRESS = os.environ.get("BIND_ADDRESS","192.168.0.196")    # address of game engine server (ryan's)
PORT = os.environ.get("BIND_PORT", 11295)               # port game engine server is listening on

SPEED = 1.0
# SPEED = 5.0 # originally 1.0
FREQUENCY = SPEED * game_frequency

GHOST_AVOID_RANGE = 6.0

char_to_direction = {
    'w': PacmanDirection.W,
    'a': PacmanDirection.A,
    's': PacmanDirection.S,
    'd': PacmanDirection.D,
    'stop': PacmanDirection.STOP
}

class InputModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

        self.loop.add_reader(sys.stdin, self.keypress)
        self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        self.state = PacmanState()
        self.state.mode = PacmanState.PAUSED
        self.lives = starting_lives
        self.clicks = 0
        self.timer = 20
        self.avoid = False
        self.grid = grid

    def _move_if_valid_dir(self, direction, x, y):
        if direction == right and grid[x + 1][y] not in [I, n]:
            self.pacbot_pos[0] += 1
            self.cur_dir = direction
            return True
        elif direction == left and grid[x - 1][y] not in [I, n]:
            self.pacbot_pos[0] -= 1
            self.cur_dir = direction
            return True
        elif direction == up and grid[x][y + 1] not in [I, n]:
            self.pacbot_pos[1] += 1
            self.cur_dir = direction
            return True
        elif direction == down and grid[x][y - 1] not in [I, n]:
            self.pacbot_pos[1] -= 1
            self.cur_dir = direction
            return True
        return False


    def msg_received(self, msg, msg_type):
        # This gets called whenever any message is received
        # This module only sends data, so we ignore incoming messages
        if msg_type == MsgType.LIGHT_STATE:
            self.state = msg
            if self.state.lives != self.lives:
                self.lives = self.state.lives
                self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]

     # this function will get called in a loop with FREQUENCY frequency
    def tick(self):
        if self.state.mode != PacmanState.PAUSED:
       
            out_char = self._pick_direction()

            msg = PacmanDirection()
            msg.direction = char_to_direction[out_char]

            self.write(msg.SerializeToString(), MsgType.PACMAN_LOCATION)

    def _pick_direction(self):
        
        self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e
        
        ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.blue_ghost, self.state.orange_ghost]

        frightened = [ghost for ghost in ghosts if ghost.frightened_counter > 0]

        if len(frightened) != 0:
            self.timer -= 1 # do this
            nonScared = [g for g in ghosts if not g.frightened_counter > 0]
            # positions of all frightened ghosts which are NOT in the ghost
            #   enclosure (path finder does not consider those as valid
            #   squares!)
            frightenedPos = [(ghost.x, ghost.y) for ghost in frightened if grid[ghost.x][ghost.y] != n]
            closestGhost = []
            for p in frightenedPos:
                closestGhost.append(abs(self.pacbot_pos[0] - p[0]) + abs(self.pacbot_pos[1] - p[1]))
            goalPos = frightenedPos[np.argmin(closestGhost)]
        else:
            nonScared = ghosts
            powerPos = [(1, 7), (1, 27), (26, 7), (26, 27)]
            closestPower = []
            for index in range(len(powerPos) -1, -1, -1):
                pos = powerPos[index]
                if grid[pos[0]][pos[1]] != e:
                    closestPower.append(abs(self.pacbot_pos[0] - pos[0]) + abs(self.pacbot_pos[1] - pos[1]))
                else:
                    powerPos.remove(pos)
            closestPower.reverse()
            
            if len(closestPower) != 0:
                goalPos = powerPos[np.argmin(closestPower)]
            else:
                ghostDists = []
                for ghost in ghosts:
                    ghostDists.append(manhattanDist(self.pacbot_pos, (ghost.x, ghost.y)))
                if not self.avoid and min(ghostDists) < GHOST_AVOID_RANGE:
                    self.avoid = True
                if self.avoid and min(ghostDists) > GHOST_AVOID_RANGE + 1:
                    self.avoid = False

                if not self.avoid:
                    goalPos = smarterInput.findClosestBFS(self.grid, *self.pacbot_pos)[:2]
                    print("now going for regular pellet at", goalPos)
                else:
                    closestGhost = ghosts[np.argmin(ghostDists)]
                    goalPos = (closestGhost.x, closestGhost.y)
                    print(f'now avoiding ghost at {goalPos}')


        return smarterInput.whichWayAStar(self.grid, self.pacbot_pos, 
                                                        goalPos, nonScared, self.avoid)


def main():
    module = InputModule(ADDRESS, PORT)
    module.run()

if __name__ == "__main__":
    main()

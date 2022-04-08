#!/usr/bin/env python3

import os, sys, curses
import robomodules as rm
from messages import *
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
PORT = os.environ.get("BIND_PORT", 11297)               # port game engine server is listening on

SPEED = 1.0
# SPEED = 5.0 # originally 1.0
FREQUENCY = SPEED * game_frequency

class InputModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.FULL_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

        self.loop.add_reader(sys.stdin, self.keypress)
        self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        self.cur_dir = right
        self.next_dir = right
        self.state = PacmanState()
        self.state.mode = PacmanState.PAUSED
        self.lives = starting_lives
        self.clicks = 0

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
        if msg_type == MsgType.FULL_STATE:
            self.state = msg
            if self.state.lives != self.lives:
                self.lives = self.state.lives
                self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        if msg_type == MsgType.LIGHT_STATE:
            self.light_state = msg

    def tick(self):
        
        
        
       
        
        
        # this function will get called in a loop with FREQUENCY frequency
        if self.state.mode != PacmanState.PAUSED:

            ###
            try: 
                newgrid
            except NameError:
                newgrid = grid

            # ghostPos = [(self.state.red_ghost.x, self.state.red_ghost.y),
            #     (self.state.pink_ghost.x, self.state.pink_ghost.y),
            #     (self.state.blue_ghost.x, self.state.blue_ghost.y),
            #     (self.state.orange_ghost.x, self.state.orange_ghost.y)]
            
            ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.blue_ghost, self.state.orange_ghost]
            # print(dir(self.state.red_ghost))
            frightened = [ghost for ghost in ghosts if ghost.frightened_counter > 0]

            if len(frightened) != 0:
                nonScared = [g for g in ghosts if not g.frightened_counter > 0]
                frightenedPos = [(ghost.x, ghost.y) for ghost in frightened]
                closestGhost = []
                for p in frightenedPos:
                    closestGhost.append(abs(self.pacbot_pos[0] - p[0]) + abs(self.pacbot_pos[1] - p[1]))
                closestGhostIndex = np.argmin(closestGhost)
                outChar, newgrid = smarterInput.whichWayAStar(newgrid, self.pacbot_pos, 
                                                              self.state, frightenedPos, closestGhostIndex, nonScared)
            else:
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
                    closestPowerIndex = np.argmin(closestPower)
                    outChar, newgrid = smarterInput.whichWayAStar(newgrid, self.pacbot_pos, 
                                                                self.state, powerPos, closestPowerIndex, ghosts)
                else:
                    outChar, newgrid = smarterInput.whichWayBFS(newgrid, self.pacbot_pos)
            print("dir: " + str(outChar))
            if outChar == 'a':
                self.next_dir = left
            elif outChar == 'd':
                self.next_dir = right
            elif outChar == 'w':
                self.next_dir = up
            elif outChar == 's':
                self.next_dir = down
            ###

            if not self._move_if_valid_dir(self.next_dir, self.pacbot_pos[0], self.pacbot_pos[1]):
                self._move_if_valid_dir(self.cur_dir, self.pacbot_pos[0], self.pacbot_pos[1])
        pos_buf = PacmanState.AgentState()
        pos_buf.x = self.pacbot_pos[0]
        pos_buf.y = self.pacbot_pos[1]

        #### AI STUFF #####################################
        ####### random movement
        # char = chars[random.randrange(0, 4, 1)]
        # sleep(1/FREQUENCY)
        # print(char)
        # if char == 'a':
        #     self.next_dir = left
        # elif char == 'd':
        #     self.next_dir = right
        # elif char == 'w':
        #     self.next_dir = up
        # elif char == 's':
        #     self.next_dir = down

        ####### smarter movement
        # try: 
        #    newgrid
        # except NameError:
        #     newgrid = grid


        # char, newgrid = smarterInput.whichWayBFS(newgrid, self.pacbot_pos)
        # print("Inputted direction: " + str(char))
        # if char == 'a':
        #     self.next_dir = left
        # elif char == 'd':
        #     self.next_dir = right
        # elif char == 'w':
        #     self.next_dir = up
        # elif char == 's':
        #     self.next_dir = down
        #######################################################

        pos_buf.direction = self.cur_dir
        self.write(pos_buf.SerializeToString(), MsgType.PACMAN_LOCATION)

    def keypress(self):
        print("oh no")

###### AI INPUT STUFF ########
chars = ['a', 'd', 'w', 's']
##############################

def main():
    module = InputModule(ADDRESS, PORT)
    curses.wrapper(lambda scr: module.run())

if __name__ == "__main__":
    main()

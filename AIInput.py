#!/usr/bin/env python3

import os, sys, curses
import robomodules as rm
from enum import Enum
from messages import *
from pacbot.variables import *
from pacbot.grid import *
import numpy as np
from Node import manhattanDist

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

FRUIT_POS = (13, 13)

GHOST_AVOID_RANGE = 6.0

SQUARES_PER_SEC = SPEED * FREQUENCY

# times are measured in ticks (calls of the planning function), not seconds.
#   could change this later (& just decrement by 1/FREQUENCY instead of 1
#   during each tick)
FRUIT_TIME = 10 * FREQUENCY
FRIGHTENED_TIME = 20 * FREQUENCY

class AiMode(Enum):
    FRUIT = 0
    GHOST_CHASE = 1
    POWER_PELLET = 2
    REGULAR_PELLET = 3
    GHOST_AVOID = 4


class InputModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

        self.loop.add_reader(sys.stdin, self.keypress)
        self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        self.cur_dir = right
        self.next_dir = right
        self.state = LightState()
        self.state.mode = LightState.GameMode.PAUSED
        self.lives = starting_lives
        self.clicks = 0
        self.fruit_timer = 0
        self.ghost_timer = 0
        self.avoid = False
        self.prev_fruit_val = False

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
    
    def tick(self):
        if self.state.mode != LightState.GameMode.PAUSED:
            # set up grid
            try: 
                newgrid
            except NameError:
                newgrid = grid
            
            # decrement timers
            if self.ghost_timer > 0:
                self.ghost_timer -= 1
            if self.fruit_timer > 0:
                self.fruit_timer -= 1

            print(f'ghost time: {self.ghost_timer}, fruit time: {self.fruit_timer}')
            if self.ghost_timer == 0:
                print('GHOST TIME IS 0')
            if self.fruit_timer == 0:
                print('FRUIT TIME IS 0')
            
            if newgrid[self.pacbot_pos[0]][self.pacbot_pos[1]] != e:
                # if just ate a power pellet, start frightened timer
                if newgrid[self.pacbot_pos[0]][self.pacbot_pos[1]] == O:
                    self.ghost_timer = FRIGHTENED_TIME
            
                # mark current spot as eaten
                newgrid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e

            print(f'cherry: {self.state.cherry}, prev cherry {self.prev_fruit_val}')
            # if fruit just appeared, start fruit timer
            if self.state.cherry and not self.prev_fruit_val and self.fruit_timer == 0:
                print('starting fruit timer')
                self.fruit_timer = FRUIT_TIME
            
            self.prev_fruit_val = self.state.cherry

            # picking direction to move in
            ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.blue_ghost, self.state.orange_ghost]

            goal_pos = self._pick_goal(ghosts, newgrid)

            non_scared_ghosts = [ghost for ghost in ghosts if ghost.state != LightState.GhostState.FRIGHTENED]
            # ok maybe we should just get non-frighteened regardless? bc if we're out of ghost time we still don't have to worry about avoiding frightened ghosts
            outChar = smarterInput.whichWayAStar(newgrid, self.pacbot_pos, 
                                                goal_pos, non_scared_ghosts, self.avoid)
            
            # updating position
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

        pos_buf.direction = self.cur_dir
        self.write(pos_buf.SerializeToString(), MsgType.PACMAN_LOCATION)

    # returns goal pos
    def _pick_goal(self, ghosts, grid):
        if self.state.cherry and manhattanDist(self.pacbot_pos, FRUIT_POS) <= SQUARES_PER_SEC * (self.fruit_timer / FREQUENCY):
            print('going for fruit!!!!')
            return FRUIT_POS
        if self.state.cherry:
            print('CHERRY TOO FAR', self.fruit_timer, manhattanDist(self.pacbot_pos, FRUIT_POS))
        
        frightened_ghosts = [ghost for ghost in ghosts if ghost.state == LightState.GhostState.FRIGHTENED]

        if len(frightened_ghosts) != 0:
            frightenedPos = [(ghost.x, ghost.y) for ghost in frightened_ghosts if grid[ghost.x][ghost.y] != n]
            closestGhost = []
            for p in frightenedPos:
                closestGhost.append(abs(self.pacbot_pos[0] - p[0]) + abs(self.pacbot_pos[1] - p[1]))
            goalPos = frightenedPos[np.argmin(closestGhost)]

            if manhattanDist(self.pacbot_pos, goalPos) <= SQUARES_PER_SEC * (self.ghost_timer / FREQUENCY):
                print(f'going for ghost at {goalPos}')
                return goalPos
            print('GHOSTS TOO FAR', self.ghost_timer, goalPos)
        
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
            print(f'going for power pellet at {goalPos}')
            return goalPos
        
        ghostDists = []
        # only avoiding ghosts that aren't in the ghost zone (otherwise path 
        #   planning returns a NoneType)
        dangerous_ghosts = [ghost for ghost in ghosts if grid[ghost.x][ghost.y] != n]
        for ghost in dangerous_ghosts:
            ghostDists.append(manhattanDist(self.pacbot_pos, (ghost.x, ghost.y)))
        if not self.avoid and min(ghostDists) < GHOST_AVOID_RANGE:
            self.avoid = True
        if self.avoid and min(ghostDists) > GHOST_AVOID_RANGE + 1:
            self.avoid = False

        if not self.avoid:
            goalPos = smarterInput.findClosestBFS(grid, *self.pacbot_pos)[:2]
            print(f'going for regular pellet at {goalPos}')
            return goalPos
        else:
            closestGhost = ghosts[np.argmin(ghostDists)]
            goalPos = (closestGhost.x, closestGhost.y)
            print(f'avoiding ghost at {goalPos}')
            return goalPos


    # def tick(self):

    #     # this function will get called in a loop with FREQUENCY frequency
    #     if self.state.mode != LightState.GameMode.PAUSED:

    #         ###
    #         try: 
    #             newgrid
    #         except NameError:
    #             newgrid = grid
            
    #         newgrid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e

    #         ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.blue_ghost, self.state.orange_ghost]

    #         frightened = [ghost for ghost in ghosts if ghost.state == LightState.GhostState.FRIGHTENED]

    #         if self.state.cherry:
    #             print('going for fruit!!!!')
    #             goalPos = FRUIT_POS
    #             nonScared = ghosts
    #         elif len(frightened) != 0:
    #             self.timer -= 1 # do this
    #             nonScared = [g for g in ghosts if not g.state == LightState.GhostState.FRIGHTENED]
    #             # positions of all frightened ghosts which are NOT in the ghost
    #             #   enclosure (path finder does not consider those as valid
    #             #   squares!)
    #             frightenedPos = [(ghost.x, ghost.y) for ghost in frightened if grid[ghost.x][ghost.y] != n]
    #             closestGhost = []
    #             for p in frightenedPos:
    #                 closestGhost.append(abs(self.pacbot_pos[0] - p[0]) + abs(self.pacbot_pos[1] - p[1]))
    #             goalPos = frightenedPos[np.argmin(closestGhost)]
    #             print(f'going for ghost at {goalPos}')
    #             # outChar = smarterInput.whichWayAStar(newgrid, self.pacbot_pos, 
    #             #                                               closestGhostPos, nonScared)
    #         else:
    #             nonScared = ghosts
    #             powerPos = [(1, 7), (1, 27), (26, 7), (26, 27)]
    #             closestPower = []
    #             for index in range(len(powerPos) -1, -1, -1):
    #                 pos = powerPos[index]
    #                 if grid[pos[0]][pos[1]] != e:
    #                     closestPower.append(abs(self.pacbot_pos[0] - pos[0]) + abs(self.pacbot_pos[1] - pos[1]))
    #                 else:
    #                     powerPos.remove(pos)
    #             closestPower.reverse()
                
    #             if len(closestPower) != 0:
    #                 goalPos = powerPos[np.argmin(closestPower)]
    #                 print(f'going for power pellet at {goalPos}')
    #             else:
    #                 ghostDists = []
    #                 for ghost in ghosts:
    #                     ghostDists.append(manhattanDist(self.pacbot_pos, (ghost.x, ghost.y)))
    #                 if not self.avoid and min(ghostDists) < GHOST_AVOID_RANGE:
    #                     self.avoid = True
    #                 if self.avoid and min(ghostDists) > GHOST_AVOID_RANGE + 1:
    #                     self.avoid = False

    #                 if not self.avoid:
    #                     goalPos = smarterInput.findClosestBFS(newgrid, *self.pacbot_pos)[:2]
    #                     print("going for regular pellet at", goalPos)
    #                 else:
    #                     closestGhost = ghosts[np.argmin(ghostDists)]
    #                     goalPos = (closestGhost.x, closestGhost.y)
    #                     print(f'avoiding ghost at {goalPos}')


    #         outChar = smarterInput.whichWayAStar(newgrid, self.pacbot_pos, 
    #                                                         goalPos, nonScared, self.avoid)

    #         print("dir: " + str(outChar))
    #         if outChar == 'a':
    #             self.next_dir = left
    #         elif outChar == 'd':
    #             self.next_dir = right
    #         elif outChar == 'w':
    #             self.next_dir = up
    #         elif outChar == 's':
    #             self.next_dir = down
    #         ###

    #         if not self._move_if_valid_dir(self.next_dir, self.pacbot_pos[0], self.pacbot_pos[1]):
    #             self._move_if_valid_dir(self.cur_dir, self.pacbot_pos[0], self.pacbot_pos[1])
    #     pos_buf = PacmanState.AgentState()
    #     pos_buf.x = self.pacbot_pos[0]
    #     pos_buf.y = self.pacbot_pos[1]

    #     #### AI STUFF #####################################
    #     ####### random movement
    #     # char = chars[random.randrange(0, 4, 1)]
    #     # sleep(1/FREQUENCY)
    #     # print(char)
    #     # if char == 'a':
    #     #     self.next_dir = left
    #     # elif char == 'd':
    #     #     self.next_dir = right
    #     # elif char == 'w':
    #     #     self.next_dir = up
    #     # elif char == 's':
    #     #     self.next_dir = down

    #     ####### smarter movement
    #     # try: 
    #     #    newgrid
    #     # except NameError:
    #     #     newgrid = grid


    #     # char, newgrid = smarterInput.whichWayBFS(newgrid, self.pacbot_pos)
    #     # print("Inputted direction: " + str(char))
    #     # if char == 'a':
    #     #     self.next_dir = left
    #     # elif char == 'd':
    #     #     self.next_dir = right
    #     # elif char == 'w':
    #     #     self.next_dir = up
    #     # elif char == 's':
    #     #     self.next_dir = down
    #     #######################################################

    #     pos_buf.direction = self.cur_dir
    #     self.write(pos_buf.SerializeToString(), MsgType.PACMAN_LOCATION)
    
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

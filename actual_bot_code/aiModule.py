#!/usr/bin/env python3

import os, sys
import robomodules as rm
from messages import MsgType, message_buffers, PacmanDirection, PacmanState, LightState
from Node import manhattanDist
from variables import *
from grid import *
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

FRUIT_POS = (13, 13)
GHOST_AVOID_RANGE = 6.0

# TODO: should be an actual experimentally- determined robot speed estimate
SQUARES_PER_SEC = SPEED * FREQUENCY

# times are measured in ticks (calls of the planning function), not seconds.
#   could change this later (& just decrement by 1/FREQUENCY instead of 1
#   during each tick)
FRUIT_TIME = 10 * FREQUENCY
FRIGHTENED_TIME = 20 * FREQUENCY

char_to_direction = {
    'w': PacmanDirection.W,
    'a': PacmanDirection.A,
    's': PacmanDirection.S,
    'd': PacmanDirection.D,
    'stop': PacmanDirection.STOP
}

class AIModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)

        self.pacbot_pos = [pacbot_starting_pos[0], pacbot_starting_pos[1]]
        self.state = LightState()
        self.state.mode = LightState.GameMode.PAUSED
        self.lives = starting_lives
        self.avoid = False
        self.grid = grid
        self.fruit_timer = 0
        self.ghost_timer = 0
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

     # this function will get called in a loop with FREQUENCY frequency
    def tick(self):
        msg = PacmanDirection()

        if self.state.mode != LightState.GameMode.PAUSED:

            # decrement timers
            if self.ghost_timer > 0:
                self.ghost_timer -= 1
            if self.fruit_timer > 0:
                self.fruit_timer -= 1

            if self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] != e:
                # if just ate a power pellet, start frightened timer
                if self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] == O:
                    self.ghost_timer = FRIGHTENED_TIME
                # mark current spot as eaten
                self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e
            
                        # if fruit just appeared, start fruit timer
            if self.state.cherry and not self.prev_fruit_val and self.fruit_timer == 0:
                print('starting fruit timer')
                self.fruit_timer = FRUIT_TIME
            self.prev_fruit_val = self.state.cherry
        
            ghosts = [self.state.red_ghost, self.state.pink_ghost,
                      self.state.blue_ghost, self.state.orange_ghost]

            non_scared_ghosts = [ghost for ghost in ghosts if ghost.state != LightState.GhostState.FRIGHTENED]

            goal_pos = self._pick_goal()

            out_char = smarterInput.whichWayAStar(self.grid, self.pacbot_pos, 
                                                        goal_pos, non_scared_ghosts, self.avoid)

            msg.direction = char_to_direction[out_char]
        else:
            msg.direction = PacmanDirection.STOP

        self.write(msg.SerializeToString(), MsgType.PACMAN_DIRECTION)
    
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

    def _pick_direction(self):
        
        self.grid[self.pacbot_pos[0]][self.pacbot_pos[1]] = e
        
        ghosts = [self.state.red_ghost, self.state.pink_ghost, self.state.blue_ghost, self.state.orange_ghost]

        frightened = [ghost for ghost in ghosts if ghost.state == LightState.FRIGHTENED]

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
    module = AIModule(ADDRESS, PORT)
    module.run()

if __name__ == "__main__":
    main()

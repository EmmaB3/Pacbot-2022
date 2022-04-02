from pacbot.variables import *


def manhattanDist(currPos, goalPos) -> float:
    return abs(currPos[0] - goalPos[0]) + abs(currPos[1] - goalPos[1])


# Class definition of a node in the search space
class Node:
    def __init__(self, grid, pos, goalPos, cost, prev, ghostPos):
        self.grid = grid
        self.pos = pos
        self.goalPos = goalPos
        self.cost = cost
        self.neighbors = []
        self.prev = prev
        self.ghostPos = ghostPos

        self.state = pos

    # Overloading the greater than operator so that node comparisons can be made
    # Compares based on the cost of the state
    def __gt__(self, other):
        if self.cost > other.cost:
            return True
        else:
            return False

    # Generating neighbors for a given state.
    def generateNeighbors(self):
        self.neighbors = []
        x = self.pos[0]
        y = self.pos[1]
        newCost = self.cost
        if self.grid[x][y + 1] not in [I, n]:
            if self.grid[x][y + 1] in self.ghostPos: #Fix this
                newCost = 9999
            else:
                newCost += 1 + manhattanDist((x, y - 1), self.goalPos)
            node = Node(self.grid, (x, y + 1), self.goalPos, newCost, self, self.ghostPos)
            self.neighbors.append(node)
        if self.grid[x][y - 1] not in [I, n]:
            if self.grid[x][y + 1] in self.ghostPos: #Fix this
                newCost = 9999
            else:
                newCost += 1 + manhattanDist((x, y - 1), self.goalPos)
            node = Node(self.grid, (x, y - 1), self.goalPos, newCost, self, self.ghostPos)
            self.neighbors.append(node)
        if self.grid[x + 1][y] not in [I, n]:
            if self.grid[x][y + 1] in self.ghostPos: #Fix this
                newCost = 9999
            else:
                newCost += 1 + manhattanDist((x + 1, y), self.goalPos)
            node = Node(self.grid, (x + 1, y), self.goalPos, newCost, self, self.ghostPos)
            self.neighbors.append(node)
        if self.grid[x - 1][y] not in [I, n]:
            if self.grid[x][y + 1] in self.ghostPos: #Fix this
                newCost = 9999
            else:
                newCost += 1 + manhattanDist((x - 1, y), self.goalPos)
            node = Node(self.grid, (x - 1, y), self.goalPos, newCost, self, self.ghostPos)
            self.neighbors.append(node)

    # Method to print neighbors of a node. Used for debugging
    def printNeighbors(self):
        for neighbor in range(len(self.neighbors)):
            print(self.neighbors[neighbor].pos)
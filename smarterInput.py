from pacbot.variables import *
import Node
import HQ
import numpy as np


# Generates the path taken from the start node to the parameter node (typically
# the goal state)
def generateSolution(node, start):
    solution = []
    while node.pos != start.pos:
        solution.append(node.pos)
        node = node.prev
    solution.append(start.pos)
    solution.reverse()
    # for i in range(len(solution) - 1, -1, -1):
    #     print("Step " + str(len(solution) - i) + ":", solution[i])

    return solution


# take in the grid and the pacman position
# return which direction to go
def whichWay(grid, pacpos):
    x = pacpos[0]
    y = pacpos[1]
    if grid[x][y] in [e]:
        print("already eaten!")
    grid[x][y] = e
    if grid[x - 1][y] in [o, O]:
        return 'a', grid
    elif grid[x][y + 1] in [o, O]:
        return 'w', grid
    elif grid[x + 1][y] in [o, O]:
        return 'd', grid
    elif grid[x][y - 1] in [o, O]:
        return 's', grid
    else:  # not immediately next to a dot, find the closest one
        print("\nsearching for a dot!!\n")
        # look through every dot in the grid, return the closest
        goalX, goalY = findClosest(grid, x, y)
        print("Closest dot is at: " + str(goalX) + ", " + str(goalY))
        # go to the dot at goalX, goalY

        if goalX > x:  # we have to go right
            if grid[x + 1][y] not in [I, n]:  # not blocked
                return 'd', grid
            if goalY > y:  # we have to go up
                if grid[x][y + 1] not in [I, n]:  # not blocked
                    return 'w', grid
                # can't go up or right but need to
                if grid[x][y - 1] not in [I, n]:  # not blocked
                    return 's', grid
                return 'a'

        if goalX < x:  # we have to go left
            if grid[x - 1][y] not in [I, n]:  # not blocked
                return 'a', grid
            if goalY > y:  # we have to go up
                if grid[x][y + 1] not in [I, n]:  # not blocked
                    return 'w', grid
                if grid[x][y - 1] not in [I, n]:  # not blocked
                    return 's', grid
                return 'd'

        if goalY > y:  # we have to go up
            if grid[x][y + 1] not in [I, n]:  # not blocked
                return 'w', grid
            if goalY > y:  # we have to go up
                if grid[x + 1][y] not in [I, n]:  # not blocked
                    return 'd', grid
                if grid[x - 1][y] not in [I, n]:  # not blocked
                    return 'a', grid
                return 's'

        if goalY < y:  # we have to go down
            if grid[x][y - 1] not in [I, n]:  # not blocked
                return 's', grid
            if goalY > y:  # we have to go up
                if grid[x + 1][y] not in [I, n]:  # not blocked
                    return 'd', grid
                if grid[x - 1][y] not in [I, n]:  # not blocked
                    return 'a', grid
                return 'w'

        return 'a', grid


def whichWayBFS(grid, pacpos):
    x = pacpos[0]
    y = pacpos[1]

    if grid[x][y] in [e]:
        print("already eaten!")
    grid[x][y] = e
    if grid[x - 1][y] in [o, O]:
        return 'a', grid
    elif grid[x][y + 1] in [o, O]:
        return 'w', grid
    elif grid[x + 1][y] in [o, O]:
        return 'd', grid
    elif grid[x][y - 1] in [o, O]:
        return 's', grid
    else:  # not immediately next to a dot, find the closest one
        print("\nsearching for a dot!!\n")
        # look through every dot in the grid, return the closest
        # returns x, y, path
        goalX, goalY, path = findClosestBFS(grid, x, y)
        print("Closest dot is at: " + str(goalX) + ", " + str(goalY))
        # go to the dot at goalX, goalY
        print("Next move is to: " + str(path[1]))
        print("Next move is to: " + str(path))
        if path[1][0] > x:  # we have to go right
            return 'd', grid
        elif path[1][0] < x:  # we have to go left
            return 'a', grid
        elif path[1][1] > y:  # we have to go up
            return 'w', grid
        elif path[1][1] < y:
            return 's', grid
        else:
            print("\n****\n****\nLINE\n\n\n")
            print(path[0])
            print(path[1])
            print("\n****\n****\nLINE\n\n\n")
            return 's', grid


def whichWayAStar(grid, pacpos, state):
    x = pacpos[0]
    y = pacpos[1]
    print("My position is", x, y)
    if grid[x][y] == e:
        print("already eaten!")
    grid[x][y] = e
    powerPos = [(1, 7), (1, 27), (26, 7), (26, 27)]  # TO-DO: Find closest power pellet
    ghostPos = [(state.red_ghost.x, state.red_ghost.y),
                (state.pink_ghost.x, state.pink_ghost.y),
                (state.blue_ghost.x, state.blue_ghost.y),
                (state.orange_ghost.x, state.orange_ghost.y)]
    closestPower = []
    for index in range(len(powerPos) -1, -1, -1):
        pos = powerPos[index]
        if grid[pos[0]][pos[1]] != e:
            closestPower.append(abs(x - pos[0]) + abs(y - pos[1]))
        else:
            powerPos.remove(pos)
    closestPower.reverse()
    closestPowerIndex = np.argmin(closestPower)
    print('power pos', powerPos)
    print('closest power', closestPowerIndex)
    goalX, goalY, path = findClosestAStar(grid, x, y, powerPos[closestPowerIndex], ghostPos)
    print("Closest goal is at: " + str(goalX) + ", " + str(goalY))
    # go to the dot at goalX, goalY
    print("Next move is to: " + str(path[1]))
    print("Next move is to: " + str(path))
    if path[1][0] > x:  # we have to go right
        return 'd', grid
    elif path[1][0] < x:  # we have to go left
        return 'a', grid
    elif path[1][1] > y:  # we have to go up
        return 'w', grid
    elif path[1][1] < y:
        return 's', grid
    else:
        print("\n****\n****\nLINE\n\n\n")
        print(path[0])
        print(path[1])
        print("\n****\n****\nLINE\n\n\n")
        return 's', grid


def distTo(x, y, newx, newy):
    return ((newx - x) ** 2 + (newy - y) ** 2) ** 1 / 2


def findClosest(grid, x, y):
    closestDot = -1, -1
    closestDist = 1000
    for xs in range(len(grid)):
        for ys in range(len(grid[0])):
            if grid[xs][ys] in [o, O]:
                dist = distTo(x, y, xs, ys)
                if dist < closestDist:
                    closestDist = dist
                    closestDot = xs, ys
    return closestDot


def findClosestBFS(grid, x, y):
    childPath = []
    queue = []

    while len(queue) > 0:
        args = queue.pop(0)
        childX = args[0]
        childY = args[1]
        childPath = args[2]
        if (childX, childY) not in childPath:
            childPath.append((childX, childY))
            if grid[childX][childY] in [o, O]:  # found the dot
                return (childX, childY, childPath)
            if grid[childX - 1][childY] not in [I, n]:  # not blocked
                queue.append((childX - 1, childY, childPath.copy()))
            if grid[childX + 1][childY] not in [I, n]:  # not blocked
                queue.append((childX + 1, childY, childPath.copy()))
            if grid[childX][childY - 1] not in [I, n]:  # not blocked
                queue.append((childX, childY - 1, childPath.copy()))
            if grid[childX][childY + 1] not in [I, n]:  # not blocked
                queue.append((childX, childY + 1, childPath.copy()))


def findClosestAStar(grid, x, y, goalPos, ghostPos):
    print(goalPos)
    print(ghostPos)
    pq = HQ.HQ()
    start = Node.Node(grid, (x, y), goalPos, 0, None, ghostPos)
    pq.insert(start)
    visited = set()
    nodeNumber = 0
    while True:
        # Checking if the priority queue is empty. In other words, making sure
        # there are nodes to visit.
        if pq.isEmpty():
            print("Failure")
            break
        node = pq.remove()
        nodeNumber += 1
        print("***NODE NUMBER: ", nodeNumber)
        # "Marking" (adding to a list) the state as visited
        visited.add(node.pos)
        # Checks if the current state is the goal state
        if node.pos == goalPos:
            print("Done!")
            print("Num states visited:", len(visited))
            print("Path steps:")
            return goalPos[0], goalPos[1], generateSolution(node, start)
        # Generating neighbors of the current state and adding them to the PQ
        # if not visited already
        node.generateNeighbors()
        for i in range(len(node.neighbors)):
            # Checking if the neighbor was visited or already in the PQ
            if node.neighbors[i].pos not in visited and (pq.contains(node.neighbors[i].pos) == -1):
                pq.insert(node.neighbors[i])
            # Updating cost if necessary
            elif pq.contains(node.neighbors[i].pos) > -1:
                index = pq.contains(node.neighbors[i].pos)
                newCost = node.neighbors[i].cost
                currentCost = pq.q[index].cost
                if newCost < currentCost:
                    pq.q[index] = node.neighbors[i]
    print("reached end")

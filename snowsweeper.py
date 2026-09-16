#!/usr/bin/env python3
#
#   hw5_localize.py
#
#   Homework 5 code framework to localize a robot in a grid...
#
#   Places to edit are marked as FIXME.
#
import numpy as np

from utilities import Visualization, Robot, Node
import bisect, copy
from sympy.utilities.iterables import multiset_permutations


#
#  Define the Walls
#
w = ['xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x               x               x               x',
    #  'x                x             x                x',
    #  'x                 x           x                 x',
    #  'x        xxxx      x         x                  x',
    #  'x        x   x      x       x                   x',
    #  'x        x    x      x     x      xxxxx         x',
    #  'x        x     x      x   x     xxx   xxx       x',
    #  'x        x      x      x x     xx       xx      x',
    #  'x        x       x      x      x         x      x',
    #  'x        x        x           xx         xx     x',
    #  'x        x        x           x           x     x',
    #  'x        x        x           x           x     x',
    #  'x        x        x           x           x     x',
    #  'x                 xx         xx           x     x',
    #  'x                  x         x                  x',
     'x                  xx       xx                  x',
     'x                   xxx   xxx                   x',
     'x                     xxxxx         x           x',
     'x                                   x          xx',
     'x                                   x         xxx',
     'x            x                      x        xxxx',
     'x           xxx                     x       xxxxx',
     'x          xxxxx                    x      xxxxxx',
     'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx']

w = ['xxxxxxxxxxxxxxxxxxxxxxxxxx',
     'x   x               x    x',
    #  'x                        x',
     'x      xx       xx       x',
     'x       xxx   xxx        x',
     'x         xxxxx      x   x',
     'x                    x   x',
     'xxxxxxxxxxxxxxxxxxxxxxxxxx']

# w = ['xxxxxxx',
#      'x     x',
#      'x   x x',
#      'x  x  x',
#      'xxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)
# directions = [ (-1, 1), (-1, -1), (1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1)]
directions = [(-1, -1), (1, -1), (0, -1), (1, 0), (-1, 0), (1, 1), (0, 1), (-1, 1)]
directions = [(1, 0), (0, 1), (0, -1), (-1, 0)]
# directions = [ (0, 1), (0, -1), (-1, 0), (1, 0),  (1, 1), (1, -1), (-1, 1), (-1, -1), ]

def distTransform(goal):
    dists = np.zeros((rows, cols))
    queue = [goal]
    visited = [goal]
    while len(queue) > 0:
        curr = queue.pop(0)
        # print(curr, "dist : ", dists[curr[0]][curr[1]])
        for (i, (drow, dcol)) in enumerate(directions):
            if i <= 8:
                if curr[0] + drow >= 0 and curr[1] + dcol >= 0\
                and curr[0] + drow < rows and curr[1] + dcol < cols \
                and [curr[0] + drow, curr[1] + dcol] not in visited\
                and not walls[curr[0] + drow][curr[1] + dcol]:
                    child = [curr[0] + drow, curr[1] + dcol]
                    dists[child[0]][child[1]] = dists[curr[0]][curr[1]] + 1
                    queue.append(child)
                    # print(visited, [curr[0] + drow, curr[1] + dcol])
                    # print(dists, "curr : ", curr, " q : ", queue)
                    visited.append(child)
    return dists
    
    # goalNode = Node(goal[0], goal[1], 0)
    # startNode = None
    # queue = [goalNode]
    # visited = []
    # while len(queue) > 0:
    #     node = queue.pop()
    #     print(node)
    #     if node.row == start[0] and node.col == start[1]:
    #         startNode = node
    #     for (i, (drow, dcol)) in enumerate(directions):
    #         if node.row + drow >= 0 and node.col + dcol >= 0\
    #         and node.row + drow < rows and node.col + dcol < cols \
    #         and (node.row + drow, node.col + dcol) not in visited\
    #         and not walls[node.row + drow][node.col + dcol]:
    #             child = Node(node.row + drow, node.col + dcol, node.dist + 1)
    #             child.neighbors[(i + 2) % 4] = node
    #             queue.append(child)
    #     visited.append((node.row, node.col))
    # return startNode


def checkFinished(goal, snow):
    for row in range(len(snow)):
        for col in range(len(snow[row])):
            if [row, col] != goal and snow[row][col] > 0:
                # print(snow)
                print(snow[goal[0]][goal[1]])
                return False
    return True

paths = []
finished = []

def pathRec(dists, goal, path, curr, snow, visited, prev, d):
    # print("prev : ", prev, "curr : ", curr)
    sortedIdx = []
    # print(curr)
    if curr == goal:
        paths.append(path)
        finished.append(snow[goal[0]][goal[1]])
        if len(finished) > 20:
            return True
        return
        # return snow[goal[0]][goal[1]]
        # return path
    maxDist = -1
    for i in range(len(d)):
        drow = d[i][0]
        dcol = d[i][1]
        nextRow = curr[0] + drow
        nextCol = curr[1] + dcol
        if [nextRow, nextCol] not in visited\
        and curr[0] + drow >= 0 and curr[1] + dcol >= 0\
        and curr[0] + drow < rows and curr[1] + dcol < cols \
        and not walls[nextRow][nextCol]:
            if dists[nextRow][nextCol] > maxDist:
                sortedIdx = [i]
                maxDist = dists[nextRow][nextCol]
            elif dists[nextRow][nextCol] ==  maxDist:
                sortedIdx.append(i)
            # print("idxs append: ", sortedIdx)
    # sortedIdx.sort(reverse=True, key=lambda a : dists[curr[0] + d[a][0]][curr[1] + d[a][1]])


    # temp_visited = copy.deepcopy(visited)
    # temp_visited.append(curr)
    visited.append(curr)
    
    # print(sortedIdx)
    for idx in sortedIdx:
        drow = d[idx][0]
        dcol = d[idx][1]
        nextNode = [curr[0] + drow, curr[1] + dcol]
        # print("next: ", nextNode, drow, dcol)
        
        temp_path = copy.deepcopy(path)
        temp_path.append(d[idx])
        
        
        newSnow = copy.deepcopy(snow)
        newSnow[nextNode[0]][nextNode[1]] += newSnow[curr[0]][curr[1]]
        newSnow[curr[0]][curr[1]] = 0
        
        # print("next : ", nextNode, "curr : ", curr)
        # print("prev snow : ", snow)
        # print("next snow : ", newSnow)
        # print("curr : ", curr, "idxs : ", sortedIdx)
        # print("idx : ", idx)
        # print("dist : ", dists[curr[0] + directions[idx][0]][curr[1] + directions[idx][1]])
        
        result = pathRec(dists, goal, temp_path, nextNode, newSnow, visited, curr, d)
        if result:
            return
        # if result is not None:
        #     return
        # if result is not None:
        #     return result
    # return None
    
def pathFromGraph(dists, start, goal, snow):
   
    # permutations =  [i for i in multiset_permutations(directions)]
    # permut = None
    # pidx = -1
    # maxSum = 93
    # maxIdx = -1
    # for i in range(len(permutations)):
    #     d = permutations[i]
    #     pathRec(dists, goal, [], start, snow, [], start, d)
    #     if len(finished) > 0:
    #         idx = np.argmax(finished)
    #         if finished[idx] == maxSum:
    #             permut = d
    #             pidx = i
    #             maxSum = finished[idx]
    #             maxIdx = idx
    #             break
    #     if i % 500 == 0:
    #         # print(i,"Final: ", finished[idx], "Of : ", np.sum(snow))
    #         print(i)
    pathRec(dists, goal, [], start, snow, [], start, directions)
    # print(finished)
    maxIdx = np.argmax(finished)
    print("Final: ", finished[maxIdx], "Of : ", np.sum(snow))
    # print("directions: ", permut, permutations[pidx])
    return paths[maxIdx]
# 
#
#  Main Code
#
def main():
    # Initialize the robot simulation.
    # Prob 1(a)  robot=Robot(walls)
    # Prob 1(b)  robot=Robot(walls, row=12, col=26)
    # Prob 2     robot=Robot(walls, row=12, col=26, probProximal=[0.9,0.6,0.3])
    # Prob 3     robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3],
    #                        probCmd=0.8)
    # Prob 4     robot=Robot(walls, row=15, col=47, probProximal=[0.9,0.6,0.3],
    #                        probCmd=0.8, kidnap=True)
    # Or to play robot=Robot(walls, probProximal=[0.9,0.6,0.3], probCmd=0.8)
    
    robot = Robot(walls, row = 1, col = 1)
    
    start = [robot.row, robot.col]
    
    # goal = (np.random.randint(0, rows), np.random.randint(0, cols))
    # while walls[goal[0]][goal[1]]:
    #     goal = (np.random.randint(0, rows), np.random.randint(0, cols))
    goal = [3, 4]
        
    dists = distTransform(goal)
    # print(dists)
    


    # Initialize the figure.
    visual = Visualization(walls, robot, dists)

    # Start with a uniform belief grid.
    snow = (1.0 - walls)
    
    # path = pathFromGraph(dists, start, goal, snow)
    
    path = [(1, 0), (1, 0), (1, 0), (1, 0), (0, 1), \
        (-1, 0), (-1, 0), (-1, 0), (-1, 0), (0, 1), \
            (1, 0), (1, 0), (1, 0), (1, 0), (0, 1), \
            (0, 1), (0, 1), (0, 1), (0, 1), (0, 1), \
            (0, 1), (0, 1), (0, 1), (0, 1), (0, 1), \
            (0, 1), (0, 1), (0, 1), (0, 1), (0, 1),\
            (0, 1), (-1, 0), (-1, 1), (1, 1), (1, 0), \
            (0, 1), (0, 1), (-1, 0), (-1, 0), (-1, 0), \
            (-1, 0), (1, -1), (1, 0), (1, 0), (-1, -1), \
            (-1, 0), (-1, 1), (0, -1), (1, -1), (-1, 0), \
            (1, -1), (1, 0), (-1, -1), (1, 0), (1, 0), \
            (-1, -1), (1, 0), (-1, -1), (-1, 1), (-1, 1), \
            (0, -1), (0, -1), (0, -1), (1, -1), (-1, 0), \
            (1, -1), (-1, 0), (1, -1), (1, 0), (-1, -1), \
            (-1, 1), (0, -1), (1, -1), (1, 1), (0, -1), \
            (-1, -1), (-1, 1), (0, -1), (1, -1), (-1, 0), \
            (0, -1), (0, -1), (0, -1), (0, -1), #(1, -1),\
            (1, 1), (0, -1), (0, -1),
            (1, 1), (1, 1), (0, 1), (-1, -1), (1, -1), (0, -1), (-1, 0)]
    
    print(path)
    # print(bel)
    input("continue")
    
    for p in path:
        # while True:
        #     key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
        #     if   (key == 'q'):  return
        #     else:
        #         break
        prevRobot = (robot.row, robot.col)
        # (drow, dcol) = directions[p]
        (drow, dcol) = p
        if robot.row + drow >= 0 and robot.col + dcol >= 0\
            and robot.row + drow < rows and robot.col + dcol < cols \
            and not walls[robot.row + drow][robot.col + dcol]:
                snow[robot.row + drow][robot.col + dcol] += snow[robot.row][robot.col]
                snow[robot.row][robot.col] = 0
        robot.Command(drow, dcol)
        print("p : ", p, "row, col : ", (robot.row, robot.col))
        
        visual.Show(snow, markRobot=True, goal = goal, wait = 0.2, prevRobot = prevRobot)
    input("finished")
        
    #     # Move the robot in the simulation.
    #     robot.Command(drow, dcol)

    # Loop continually.
    # while True:
    #     print(snow)
    #     # Show the current belief.  Also show the actual position.
    #     visual.Show(snow, markRobot=True)

    #     # Get the command key to determine the direction.
    #     while True:
    #         key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
    #         if   (key == 'q'):  return
    #         elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
    #         elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
    #         elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
    #         elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break

        
    #     if robot.row + drow >= 0 and robot.col + dcol >= 0\
    #         and robot.row + drow < rows and robot.col + dcol < cols \
    #         and not walls[robot.row + drow][robot.col + dcol]:
    #             snow[robot.row + drow][robot.col + dcol] += snow[robot.row][robot.col]
    #             snow[robot.row][robot.col] = 0
                
        
    #     # Move the robot in the simulation.
    #     robot.Command(drow, dcol)


if __name__== "__main__":
    main()

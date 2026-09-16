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
    #  'x                                   x         xxx',
    #  'x            x                      x        xxxx',
    #  'x           xxx                     x       xxxxx',
    #  'x          xxxxx                    x      xxxxxx',
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

# w = ['xxxxxxxxx',
#      'x       x',
#      'x       x',
#      'x       x',
#      'x       x',
#      'x       x',
#      'xxxxxxxxx']

walls = np.array([[1.0*(c == 'x') for c in s] for s in w])
rows  = np.size(walls, axis=0)
cols  = np.size(walls, axis=1)
# directions = [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
# directions = [(1, 0), (0, 1), (1, 1), (1, -1), (-1, 1), (-1, -1), (0, -1), (-1, 0)]
# ops = [7,6,5,4,3,2,1,0]
directions = [(1, 0), (0, 1), (1, 1), (1, -1), (0, -1), (-1, 0), (-1, 1), (-1, -1)]
ops = [5,4,7,6,1,0,3,2]
# directions = [(1, 0), (0, 1), (0, -1), (-1, 0)]

def distTransform(goal, directions):
    dists = np.zeros((rows, cols))
    queue = [goal]
    visited = [goal]
    while len(queue) > 0:
        curr = queue.pop(0)
        # print(curr, "dist : ", dists[curr[0]][curr[1]])
        for (i, (drow, dcol)) in enumerate(directions):
            # if i <= 3:
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

def pathFromGraph_dep(dists, start, goal):
    visited = []
    pathIdx = [start]    
    path = []
    curr = start
    while curr is not goal:
        maxDist = -1
        maxIdx = -1
        # print(curr.row, curr.col, curr.neighbors)
        for i in range(len(directions)):
            drow = directions[i][0]
            dcol = directions[i][1]
            nextNode = [curr[0] + directions[i][0], curr[1] + directions[i][1]]
            if nextNode not in visited\
            and curr[0] + drow >= 0 and curr[1] + dcol >= 0\
            and curr[0] + drow < rows and curr[1] + dcol < cols \
            and not walls[nextNode[0]][nextNode[1]] \
            and dists[nextNode[0]][nextNode[1]] > maxDist:
                # print(nextNode, visited)
                maxDist = dists[nextNode[0]][nextNode[1]]
                maxIdx = i
        if maxIdx < 0:
            break
        path.append(directions[maxIdx])
        pathIdx.append([curr[0] + directions[maxIdx][0], curr[1] + directions[maxIdx][1]])
        
        visited.append(curr)
        # print("curr : ", curr)
        curr = [curr[0] + directions[maxIdx][0], curr[1] + directions[maxIdx][1]]
        # print("next: ", curr)
    return path

def pathFromGraph(dists, start, goal):
    visited = []
    pathIdx = []    
    path = []
    backNodes = []
    backIdxs = []
    curr = start
    while curr != goal:
        pathIdx.append(curr)
        visited.append(curr)
        
        maxDist = -1
        maxDidx = -1
        maxIdx = [-1, -1]
        # print(curr.row, curr.col, curr.neighbors)
        fin_p = -1
        fin_curr = None
        for p in range(len(pathIdx)):
            currChecking = pathIdx[-1 - p]
            for i in range(len(directions)):
                drow = directions[i][0]
                dcol = directions[i][1]
                nextRow = currChecking[0] + drow
                nextCol = currChecking[1] + dcol
                nextNode = [nextRow, nextCol]
                if nextNode not in visited\
                and nextRow >= 0 and nextCol >= 0\
                and nextRow < rows and nextCol < cols \
                and not walls[nextRow][nextCol] \
                and dists[nextRow][nextCol] > maxDist:
                    # print(nextNode, visited)
                    maxDist = dists[nextNode[0]][nextNode[1]]
                    maxIdx = nextNode
                    maxDidx = i
                    fin_p = p
                    fin_curr = currChecking
            # if maxIdx != [-1, -1]:
                # break
        if maxIdx == [-1, -1]:
            print("broken here")
            break
        
        curr = maxIdx
        if fin_p > 0:
            print("backtracking to ", maxIdx, "from : ", fin_curr, "p : ", p)
            print("dix : ", maxDidx)
            drow = directions[maxDidx][0]
            dcol = directions[maxDidx][1]
            nextRow = fin_curr[0] + drow
            nextCol = fin_curr[1] + dcol
            nextNode = [nextRow, nextCol]
            print("next node : ", nextNode)
            print("len : ", len(pathIdx))
            # Shortest dist to maxIdx? 
            # backPath = []
            # for x in range(fin_p):
            #     opIdx = len(directions) - 1 -  path[-1 - x]
            #     backPath.append(opIdx)
            # backPaths.append(backPath)
            backNodes.append(fin_curr)
            backIdxs.append(len(path) - 1)
        path.append(maxDidx)
    finPath = []
    finIdxs = [start]
    j = 0
    for i in range(len(path)):
        finPath.append(path[i])
        finIdxs.append(pathIdx[i + 1])
        if j < len(backIdxs) and i == backIdxs[j]:
            togo = backNodes[j]
            currBack = finIdxs[-1]
            curridx = len(finPath) - 1
            op = len(directions) - 1 - finPath[curridx]
            finPath.append(op)
            curridx -= 1
            
            if i == 59:
                print("** ", togo, currBack, curridx)
            while curridx > 0 and currBack != togo:
                op = len(directions) - 1 - finPath[curridx]
                currBack = finIdxs[curridx + 1]
                finIdxs.append(currBack)
                finPath.append(op)
                curridx -= 1
            finPath = finPath[: -1]
            j += 1
            # if i == 59:
            #     print("** ", togo, curridx, finIdxs)
            if j >= len(backIdxs):
                finIdxs += pathIdx[i + 2 :]
                finPath += path[i + 1:]
                break
    finIdxs.append(goal)
        
    print("path idx: ", pathIdx)
    # print("path : ", path)
    print("back idxs: ", backIdxs)
    # print("fin paths: ", finPath)
    print("fin idxs: ", finIdxs)
    return finPath

def pathFromGraphOpt(dists, start, goal, directions, ops):
    visited = np.zeros((rows, cols))
    pathIdx = []    
    path = []
    backNodes = []
    backIdxs = []
    backVisited = []
    curr = start
    while curr != goal:
        pathIdx.append(curr)
        visited[curr[0]][curr[1]] = 1
        
        maxDist = -1
        maxDidx = -1
        maxIdx = [-1, -1]
        # print(curr.row, curr.col, curr.neighbors)
        fin_p = -1
        fin_curr = None
        for p in range(len(pathIdx)):
            currChecking = pathIdx[-1 - p]
            for i in range(len(directions)):
                drow = directions[i][0]
                dcol = directions[i][1]
                nextRow = currChecking[0] + drow
                nextCol = currChecking[1] + dcol
                nextNode = [nextRow, nextCol]
                if visited[nextRow][nextCol] == 0\
                and nextRow >= 0 and nextCol >= 0\
                and nextRow < rows and nextCol < cols \
                and not walls[nextRow][nextCol] \
                and dists[nextRow][nextCol] > maxDist:
                    # print(nextNode, visited)
                    maxDist = dists[nextNode[0]][nextNode[1]]
                    maxIdx = nextNode
                    maxDidx = i
                    fin_p = p
                    fin_curr = currChecking
            if maxIdx != [-1, -1]:
                break
        if maxIdx == [-1, -1]:
            print("broken here")
            break
        
        curr = maxIdx
        if fin_p > 0:
            print("backtracking to ", maxIdx, "from : ", fin_curr, "p : ", p)
            print("dix : ", maxDidx)
            drow = directions[maxDidx][0]
            dcol = directions[maxDidx][1]
            nextRow = fin_curr[0] + drow
            nextCol = fin_curr[1] + dcol
            nextNode = [nextRow, nextCol]
            print("next node : ", nextNode)
            print("len : ", len(pathIdx))
            # Shortest dist to maxIdx? 
            # backPath = []
            # for x in range(fin_p):
            #     opIdx = len(directions) - 1 -  path[-1 - x]
            #     backPath.append(opIdx)
            # backPaths.append(backPath)
            backNodes.append(fin_curr)
            backIdxs.append(len(path) - 1)
            backVisited.append(copy.deepcopy(visited))
        path.append(maxDidx)
    finPath = []
    finIdxs = [start]
    j = 0
    for i in range(len(path)):
        finPath.append(path[i])
        finIdxs.append(pathIdx[i + 1])
        if i == backIdxs[j]:
            togo = backNodes[j]
            currBack = finIdxs[-1]
            backPath = pathToBack(backVisited[j], currBack, togo, directions, ops)
            # curridx = len(finPath) - 1
            # op = len(directions) - 1 - finPath[curridx]
            # finPath.append(op)
            # curridx -= 1
            # while curridx > 0 and currBack != togo:
            #     op = len(directions) - 1 - finPath[curridx]
            #     currBack = finIdxs[curridx + 1]
            #     finIdxs.append(currBack)
            #     finPath.append(op)
            #     curridx -= 1
            finPath = finPath + backPath
            j += 1
            # if i == 59:
            #     print("** ", togo, curridx, finIdxs)
            if j >= len(backIdxs):
                finIdxs += pathIdx[i + 2 :]
                finPath += path[i + 1:]
                break
    finIdxs.append(goal)
        
    print("path idx: ", pathIdx)
    # print("path : ", path)
    print("back idxs: ", backIdxs)
    # print("fin paths: ", finPath)
    print("fin idxs: ", finIdxs)
    return finPath

def sortedInsert(queue, node, nodeValue):
    for i in range(len(queue)):
        curr = queue[i]
        if nodeValue[curr[0]][curr[1]] > nodeValue[node[0]][node[1]]:
            queue.insert(i, node)
            return
    queue.append(node)
    return

def pathToBack(visited, start, goal, directions, ops):
    print("Djikstra: ", visited, start, goal)
    graph = visited
    nodeValue = np.full((rows, cols), np.inf)
    directionsPath = np.full((rows, cols), -1)
    nodeValue[start[0]][start[1]] = 0
    queue = [start]
    while len(queue) > 0:
        curr = queue.pop(0)
        if curr == goal:
            break
        currVal = nodeValue[curr[0]][curr[1]]
        for i in range(len(directions)):
            drow = directions[i][0]
            dcol = directions[i][1]
            nextRow = curr[0] + drow
            nextCol = curr[1] + dcol
            nextNode = [nextRow, nextCol]
            if graph[nextRow][nextCol] == 1\
            and nextRow >= 0 and nextCol >= 0\
            and nextRow < rows and nextCol < cols:
                if nodeValue[nextRow][nextCol] > currVal + 1:
                    if nodeValue[nextRow][nextCol] < np.inf:
                        queue.remove(nextNode)
                    nodeValue[nextRow][nextCol] = currVal + 1
                    # directionsPath[nextRow][nextCol] = len(directions) - 1 - i
                    directionsPath[nextRow][nextCol] = ops[i]
                    sortedInsert(queue, nextNode, nodeValue)
    path = []
    curr = goal
    while curr != start:
        d = directionsPath[curr[0]][curr[1]]
        # path += [len(directions) - 1 - d]
        path += [ops[d]]
        curr = [curr[0] + directions[d][0], curr[1] + directions[d][1]]
    path.reverse()
    print("back path: ", path)
    return path
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
    
    # if start[0] <= goal[0] and start[1] < goal[1]:
    #     directions = [[1, 0], [0, 1], [-1, 0], [0, -1],
    #              [1, 1], [1, -1], [-1, 1], [-1, -1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # elif start[0] > goal[0] and start[1] <= goal[1]:
    #     directions = [[-1, 0], [0, 1], [1, 0], [0, -1],
    #              [-1, 1], [-1, -1], [1, 1], [1, -1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # elif start[0] < goal[0] and start[1] > goal[1]:
    #     directions = [[1, 0], [0, -1], [-1, 0], [0, 1],
    #              [1, -1], [-1, -1], [1, 1], [-1, 1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # else:
    #     directions = [[-1, 0], [0, -1], [0, 1], [1, 0],
    #              [-1, -1], [-1, 1], [1, -1], [1, 1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
        
    # if start[0] >= goal[0] and start[1] >= goal[1]:
    #     directions = [[1, 0], [0, 1], [-1, 0], [0, -1],
    #              [1, 1], [1, -1], [-1, 1], [-1, -1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # elif start[0] <= goal[0] and start[1] >= goal[1]:
    #     directions = [[-1, 0], [0, 1], [1, 0], [0, -1],
    #              [-1, 1], [-1, -1], [1, 1], [1, -1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # elif start[0] >= goal[0] and start[1] <= goal[1]:
    #     directions = [[1, 0], [0, -1], [-1, 0], [0, 1],
    #              [1, -1], [-1, -1], [1, 1], [-1, 1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
    # else:
    #     directions = [[-1, 0], [0, -1], [0, 1], [1, 0],
    #              [-1, -1], [-1, 1], [1, -1], [1, 1]]
    #     ops = [2, 3, 1, 0, 7, 6, 5, 4]
        
    dists = distTransform(goal, directions)
    print(dists)
    
    # Start with a uniform belief grid.
    snow = (1.0 - walls)
    # print(snow)
    
    # Initialize the figure.
    visual = Visualization(walls, robot, dists, goal)
    input("Next")
    
    # path = pathFromGraph(dists, start, goal)
    # print("Not Opt: ", len(path))
    
    
    # path = pathFromGraph(dists, start, goal)
    path = pathFromGraphOpt(dists, start, goal, directions, ops)
    print("Opt: ", len(path))
    # print(path)
    # print(bel)
    
    for p in path:
        # while True:
        #     key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
        #     if   (key == 'q'):  return
        #     else:
        #         break
        prevRobot = (robot.row, robot.col)
        (drow, dcol) = directions[p]
        # (drow, dcol) = p
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

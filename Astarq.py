from heapq import heappush, heappop
import math
import time
import random

class node:
    xPos = 0
    yPos = 0
    zPos = 0
    distance = 0
    priority = 0
    def __init__(self, xPos, yPos, zPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.zPos = zPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other):
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest, zDest):
        self.priority = self.distance + self.estimate(xDest, yDest, zDest) * 10 # A*

    def nextMove(self, dirs, d):
        if dirs == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10

    def estimate(self, xDest, yDest, zDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        zd = zDest - self.zPos

        d = math.sqrt(xd * xd + yd * yd + zd * zd)

        return(d)

# A-star algorithm.

def pathFind(the_map, n, m, v, dirs, dx, dy, dz, zA, zB, xA, yA, xB, yB):
    closed_nodes_map = []
    open_nodes_map = []
    dir_map = []
    row = []
    rowq = [0] * v
    for i in range(n):
        row.append(list(rowq))
    for i in range(m):
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], [], []]
    pqi = 0

    n0 = node(xA, yA, zA, 0, 0)
    n0.updatePriority(xB, yB, zB)
    heappush(pq[pqi], n0)
    open_nodes_map[yA][xA][zA] = n0.priority

    # A* search
    while len(pq[pqi]) > 0:

        n1 = pq[pqi][0]
        n0 = node(n1.xPos, n1.yPos, n1.zPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        z = n0.zPos
        heappop(pq[pqi])
        open_nodes_map[y][x][z] = 0
        closed_nodes_map[y][x][z] = 1

        if x == xB and y == yB and z == zB:

            path = ''
            while not (x == xA and y == yA and z == zA):
                j = dir_map[y][x][z]
                c = str((j + dirs / 2) % dirs)
                path = c + path
                x += dx[j]
                y += dy[j]
                z += dz[j]
            return path


        for i in range(dirs):
            xdx = x + dx[i]
            ydy = y + dy[i]
            zdz = z + dz[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1 or zdz < 0
                or zdz > v - 1 or the_map[ydy][xdx][zdz] == 1
                or closed_nodes_map[ydy][xdx][zdz] == 1):

                m0 = node(xdx, ydy,zdz, n0.distance, n0.priority)
                m0.nextMove(dirs, i)
                m0.updatePriority(xB, yB, zB)

                if open_nodes_map[ydy][xdx][zdz] == 0:
                    open_nodes_map[ydy][xdx][zdz] = m0.priority
                    heappush(pq[pqi], m0)

                    dir_map[ydy][xdx][zdz] = (i + dirs / 2) % dirs
                elif open_nodes_map[ydy][xdx][zdz] > m0.priority:

                    open_nodes_map[ydy][xdx][zdz] = m0.priority

                    dir_map[ydy][xdx][zdz] = (i + dirs / 2) % dirs


                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy
                    and pq[pqi][0].zPos == zdz):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi])
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0)
    return ''

# MAIN
dirs = 8
if dirs == 4:
    dx = [1, 0, -1, 0, 0, 1]
    dy = [0, 1, 0, -1, 0, 0]
    dz = [1, 0, -1, 0, 1, 0]
elif dirs == 8:
    dx = [1, 1, 0, -1, -1, -1, 0, 1]
    dy = [0, 1, 1, 1, 0, -1, -1, -1]
    dz = [-1, 1, -1, 0, 1, 0, 1, 0]

n = 30
m = 30
v = 30
the_map = []
row = []
rowq = [0] * v
for i in range(n):
    row.append(list(rowq))
for i in range(m):
    the_map.append(list(row))

for x in range(n / 8, n * 7 / 8):
    the_map[m / 2][x] = 1
for y in range(m / 8, m * 7 / 8):
    the_map[y][v / 2] = 1
for z in range(v / 8, v * 7 / 8):
    the_map[z][n / 2] = 1

sf = []
sf.append((0, 0, n - 1, m - 1, v / 2 - 1, v / 2))
sf.append((0, m - 1, n - 1, 0, 1, v - 1))
sf.append((n / 2 - 1, m / 2 - 1, n / 2 + 1, m / 2 + 1, 1, v / 2))
sf.append((n / 2 - 1, m / 2 + 1, n / 2 + 1, m / 2 - 1, 1, v - 1))
sf.append((n / 2 - 1, 0, n / 2 + 1, m - 1, 1, v / 2))
sf.append((n / 2 + 1, m - 1, n / 2 - 1, 0, 1, v - 1))
sf.append((0, m / 2 - 1, n - 1, m / 2 + 1, 1, v / 2))
sf.append((n - 1, m / 2 + 1, 0, m / 2 - 1, 1, v - 1))
(xA, yA, zA, xB, yB, zB) = random.choice(sf)

print 'Map size (X,Y,Z): ', n, m, v
print 'Start: ', xA, yA, zA
print 'Finish: ', xB, yB, zB
t = time.time()
route = pathFind(the_map, n, m, v, dirs, dx, dy, dz, xA, yA, zA, xB, yB, zB)
print 'Time to generate the route (seconds): ', time.time() - t
print 'Route:'
print route

if len(route) > 0:
    x = xA
    y = yA
    z = zA
    the_map[y][x][z] = 2
    for i in range(len(route)):
        j = int(route[i])
        x += dx[j]
        y += dy[j]
        z += dz[j]
        the_map[y][x][z] = 3
    the_map[y][x][z] = 4

    print

raw_input('Press Enter...')

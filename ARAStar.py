import time
import sys
import math
from File import *
from PriorityQueue import PriorityQueue


class ARAStar():

    def __init__(self, epsilon=5.0, time_limit=100000, de_epsilon=0.5):
        self.map = []
        self.mapSize = 0
        self.start = ()
        self.goal = ()
        self.minStep = 0
        self.minPath = []
        self.g_score = {}
        self.f_score = {}
        self.close_set = set()
        self.open_set = PriorityQueue()
        self.came_from = {}  # father
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.incons = PriorityQueue()

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def offerHeuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def trackingPath(self):
        data = []
        current = self.goal
        step = 1
        while current in self.came_from:
            data.append(current)
            step += 1
            current = self.came_from[current]
        data.append(self.start)
        data.reverse()
        return step, data

    def isValid(self, neighbor):
        return (0 <= neighbor[0] < len(self.map)) and \
               (0 <= neighbor[1] < len(self.map[0])) and \
               (self.map[neighbor[0]][neighbor[1]] == 0)

    def writeToFile(self, output_name):
        fout = open(output_name, "w")
        fout.write("%s\n" % str(self.minStep))
        if self.minStep != -1:
            for coor in self.minPath:
                fout.write(str(coor) + ' ')
            fout.write("\n")
            mapFile = self.map
            for i in range(self.size):
                for j in range(self.size):
                    if self.map[i][j] == 1:
                        mapFile[i][j] = 'o'
                    elif self.map[i][j] == 0:
                        mapFile[i][j] = '-'
            for i, j in self.minPath:
                mapFile[i][j] = 'x'
            mapFile[self.start[0]][self.start[1]] = 'S'
            mapFile[self.goal[0]][self.goal[1]] = 'G'
            for row in mapFile:
                for i in row:
                    fout.write("%s " % i)
                fout.write("\n")
        fout.close()

    def improvePath(self):
        while not self.open_set.empty() and \
                ((self.goal not in self.f_score) or self.f_score[self.goal] > self.f_score[self.start]):
            neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
            while not self.open_set.empty():
                current = self.open_set.get()
                if current == self.goal:
                    break
                self.close_set.add(current)
                path = []
                for direction in neighbors:
                    neighbor = (current[0] + direction[0], current[1] + direction[1])
                    tentative_g_score = self.g_score[current] + 1
                    if not self.isValid(neighbor):
                        continue
                    if neighbor in self.close_set and tentative_g_score >= self.g_score[neighbor]:
                        continue
                    if neighbor not in self.close_set or tentative_g_score < self.g_score[neighbor]:
                        self.came_from[neighbor] = current
                        self.g_score[neighbor] = tentative_g_score
                        self.f_score[neighbor] = tentative_g_score + self.epsilon * self.heuristic(neighbor, self.goal)
                        path.append(neighbor)
                        if neighbor not in self.close_set:
                            self.open_set.put(neighbor, self.f_score[neighbor])
                        else:
                            self.incons.put(neighbor, self.f_score[neighbor])
            return current, path

    def minimalF(self):
        tmp = float("inf")
        for i in self.incons.elements:
            tmp = min(tmp, self.g_score[i[1]] + self.heuristic(i[1], self.start))
        for i in self.open_set.elements:
            tmp = min(tmp, self.g_score[i[1]] + self.heuristic(i[1], self.start))
        return min(float(self.epsilon), self.g_score[self.goal] / tmp)

    def araStar(self):
        self.open_set.put(self.start, 0)
        self.g_score[self.start] = 0
        self.g_score[self.goal] = 1000000
        self.f_score[self.start] = self.g_score[self.start] + self.heuristic(self.start, self.goal)
        self.open_set.put(self.start, self.f_score[self.start])

        start_time = time.time()
        path = self.improvePath()
        o_epsilon = self.minimalF()
        while o_epsilon > 1 and (time.time() - start_time) <= self.time_limit / 1000:
            self.epsilon -= self.de_epsilon
            print(self.epsilon)
            self.open_set.add(self.incons)
            self.close_set = set()
            path = self.improvePath()
            o_epsilon = self.minimalF()
        path = []
        tmp = self.goal
        while (tmp != -1):  # Tracks the path.
            path.append(tmp)
            tmp = self.came_from[tmp]
        path = list(reversed(path))
        return self.g[self.goal] + 1, path
        # return self.trackingPath()

    def runARAStar(self, input_name='input1.txt', output_name='output1.txt', epsilon=5.0, time_limit=100000,
                   de_epsilon=0.5):
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.map, self.mapSize, self.start, self.goal = readFromFile(input_name)
        self.minStep, self.minPath = self.araStar()
        writeToFile(output_name, self.map, self.mapSize, self.start, self.goal, self.minStep, self.minPath)


if __name__ == "__main__":
    if len(sys.argv) == 5:
        input, output, epsilon, tmax = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
        findPath = ARAStar()
        findPath.runARAStar(input, output, epsilon, tmax)
    else:
        print('Parameter is incorrect')

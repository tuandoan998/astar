import time
import sys
from Heuristic import *
from File import *
from PriorityQueue import PriorityQueue


class ARAStar():

    def __init__(self, epsilon=5.0, time_limit=100000, de_epsilon=0.5):
        self.map = []
        self.mapSize = 0
        self.start = ()
        self.goal = ()
        self.minStep = -1
        self.minPath = []
        self.g_score = {}
        self.f_score = {}
        self.close_set = set()
        self.open_set = PriorityQueue()
        self.came_from = {}  # father
        self.incons = []
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon

    def trackingPath(self):
        data = []
        current = self.goal
        while current in self.came_from:
            data.append(current)
            current = self.came_from[current]
        data.append(self.start)
        data.reverse()
        self.minStep = len(data)
        return data

    def isValid(self, neighbor):
        return (0 <= neighbor[0] < len(self.map)) and (0 <= neighbor[1] < len(self.map[0])) and (
                    self.map[neighbor[0]][neighbor[1]] == 0)

    def improvePath(self):
        while not self.open_set.empty():
            f, current = self.open_set.get()
            if current == self.goal:
                path = self.trackingPath()
                return path
            self.close_set.add(current)
            neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
            for direction in neighbors:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if not self.isValid(neighbor):
                    continue
                if neighbor not in self.g_score.keys():
                    self.g_score[neighbor] = float('inf')
                if self.g_score[current] + 1 < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = self.g_score[current] + 1
                    self.f_score[neighbor] = self.g_score[neighbor] + self.epsilon * heuristic(neighbor, self.goal)
                    if neighbor not in self.close_set:
                        self.open_set.put(neighbor, self.f_score[neighbor])
                    else:
                        self.incons.append(neighbor)
        return []

    def minE(self):
        tmp = float('inf')
        for _, s in self.open_set.elements:
            tmp = min(tmp, self.g_score[s] + heuristic(s, self.start))
        for s in self.incons:
            tmp = min(tmp, self.g_score[s] + heuristic(s, self.start))
        return min(self.epsilon, self.g_score[self.goal] / tmp)

    def araStar(self):
        start_time = time.time()

        # Init
        self.g_score[self.start] = 0
        self.g_score[self.goal] = float('inf')
        self.f_score[self.start] = self.g_score[self.start] + self.epsilon * heuristic(self.start, self.goal)
        self.open_set.put(self.start, self.f_score[self.start])

        # Improve path. If path not found, return []; else continue find
        path = self.improvePath()
        if len(path) == 0:
            print("Path not found")
            return []
        else:
            print("Found a path")
            print(path)

        o_epsilon = self.minE()
        while o_epsilon > 1 and (time.time() - start_time) * 1000 <= self.time_limit:
            self.epsilon -= self.de_epsilon

            # Move point from incons to open_set and update the priorities for all s in open_set
            for s in self.incons:
                self.open_set.update(s, self.g_score[s] + self.epsilon * heuristic(s, self.goal))

            # Reset close_set and incons
            self.close_set = set()
            self.incons = []

            # Improve Path
            o_path = self.improvePath()
            print(o_path)
            if 0 < len(o_path) <= len(path):
                path = o_path

            o_epsilon = self.minE()

        return path

    def runARAStar(self, input_name='input1.txt', output_name='output1.txt', epsilon=5.0, time_limit=10000,
                   de_epsilon=0.5):
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.map, self.mapSize, self.start, self.goal = readFromFile(input_name)
        self.minPath = self.araStar()
        writeToFile(output_name, self.map, self.mapSize, self.start, self.goal, self.minStep, self.minPath)
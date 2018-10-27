import math
from PriorityQueue import PriorityQueue
from File import *
from GUI import *
from Heuristic import *


# A* search algorithm
class AStar:

    def __init__(self, input_name = 'input1.txt'):
        self.minStep = -1
        self.minPath = []
        self.g_score = {}
        self.f_score = {}
        self.close_set = set()
        self.open_set = PriorityQueue()
        self.came_from = {}  # father
        self.map, self.mapSize, self.start, self.goal = readFromFile(input_name)

    def getMapInformation(self):
        return self.map, self.mapSize, self.start, self.goal

    def getStart(self):
        return self.start

    def trackingPath(self):
        data = []
        current = tuple(self.goal)
        while current in self.came_from:
            data.append(current)
            current = self.came_from[current]
        data.append(self.start)
        data.reverse()
        self.minStep = len(data)
        return data

    def isValid(self, neighbor):
        map = self.map
        return (0 <= neighbor[0] < self.mapSize) and \
               (0 <= neighbor[1] < self.mapSize) and \
               (map[neighbor[0]][neighbor[1]] == 0)

    def aStar(self, gui):
        self.open_set.put(self.start, 0)
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.g_score[self.start] + heuristic(self.start, self.goal)
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]

        while not self.open_set.empty():
            _, current = self.open_set.get()
            if current == self.goal:
                return self.trackingPath()
            self.close_set.add(current)
            gui.updateMap(current, CURRENT_COLOR)
            for direction in neighbors:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if not self.isValid(neighbor):
                    continue
                if neighbor in self.close_set and self.g_score[current] + 1 >= self.g_score[neighbor]:
                    continue
                gui.updateMap(neighbor, NEIGHBOR_COLOR)
                if neighbor not in self.g_score or self.g_score[current] + 1< self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = self.g_score[current] + 1
                    self.f_score[neighbor] = self.g_score[neighbor] + heuristic(neighbor, self.goal)
                    self.open_set.put(neighbor, self.f_score[neighbor])
        return []

    def runAStar(self, gui, input_name = 'input1.txt', output_name = 'output1.txt'):
        global sum_delay
        sum_delay = 0

        START_TIME = time.clock()
        self.map, self.mapSize, self.start, self.goal = readFromFile(input_name)
        self.minPath = self.aStar(gui)
        writeToFile(output_name, self.map, self.mapSize, self.start, self.goal, self.minStep, self.minPath)

        if self.minStep == -1:
            Notification().alert("Notification", "Path not found!")
        else:
            gui.drawPath(self.minPath)
            Notification().alert("Time: ",
                                 repr(time.clock() - START_TIME - sum_delay) + "s\nStep: " + repr(self.minStep))


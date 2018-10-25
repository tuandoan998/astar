# -*- coding: utf-8 -*-
# @Author: Doan Quang Tuan - Le Hoang Sang

import pygame
import sys
from tkinter import messagebox
from tkinter import Tk
import math
import time
import heapq

item_width = 20
item_height = 20
DELAY_TIME = 0.005  # 0.1s
# color
NEIGHBOR_COLOR = (255, 255, 153)
WALL_COLOR = (0, 0, 0)
START_GOAL_COLOR = (255, 0, 0)
CURRENT_COLOR = (102, 102, 255)
BACKGROUND_COLOR = (204, 255, 229)
# variable
screen_width = 0
screen_height = 0
margin = 1


class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

    def add(self, other):
        for i in other.elements:
            self.elements.append(i[1])


class Map:

    def __init__(self):
        self.size = 0
        self.map = []
        self.start = ()
        self.goal = ()

    def readFromFile(self, file_name):
        fin = open(file_name, "r")
        self.size = int(fin.readline())
        s = fin.readline().strip("\n").split(" ")
        g = fin.readline().strip("\n").split(" ")
        self.start = (int(s[0]), int(s[1]))
        self.goal = (int(g[0]), int(g[1]))
        for i in range(self.size):
            tmp = [int(i) for i in fin.readline().strip("\n").split(" ")]
            self.map.append(tmp)
        fin.close()

    def writeToFile(self, file_name, step, path):
        fout = open(file_name, "w")
        fout.write("%s\n" % str(step))
        if step != -1:
            for coor in path:
                fout.write(str(coor) + ' ')
            fout.write("\n")
            mapFile = self.map
            for i in range(self.size):
                for j in range(self.size):
                    if self.map[i][j] == 1:
                        mapFile[i][j] = 'o'
                    elif self.map[i][j] == 0:
                        mapFile[i][j] = '-'
            for i, j in path:
                mapFile[i][j] = 'x'
            mapFile[self.start[0]][self.start[1]] = 'S'
            mapFile[self.goal[0]][self.goal[1]] = 'G'
            for row in mapFile:
                for i in row:
                    fout.write("%s " % i)
                fout.write("\n")
        fout.close()

    def getStart(self):
        return self.start

    def getGoal(self):
        return self.goal

    def getMap(self):
        return self.map

    def getSize(self):
        return self.size


class AStar:

    def __init__(self, file_input_name):
        self.map = Map()
        self.minStep = 0
        self.minPath = []
        self.g_score = {}
        self.f_score = {}
        self.close_set = set()
        self.open_set = PriorityQueue()
        self.came_from = {}  # father

        self.map.readFromFile(file_input_name)
        self.start = self.map.getStart()
        self.goal = self.map.getGoal()
        self.open_set.put(self.start, 0)
        self.g_score[self.start] = 0
        self.f_score[self.start] = self.g_score[self.start] + self.heuristic(self.start, self.goal)

    def getMapObject(self):
        return self.map

    def getStart(self):
        return self.start

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def trackingPath(self):
        data = []
        current = tuple(self.map.getGoal())
        step = 1
        while current in self.came_from:
            data.append(current)
            step += 1
            current = self.came_from[current]
        data.append(self.map.getStart())
        data.reverse()
        return step, data

    def offerHeuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def isValid(self, neighbor):
        map = self.map.getMap()
        return (0 <= neighbor[0] < len(map)) and \
               (0 <= neighbor[1] < len(map[0])) and \
               (map[neighbor[0]][neighbor[1]] == 0)

    def aStar(self, gui):
        neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
        while not self.open_set.empty():
            current = self.open_set.get()
            if current == self.goal:
                return self.trackingPath()
            self.close_set.add(current)
            gui.updateMap(current, CURRENT_COLOR)
            for direction in neighbors:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                tentative_g_score = self.g_score[current] + 1
                if not self.isValid(neighbor):
                    continue
                if neighbor in self.close_set and tentative_g_score >= self.g_score[neighbor]:
                    continue
                gui.updateMap(neighbor, NEIGHBOR_COLOR)
                if neighbor not in self.g_score or tentative_g_score < self.g_score[neighbor]:
                    self.came_from[neighbor] = current
                    self.g_score[neighbor] = tentative_g_score
                    self.f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    self.open_set.put(neighbor, self.f_score[neighbor])
        return -1, []

    def writeToFile(self, file_name):
        self.map.writeToFile(file_name, self.minStep, self.minPath)

    def runAStar(self, gui, input_name='input1.txt', output_name='output1.txt'):
        START_TIME = time.clock()
        self.map.readFromFile(input_name)
        self.minStep, self.minPath = self.aStar(gui)
        self.writeToFile(output_name)
        if self.minStep == -1:
            alert("Notification", "Path not found!")
        else:
            gui.drawPath(self.minPath)
            alert("Time: ",
                  repr(time.clock() - START_TIME - sum_delay) + "s\nStep: " + repr(self.minStep))


class ARAStar(AStar):

    def __init__(self, file_input_name='input1.txt', epsilon=5.0, time_limit=100000, de_epsilon=0.5):
        AStar.__init__(self, file_input_name)
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.incons = PriorityQueue()

    def improvePath(self, gui):
        while not self.open_set.empty() and \
                ((self.goal not in self.f_score) or self.f_score[self.goal] > self.f_score[self.start]):
            neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
            while not self.open_set.empty():
                current = self.open_set.get()
                if current == self.goal:
                    break
                self.close_set.add(current)
                gui.updateMap(current, CURRENT_COLOR)
                path = []
                for direction in neighbors:
                    neighbor = (current[0] + direction[0], current[1] + direction[1])
                    tentative_g_score = self.g_score[current] + 1
                    if not self.isValid(neighbor):
                        continue
                    if neighbor in self.close_set and tentative_g_score >= self.g_score[neighbor]:
                        continue
                    gui.updateMap(neighbor, NEIGHBOR_COLOR)
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

    def araStar(self, gui):
        start_time = time.time()
        self.improvePath(gui)
        o_epsilon = self.minimalF()
        while o_epsilon > 1 and (time.time() - start_time) <= self.time_limit / 1000:
            self.epsilon -= self.de_epsilon
            self.open_set.add(self.incons)
            self.close_set = {}
            self.improvePath(gui)
            o_epsilon = self.minimalF()
        return self.trackingPath()

    def runARAStar(self, gui, input_name='input1.txt', output_name='output1.txt', epsilon=5.0, time_limit=100000,
                   de_epsilon=0.5):
        START_TIME = time.clock()
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.map.readFromFile(input_name)
        self.minStep, self.minPath = self.araStar(gui)
        self.writeToFile(output_name)
        if self.minStep == -1:
            alert("Notification", "Path not found!")
        else:
            gui.drawPath(self.minPath)
            alert("Time: ",
                  repr(time.clock() - START_TIME - sum_delay) + "s\nStep: " + repr(self.minStep))


class GUI():
    def __init__(self, map, screen):
        self.map = map
        self.screen = screen

    def drawMap(self):
        size = self.map.getSize()
        matrix = self.map.getMap()
        root = self.map.getStart()
        des = self.map.getGoal()

        screen.fill(BACKGROUND_COLOR)

        x = 0
        while x < size:
            y = 0
            while y < size:
                if (matrix[x][y] == 1):
                    pygame.draw.rect(screen, WALL_COLOR,
                                     [(item_width + margin) * y + margin, (item_height + margin) * x + margin,
                                      item_width,
                                      item_height])
                y = y + 1
            x = x + 1
        pygame.draw.rect(screen, START_GOAL_COLOR,
                         [(item_width + margin) * root[1] + margin, (item_height + margin) * root[0] + margin,
                          item_width,
                          item_height])
        pygame.draw.rect(screen, START_GOAL_COLOR,
                         [(item_width + margin) * des[1] + margin, (item_height + margin) * des[0] + margin, item_width,
                          item_height])
        pygame.display.flip()

    def updateMap(self, pos, color):
        if start != pos:
            pygame.draw.rect(self.screen, color,
                             [(item_width + margin) * pos[1] + margin, (item_height + margin) * pos[0] + margin,
                              item_width,
                              item_height])
            pygame.display.update()
        time.sleep(DELAY_TIME)
        global sum_delay
        sum_delay = sum_delay + DELAY_TIME

    def drawPath(self, path):
        for pos in reversed(path):
            self.updateMap(pos, START_GOAL_COLOR)

    def wait(self):
        while True:
            event = pygame.event.poll()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()


def alert(title, content):
    Tk().wm_withdraw()
    messagebox.showinfo(title, content)


if __name__ == "__main__":
    if len(sys.argv) == 3 or len(sys.argv) == 5:
        if len(sys.argv) == 3:
            input, output = sys.argv[1], sys.argv[2]
            findPath = AStar(input)
            map = findPath.getMapObject()

            screen_width = item_width * map.getSize() + margin * map.getSize()
            screen_height = item_height * map.getSize() + margin * map.getSize()

            global sum_delay
            sum_delay = 0
            global start
            start = findPath.getStart()

            pygame.init()
            pygame.display.set_caption("A Star")
            screen = pygame.display.set_mode([screen_width, screen_height])
            gui = GUI(map, screen)
            gui.drawMap()
            alert("A Star algorithm", "Press OK to start")
            findPath.runAStar(gui, input, output)

            gui.wait()

        else:
            input, output, epsilon, tmax = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
            findPath = ARAStar(input, epsilon, tmax)
            map = findPath.getMapObject()

            screen_width = item_width * map.getSize() + margin * map.getSize()
            screen_height = item_height * map.getSize() + margin * map.getSize()

            sum_delay = 0
            start = findPath.getStart()

            pygame.init()
            pygame.display.set_caption("ARA Star")
            screen = pygame.display.set_mode([screen_width, screen_height])
            gui = GUI(map, screen)
            gui.drawMap()
            alert("ARA Star algorithm", "Press OK to start")
            findPath.runARAStar(gui, input, output, epsilon, tmax)

            gui.wait()

    else:
        alert("Error", "Parameter is incorrect")

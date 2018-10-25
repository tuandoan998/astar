# -*- coding: utf-8 -*-
# @Author: Doan Quang Tuan - Le Hoang Sang

import pygame
import sys
from tkinter import messagebox
from tkinter import Tk
import math
import time
import heapq

# color
NEIGHBOR_COLOR = (255, 255, 153)
WALL_COLOR = (0, 0, 0)
START_GOAL_COLOR = (255, 0, 0)
CURRENT_COLOR = (102, 102, 255)
BACKGROUND_COLOR = (204, 255, 229)

# variable
MARGIN = 1
ITEM_WIDTH = 20
ITEM_HEIGHT = 20

# time
DELAY_TIME = 0.005  # 0.1s


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

    def runAStar(self, gui, output_name='output1.txt'):
        global sum_delay
        sum_delay = 0

        START_TIME = time.clock()
        self.minStep, self.minPath = self.aStar(gui)

        self.writeToFile(output_name)

        if self.minStep == -1:
            Notification().alert("Notification", "Path not found!")
        else:
            gui.drawPath(self.minPath)
            Notification().alert("Time: ",
                                 repr(time.clock() - START_TIME - sum_delay) + "s\nStep: " + repr(self.minStep))


class ARAStar(AStar):

    def __init__(self, file_input_name='input1.txt', epsilon=5.0, time_limit=100000, de_epsilon=0.5):
        AStar.__init__(self, file_input_name)
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.incons = PriorityQueue()

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
        start_time = time.time()
        self.improvePath()
        o_epsilon = self.minimalF()
        while o_epsilon > 1 and (time.time() - start_time) <= self.time_limit / 1000:
            self.epsilon -= self.de_epsilon
            self.open_set.add(self.incons)
            self.close_set = {}
            self.improvePath()
            o_epsilon = self.minimalF()
        return self.trackingPath()

    def runARAStar(self, input_name='input1.txt', output_name='output1.txt', epsilon=5.0, time_limit=100000,
                   de_epsilon=0.5):
        START_TIME = time.clock()
        self.epsilon = epsilon
        self.time_limit = time_limit
        self.de_epsilon = de_epsilon
        self.map.readFromFile(input_name)
        self.minStep, self.minPath = self.araStar()
        self.writeToFile(output_name)
        if self.minStep == -1:
            Notification().alert("Notification", "Path not found!")
        else:
            gui.drawPath(self.minPath)
            Notification().alert("Time: ",
                                 repr(time.clock() - START_TIME - sum_delay) + "s\nStep: " + repr(self.minStep))


class GUI():
    def __init__(self, map, screen_width, screen_height, screen_caption):
        self.map = map
        self.start = map.getStart()
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.screen_caption = screen_caption
        pygame.init()
        pygame.display.set_caption(self.screen_caption)
        self.screen = pygame.display.set_mode([screen_width, screen_height])

    def drawMap(self):
        size = self.map.getSize()
        matrix = self.map.getMap()
        root = self.map.getStart()
        des = self.map.getGoal()

        self.screen.fill(BACKGROUND_COLOR)

        x = 0
        while x < size:
            y = 0
            while y < size:
                if (matrix[x][y] == 1):
                    pygame.draw.rect(self.screen, WALL_COLOR,
                                     [(ITEM_WIDTH + MARGIN) * y + MARGIN, (ITEM_HEIGHT + MARGIN) * x + MARGIN,
                                      ITEM_WIDTH,
                                      ITEM_HEIGHT])
                y = y + 1
            x = x + 1
        pygame.draw.rect(self.screen, START_GOAL_COLOR,
                         [(ITEM_WIDTH + MARGIN) * root[1] + MARGIN, (ITEM_HEIGHT + MARGIN) * root[0] + MARGIN,
                          ITEM_WIDTH,
                          ITEM_HEIGHT])
        pygame.draw.rect(self.screen, START_GOAL_COLOR,
                         [(ITEM_WIDTH + MARGIN) * des[1] + MARGIN, (ITEM_HEIGHT + MARGIN) * des[0] + MARGIN, ITEM_WIDTH,
                          ITEM_HEIGHT])
        pygame.display.flip()

    def updateMap(self, pos, color):
        if self.start != pos:
            pygame.draw.rect(self.screen, color,
                             [(ITEM_WIDTH + MARGIN) * pos[1] + MARGIN, (ITEM_HEIGHT + MARGIN) * pos[0] + MARGIN,
                              ITEM_WIDTH,
                              ITEM_HEIGHT])
            pygame.display.update()
        time.sleep(DELAY_TIME)
        global sum_delay
        sum_delay = sum_delay + DELAY_TIME

    def drawPath(self, path):
        for pos in reversed(path):
            self.updateMap(pos, START_GOAL_COLOR)

    def ready(self):
        Notification().alert(self.screen_caption, "Press OK to start")

    def wait(self):
        while True:
            event = pygame.event.poll()
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()


class Notification():
    def alert(self, title, content):
        Tk().wm_withdraw()
        messagebox.showinfo(title, content)

    def error(self, title, content):
        Tk().wm_withdraw()
        messagebox.showerror(title, content)


if __name__ == "__main__":
    if len(sys.argv) == 3 or len(sys.argv) == 5:
        if len(sys.argv) == 3:
            input, output = sys.argv[1], sys.argv[2]

            findPath = AStar(input)

            map = findPath.getMapObject()

            screen_width = ITEM_WIDTH * map.getSize() + MARGIN * map.getSize()
            screen_height = ITEM_HEIGHT * map.getSize() + MARGIN * map.getSize()

            gui = GUI(map, screen_width, screen_height, "A Star")
            gui.drawMap()
            gui.ready()

            findPath.runAStar(gui, output)

            gui.wait()

        else:
            input, output, epsilon, tmax = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
            findPath = ARAStar(input, epsilon, tmax)
            findPath.runARAStar(input, output, epsilon, tmax)

    else:
        Notification().error("Error", "Parameter is incorrect")

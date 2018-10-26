import pygame
from tkinter import messagebox
from tkinter import Tk
import time
import sys

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
sum_delay = 0

class GUI():
    def __init__(self, map, mapSize, start, goal, screen_width, screen_height, screen_caption):
        self.map = map
        self.mapSize = mapSize
        self.start = start
        self.goal = goal
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.screen_caption = screen_caption
        pygame.init()
        pygame.display.set_caption(self.screen_caption)
        self.screen = pygame.display.set_mode([screen_width, screen_height])

    def drawMap(self):
        size = self.mapSize
        matrix = self.map
        root = self.start
        des = self.goal

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
        global sum_delay
        if self.start != pos:
            pygame.draw.rect(self.screen, color,
                             [(ITEM_WIDTH + MARGIN) * pos[1] + MARGIN, (ITEM_HEIGHT + MARGIN) * pos[0] + MARGIN,
                              ITEM_WIDTH,
                              ITEM_HEIGHT])
            pygame.display.update()
        time.sleep(DELAY_TIME)
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

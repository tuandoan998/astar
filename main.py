# -*- coding: utf-8 -*-
# @Author: Doan Quang Tuan - Le Hoang Sang

from AStar import AStar
from GUI import *

if __name__ == "__main__":
    if len(sys.argv) == 3 or len(sys.argv) == 5:
        if len(sys.argv) == 3:
            input_name, output_name = sys.argv[1], sys.argv[2]

            findPath = AStar(input_name)
            map, mapSize, start, goal = findPath.getMapInformation()

            screen_width = ITEM_WIDTH * mapSize + MARGIN * mapSize
            screen_height = ITEM_HEIGHT * mapSize + MARGIN * mapSize

            gui = GUI(map, mapSize, start, goal, screen_width, screen_height, "A Star")
            gui.drawMap()
            gui.ready()

            findPath.runAStar(gui, input_name, output_name)

            gui.wait()

        else:
            input, output, epsilon, tmax = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])
            #findPath = ARAStar(input, epsilon, tmax)
            #findPath.runARAStar(input, output, epsilon, tmax)

    else:
        Notification().error("Error", "Parameter is incorrect")

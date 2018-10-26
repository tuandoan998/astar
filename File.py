def readFromFile(file_name):
    fin = open(file_name, "r")
    size = int(fin.readline())
    s = fin.readline().strip("\n").split(" ")
    g = fin.readline().strip("\n").split(" ")
    start = (int(s[0]), int(s[1]))
    goal = (int(g[0]), int(g[1]))
    map = []
    for i in range(size):
        tmp = [int(i) for i in fin.readline().strip("\n").split(" ")]
        map.append(tmp)
    fin.close()
    return map, size, start, goal

def writeToFile(file_name, map, mapSize, start, goal, step, path):
        fout = open(file_name, "w")
        fout.write("%s\n" % str(step))
        if step != -1:
            for coor in path:
                fout.write(str(coor) + ' ')
            fout.write("\n")
            mapFile = map
            for i in range(mapSize):
                for j in range(mapSize):
                    if map[i][j] == 1:
                        mapFile[i][j] = 'o'
                    elif map[i][j] == 0:
                        mapFile[i][j] = '-'
            for i, j in path:
                mapFile[i][j] = 'x'
            mapFile[start[0]][start[1]] = 'S'
            mapFile[goal[0]][goal[1]] = 'G'
            for row in mapFile:
                for i in row:
                    fout.write("%s " % i)
                fout.write("\n")
        fout.close()

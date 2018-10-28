import math


# Eclidean distance
def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def offerHeuristic(a, b):
    return math.sqrt(abs((b[0] - a[0]) ** 2 - (b[1] - a[1]) ** 2))

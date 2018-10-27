import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)

    def update(self, item, priority):
        for index, (p, i) in enumerate(self.elements):
            if i == item:
                if p <= priority:
                    break
                del self.elements[index]
                self.elements.append((priority, item))
                heapq.heapify(self.elements)
                break
        else:
            self.put(item, priority)

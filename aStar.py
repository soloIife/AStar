import numpy as np
import heapq


class Node:

    def __init__(self, coord, parent=None, g=0, h=0):
        self.coord = coord
        self.parent: Node = parent
        self.g = g
        self.h = h
        self.f = g + h
        self.master = 0
        # self.iter = self.gen_data()

    # def gen_data(self):
    #     cur_node = self
    #     while 1:
    #         if cur_node is None:
    #             break
    #         yield cur_node
    #         cur_node = cur_node.parent
    #
    # def __iter__(self):
    #     return self.iter

    def __gt__(self, other):
        return self.f > other.f

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.f == other.f

    def __str__(self):
        return "<(%s,%s) G: %s,H: %s,F: %s> " % (self.coord[0], self.coord[1], self.g, self.h, self.f)

    __repr__ = __str__


class Walker:
    def __init__(self, node):
        self.node = node
        self.iter = self.gen_data()

    def gen_data(self):
        cur_node = self.node
        while 1:
            if cur_node is None:
                break
            yield cur_node
            cur_node = cur_node.parent

    def __iter__(self):
        return self.iter


class AStar:
    def __init__(self, world_map, bar_value, direct_cost, oblique_cost):
        self.world_map = world_map
        self.bar_value = bar_value
        self.direct_cost = direct_cost
        self.oblique_cost = oblique_cost
        self.map_cache = np.zeros_like(world_map)

    def new_node(self, prev_node: Node, end_node, x, y, weight):
        coord = x, y
        height, width = self.world_map.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return
        color = self.world_map[y, x]
        if color == self.bar_value:
            return
        H = abs(end_node.coord[0] - x) * self.direct_cost + abs(end_node.coord[1] - y) * self.direct_cost
        G = prev_node.g + weight
        t_node = Node(coord, prev_node, G, H)
        return t_node

    def get_neighbors(self, p_node, end_node):
        coord = p_node.coord
        up = self.new_node(p_node, end_node, coord[0], coord[1] - 1, self.direct_cost)
        down = self.new_node(p_node, end_node, coord[0], coord[1] + 1, self.direct_cost)
        left = self.new_node(p_node, end_node, coord[0] - 1, coord[1], self.direct_cost)
        right = self.new_node(p_node, end_node, coord[0] + 1, coord[1], self.direct_cost)

        # return up, down, left, right
        left_up = self.new_node(p_node, end_node, coord[0] - 1, coord[1] - 1, self.oblique_cost)
        right_up = self.new_node(p_node, end_node, coord[0] + 1, coord[1] - 1, self.oblique_cost)
        left_down = self.new_node(p_node, end_node, coord[0] - 1, coord[1] + 1, self.oblique_cost)
        right_down = self.new_node(p_node, end_node, coord[0] + 1, coord[1] + 1, self.oblique_cost)

        return up, down, left, right, left_up, right_up, left_down, right_down

    def find_path(self, start_node, end_node):
        open_ls = []
        close_ls = []
        self.map_cache[:, :] = 0
        heapq.heappush(open_ls, start)
        self.map_cache[start_node.coord[1], start_node.coord[0]] = 1
        while 1:
            if len(open_ls) == 0:
                print("failed!")
                break
            cur_node: Node = heapq.heappop(open_ls)
            if cur_node.coord == end_node.coord:
                print("success")
                return cur_node
            for node in self.get_neighbors(cur_node, end_node):
                if node is None:
                    continue
                if self.map_cache[node.coord[1], node.coord[0]] != 0:
                    continue
                heapq.heappush(open_ls, node)
                self.map_cache[node.coord[1], node.coord[0]] = 1
            close_ls.append(cur_node)
            self.map_cache[cur_node.coord[1], cur_node.coord[0]] = 2


# #
if __name__ == '__main__':
    import time
    import cv2
    default_bar_value = 70
    default_set_path = 255
    DIRECT_WEIGHT = 10
    OBLIQUE_WEIGHT = 14
    # create a map
    maps = np.zeros((650, 750), np.intc) + 1
    maps[40:, 20:30] = default_bar_value
    maps[:400, 100:110] = default_bar_value
    maps[100:, 200:210] = default_bar_value
    maps[:200, 300:310] = default_bar_value
    maps[220:230, 210:710] = default_bar_value
    maps[220:600, 710:720] = default_bar_value
    maps[600:610, 300:720] = default_bar_value
    maps[300:610, 300:310] = default_bar_value

    start = Node((10, 10))  # start coord
    end = Node((600, 400))  # end coord
    finder = AStar(maps, default_bar_value, DIRECT_WEIGHT, OBLIQUE_WEIGHT)
    t0 = time.time()
    node = finder.find_path(start, end)
    print("耗时:", time.time()-t0)

    for node in Walker(node):
        maps[node.coord[1], node.coord[0]] = default_set_path

    maps = maps.astype(np.uint8)
    maps = maps.reshape((*maps.shape, 1))
    maps = maps.repeat(3, 2)
    cv2.circle(maps, tuple(start.coord), 5, (0, 255, 0), 5)
    cv2.circle(maps, tuple(end.coord), 5, (255, 0, 0), 5)
    maps[maps[:, :, 0] == default_set_path] = 50, 255, 50
    maps[maps[:, :, 0] == default_bar_value] = 0, 0, 255
    cv2.imshow("result", maps)
    cv2.waitKey()
    cv2.destroyAllWindows()


import heapq
import math
from collections import deque

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cell_of(x, y):
    return round(x / 2), round(y / 2)


class GridMap:
    resolution = 0.05
    origin = -18.0
    size = 720

    def __init__(self):
        self.evidence = np.zeros((self.size, self.size), np.float32)
        self.clearance = np.full_like(self.evidence, 8.0)
        self.graph = {(0, 0): set()}
        self.blocked_edges = set()

    def pixel(self, x, y):
        return (int(round((x - self.origin) / self.resolution)),
                int(round((y - self.origin) / self.resolution)))

    def world(self, point):
        return (point[0] * self.resolution + self.origin,
                point[1] * self.resolution + self.origin)

    def integrate(self, pose, angles, ranges, maximum):
        x, y, yaw = pose
        lx, ly = x + .09 * math.cos(yaw), y + .09 * math.sin(yaw)
        start = self.pixel(lx, ly)
        valid = ~np.isnan(ranges) & (ranges > .12)
        rays = np.minimum(ranges[valid], maximum - .05)
        theta = angles[valid] + yaw
        xs = lx + rays * np.cos(theta)
        ys = ly + rays * np.sin(theta)
        pixels = np.rint((np.column_stack((xs, ys)) - self.origin) / self.resolution).astype(int)
        free = np.zeros_like(self.evidence, np.uint8)
        hits = np.zeros_like(free)
        for p, r in zip(pixels, ranges[valid]):
            if not (0 <= p[0] < self.size and 0 <= p[1] < self.size):
                continue
            cv2.line(free, start, tuple(p), 1, 1)
            if math.isfinite(r) and r < maximum - .05:
                hits[p[1], p[0]] = 1
        self.evidence[free != 0] -= .35
        self.evidence[hits != 0] += 1.2
        cv2.circle(self.evidence, self.pixel(x, y), 3, -3.0, -1)
        np.clip(self.evidence, -4, 4, out=self.evidence)
        self.clearance = distance_transform_edt(self.evidence < .6) * self.resolution

    def update_graph(self):
        for a in list(self.graph):
            for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                b = a[0] + dx, a[1] + dy
                if max(abs(b[0]), abs(b[1])) > 7 or b in self.graph[a] or frozenset((a, b)) in self.blocked_edges:
                    continue
                for side in np.arange(-.6, .61, .1):
                    points = [self.pixel(2*a[0] + dx*(1+s) - dy*side,
                                         2*a[1] + dy*(1+s) + dx*side)
                              for s in (-.2, 0, .2)]
                    if all(self.evidence[py, px] < -.9 for px, py in points) and \
                            self.clearance[points[1][1], points[1][0]] >= .22:
                        self.graph[a].add(b)
                        self.graph.setdefault(b, set()).add(a)
                        break

    def block_edge(self, a, b):
        self.blocked_edges.add(frozenset((a, b)))
        self.graph.get(a, set()).discard(b)
        self.graph.get(b, set()).discard(a)

    def route(self, start, goals):
        queue = deque([start])
        previous = {start: None}
        while queue:
            a = queue.popleft()
            if a in goals:
                route = []
                while a is not None:
                    route.append(a)
                    a = previous[a]
                return route[::-1]
            for b in sorted(self.graph.get(a, ())):
                if b not in previous:
                    previous[b] = a
                    queue.append(b)
        return []

    def line_clear(self, a, b, radius=.215):
        a, b = np.asarray(a), np.asarray(b)
        count = max(2, int(np.linalg.norm(b-a) / self.resolution) + 1)
        points = np.rint((np.linspace(a, b, count) - self.origin) / self.resolution).astype(int)
        if np.any(points < 0) or np.any(points >= self.size):
            return False
        return bool(np.all(self.clearance[points[:, 1], points[:, 0]] >= radius))

    def cell_goal(self, cell, previous=None):
        if previous is not None:
            px, py = self.pixel(*previous)
            if self.clearance[py, px] >= .24:
                return previous
        center = np.array(cell, float)*2
        candidates = [center + (dx, dy) for dx in np.arange(-.6, .61, .1)
                      for dy in np.arange(-.6, .61, .1)]
        safe = [p for p in candidates if self.clearance[self.pixel(*p)[1], self.pixel(*p)[0]] > .32]
        reference = previous if previous is not None else center
        return min(safe, key=lambda p: float(np.linalg.norm(p-center) + .8*np.linalg.norm(p-reference))) if safe else center

    def plan(self, start, goal):
        start, goal = self.pixel(*start), self.pixel(*goal)
        margin = 20
        xmin, xmax = max(1, min(start[0], goal[0])-margin), min(self.size-2, max(start[0], goal[0])+margin)
        ymin, ymax = max(1, min(start[1], goal[1])-margin), min(self.size-2, max(start[1], goal[1])+margin)
        queue = [(0., start)]
        cost, previous = {start: 0.}, {start: None}
        steps = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
        while queue:
            _, a = heapq.heappop(queue)
            if a == goal:
                path = []
                while a is not None:
                    path.append(self.world(a))
                    a = previous[a]
                return path[::-1]
            for dx, dy in steps:
                b = a[0]+dx, a[1]+dy
                if not (xmin <= b[0] <= xmax and ymin <= b[1] <= ymax):
                    continue
                clearance = self.clearance[b[1], b[0]]
                if clearance < .215:
                    continue
                if dx and dy and min(self.clearance[a[1], b[0]], self.clearance[b[1], a[0]]) < .215:
                    continue
                weight = math.hypot(dx, dy) * (1 + .18/max(.05, clearance-.17))
                if self.evidence[b[1], b[0]] == 0:
                    weight += .3
                candidate = cost[a] + weight
                if candidate < cost.get(b, math.inf):
                    cost[b], previous[b] = candidate, a
                    heapq.heappush(queue, (candidate + math.hypot(b[0]-goal[0], b[1]-goal[1]), b))
        return []

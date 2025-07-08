import os
import pygame
import random
import math
import sys
import numpy as np
import imageio
from datetime import datetime

# ---------- Configuration ----------
WIDTH, HEIGHT = 800, 600
GOAL_RADIUS = 20
STEP_SIZE = 20
MAX_ITER = 5000
GOAL_BIAS = 0.05
NEIGHBOR_RADIUS = 50
DRAW_COLOR = (0, 0, 0)     # obstacles
TREE_COLOR = (0, 150, 0)   # tree edges
PATH_COLOR = (255, 0, 0)   # final path
START_COLOR = (0, 0, 255)
GOAL_COLOR = (255, 0, 0)
FPS = 30                   # slower viz
OUTPUT_DIR = "output"
# -----------------------------------

class Node:
    def __init__(self, x, y, parent=None):
        self.x, self.y = x, y
        self.parent = parent
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, obstacles, screen, clock):
        self.start = Node(*start)
        self.goal  = Node(*goal)
        self.tree  = [self.start]
        self.obstacles = obstacles
        self.screen = screen
        self.clock = clock
        self.frames = []  # for GIF

        # ensure output directory exists
        os.makedirs(OUTPUT_DIR, exist_ok=True)

    def sample(self):
        if random.random() < GOAL_BIAS:
            return self.goal.x, self.goal.y
        return random.uniform(0, WIDTH), random.uniform(0, HEIGHT)

    def nearest(self, x, y):
        return min(self.tree, key=lambda n: (n.x - x)**2 + (n.y - y)**2)

    def steer(self, from_node, to_point):
        theta = math.atan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
        return Node(
            from_node.x + STEP_SIZE * math.cos(theta),
            from_node.y + STEP_SIZE * math.sin(theta)
        )

    def collision_free(self, n1, n2):
        dx, dy = n2.x - n1.x, n2.y - n1.y
        dist = math.hypot(dx, dy)
        steps = max(int(dist / 5), 1)
        for i in range(steps + 1):
            x = n1.x + dx * (i / steps)
            y = n1.y + dy * (i / steps)
            if any(obs.collidepoint(x, y) for obs in self.obstacles):
                return False
        return True

    def get_neighbors(self, new_node):
        return [
            n for n in self.tree
            if math.hypot(n.x - new_node.x, n.y - new_node.y) <= NEIGHBOR_RADIUS
        ]

    def choose_parent(self, neighbors, new_node):
        best = neighbors[0]
        min_cost = best.cost + math.hypot(best.x-new_node.x, best.y-new_node.y)
        for n in neighbors[1:]:
            cost = n.cost + math.hypot(n.x-new_node.x, n.y-new_node.y)
            if cost < min_cost and self.collision_free(n, new_node):
                best, min_cost = n, cost
        new_node.parent = best
        new_node.cost = min_cost

    def rewire(self, neighbors, new_node):
        for n in neighbors:
            edge_cost = math.hypot(n.x-new_node.x, n.y-new_node.y)
            new_cost = new_node.cost + edge_cost
            if new_cost < n.cost and self.collision_free(new_node, n):
                n.parent = new_node
                n.cost = new_cost

    def build(self):
        for _ in range(MAX_ITER):
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    pygame.quit(); sys.exit()

            x_rand, y_rand = self.sample()
            nearest = self.nearest(x_rand, y_rand)
            new_node = self.steer(nearest, (x_rand, y_rand))

            if not self.collision_free(nearest, new_node):
                continue

            neighbors = self.get_neighbors(new_node)
            if neighbors:
                self.choose_parent(neighbors, new_node)
                self.tree.append(new_node)
                self.rewire(neighbors, new_node)
            else:
                new_node.parent = nearest
                new_node.cost = nearest.cost + STEP_SIZE
                self.tree.append(new_node)

            # goal check
            if math.hypot(new_node.x-self.goal.x, new_node.y-self.goal.y) < GOAL_RADIUS:
                self.goal.parent = new_node
                self.goal.cost = new_node.cost + math.hypot(new_node.x-self.goal.x, new_node.y-self.goal.y)
                self.tree.append(self.goal)
                break

            self.draw()
            self.clock.tick(FPS)

        # final display + GIF
        self.draw()
        path = self.extract_path()
        self.draw_path(path)
        self.save_gif()

    def draw(self):
        self.screen.fill((255,255,255))
        for obs in self.obstacles:
            pygame.draw.rect(self.screen, DRAW_COLOR, obs)
        for node in self.tree:
            if node.parent:
                pygame.draw.line(self.screen, TREE_COLOR,
                                 (node.x, node.y),
                                 (node.parent.x, node.parent.y), 1)
        # persistent start & goal
        pygame.draw.circle(self.screen, START_COLOR,
                           (int(self.start.x), int(self.start.y)), 10)
        pygame.draw.circle(self.screen, GOAL_COLOR,
                           (int(self.goal.x), int(self.goal.y)), GOAL_RADIUS, 2)
        pygame.display.update()

        arr = pygame.surfarray.array3d(self.screen)
        frame = np.transpose(arr, (1, 0, 2))
        self.frames.append(frame)

    def extract_path(self):
        path, node = [], self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return list(reversed(path))

    def draw_path(self, path):
        for i in range(len(path)-1):
            pygame.draw.line(self.screen, PATH_COLOR,
                             path[i], path[i+1], 3)
        pygame.display.update()
        # hold on-screen for ~1s
        for _ in range(FPS):
            arr = pygame.surfarray.array3d(self.screen)
            self.frames.append(np.transpose(arr, (1, 0, 2)))

    def save_gif(self):
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rrt_star_path_{now}.gif"
        path = os.path.join(OUTPUT_DIR, filename)
        imageio.mimsave(path, self.frames, fps=FPS)
        print(f"Saved GIF to {path}")

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(
        "RRT* — Click START, then GOAL, then draw obstacles, SPACE to run"
    )
    clock = pygame.time.Clock()

    start = goal = None
    obstacles = []
    drawing = False
    drag_start = None

    # Phase 1: select start & goal
    while not (start and goal):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                if not start:
                    start = ev.pos
                elif not goal:
                    goal = ev.pos
        screen.fill((255,255,255))
        if start:
            pygame.draw.circle(screen, START_COLOR, start, 10)
        if goal:
            pygame.draw.circle(screen, GOAL_COLOR, goal, GOAL_RADIUS, 2)
        pygame.display.update()
        clock.tick(FPS)

    # Phase 2: draw obstacles
    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                drawing = True
                drag_start = ev.pos
            if ev.type == pygame.MOUSEBUTTONUP and ev.button == 1 and drawing:
                x1,y1 = drag_start; x2,y2 = ev.pos
                rect = pygame.Rect(
                    min(x1,x2), min(y1,y2),
                    abs(x2-x1), abs(y2-y1)
                )
                obstacles.append(rect)
                drawing = False
            if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                planner = RRTStar(start, goal, obstacles, screen, clock)
                planner.build()
                # hold open until quit
                while True:
                    for e in pygame.event.get():
                        if e.type == pygame.QUIT:
                            pygame.quit(); sys.exit()

        screen.fill((255,255,255))
        for obs in obstacles:
            pygame.draw.rect(screen, DRAW_COLOR, obs)
        if drawing and drag_start:
            x1,y1 = drag_start; x2,y2 = pygame.mouse.get_pos()
            preview = pygame.Rect(min(x1,x2), min(y1,y2),
                                  abs(x2-x1), abs(y2-y1))
            pygame.draw.rect(screen, DRAW_COLOR, preview, 1)
        pygame.draw.circle(screen, START_COLOR, start, 10)
        pygame.draw.circle(screen, GOAL_COLOR, goal, GOAL_RADIUS, 2)
        pygame.display.update()
        clock.tick(FPS)

if __name__ == "__main__":
    main()

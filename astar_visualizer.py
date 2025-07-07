import asyncio
import platform
import pygame
import heapq
import math
import imageio
import numpy as np
from datetime import datetime

# Constants
WIDTH = 800
ROWS = 50
FPS = 120

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
PURPLE = (128, 0, 128)

# Initialize Pygame
pygame.init()
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Planning Visualizer")

gif_frames = []  # to store frames for GIF

class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.came_from = None

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.color == RED

    def is_open(self):
        return self.color == GREEN

    def is_barrier(self):
        return self.color == BLACK

    def is_start(self):
        return self.color == BLUE

    def is_end(self):
        return self.color == RED

    def reset(self):
        self.color = WHITE
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.came_from = None

    def make_closed(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def make_barrier(self):
        self.color = BLACK

    def make_start(self):
        self.color = BLUE

    def make_end(self):
        self.color = RED

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),     # N, S, W, E
            (-1, -1), (-1, 1), (1, -1), (1, 1)    # NW, NE, SW, SE
        ]
        for dr, dc in directions:
            r, c = self.row + dr, self.col + dc
            if 0 <= r < self.total_rows and 0 <= c < self.total_rows:
                if not grid[r][c].is_barrier():
                    self.neighbors.append(grid[r][c])

def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)
    return grid

def draw_grid(win, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(win, BLACK, (0, i * gap), (width, i * gap))
        for j in range(rows):
            pygame.draw.line(win, BLACK, (j * gap, 0), (j * gap, width))

def draw(win, grid, rows, width, capture=False):
    win.fill(WHITE)
    for row in grid:
        for node in row:
            node.draw(win)
    draw_grid(win, rows, width)
    pygame.display.update()

    if capture:
        frame = pygame.surfarray.array3d(win)
        frame = np.transpose(frame, (1, 0, 2))  # (W,H,3) → (H,W,3)
        gif_frames.append(frame)

def heuristic(a, b):
    return abs(a.row - b.row) + abs(a.col - b.col)

def astar(grid, start, end):
    count = 0
    open_set = []
    heapq.heappush(open_set, (0, count, start))
    start.g_cost = 0
    start.h_cost = heuristic(start, end)
    start.f_cost = start.g_cost + start.h_cost

    open_set_hash = {start}

    while open_set:
        current = heapq.heappop(open_set)[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(current)
            draw(WIN, grid, ROWS, WIDTH, capture=True)
            save_gif()
            return True

        for neighbor in current.neighbors:
            temp_g_cost = current.g_cost + 1
            if temp_g_cost < neighbor.g_cost:
                neighbor.came_from = current
                neighbor.g_cost = temp_g_cost
                neighbor.h_cost = heuristic(neighbor, end)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                if neighbor not in open_set_hash:
                    count += 1
                    heapq.heappush(open_set, (neighbor.f_cost, count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        draw(WIN, grid, ROWS, WIDTH, capture=True)

        if current != start:
            current.make_closed()

    save_gif()  # save even if path not found
    return False

def reconstruct_path(current):
    while current.came_from:
        current = current.came_from
        current.make_path()
        draw(WIN, grid, ROWS, WIDTH, capture=True)

def save_gif():
    if not gif_frames:
        return
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"astar_path_{timestamp}.gif"
    imageio.mimsave("output/"+filename, gif_frames, fps=10)  # Use fps instead of duration
    print(f"Saved: {filename}")
    gif_frames.clear()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos
    row = y // gap
    col = x // gap
    return row, col

def setup():
    global grid, start, end
    grid = make_grid(ROWS, WIDTH)
    start = None
    end = None

def update_loop():
    global grid, start, end
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit

        if pygame.mouse.get_pressed()[0]:  # Left click
            pos = pygame.mouse.get_pos()
            row, col = get_clicked_pos(pos, ROWS, WIDTH)
            node = grid[row][col]
            if not start and node != end:
                start = node
                start.make_start()
            elif not end and node != start:
                end = node
                end.make_end()
            elif node != start and node != end:
                node.make_barrier()

        elif pygame.mouse.get_pressed()[2]:  # Right click
            pos = pygame.mouse.get_pos()
            row, col = get_clicked_pos(pos, ROWS, WIDTH)
            node = grid[row][col]
            node.reset()
            if node == start:
                start = None
            elif node == end:
                end = None

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE and start and end:
                for row in grid:
                    for node in row:
                        node.update_neighbors(grid)
                astar(grid, start, end)

            if event.key == pygame.K_c:
                start = None
                end = None
                grid = make_grid(ROWS, WIDTH)

    draw(WIN, grid, ROWS, WIDTH)

async def main():
    setup()
    while True:
        update_loop()
        await asyncio.sleep(1.0 / FPS)

if platform.system() == "Emscripten":
    asyncio.ensure_future(main())
else:
    if __name__ == "__main__":
        asyncio.run(main())

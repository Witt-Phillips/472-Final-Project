### note - not for submission! Some of this code is copied from astar.py (youtube video). It should not make it to the final repo.


import pygame
import math
import random
import numpy as np
from queue import PriorityQueue

# Display
WIDTH = 800
INIT_WINDOW = pygame.display.set_mode((WIDTH, WIDTH))


pygame.display.set_caption("Pathfinding")
GRAY = (128, 128, 128)

type_map = {
    'none'   : (255, 255, 255),
    'start'  : (255, 0, 0),
    'end'    : (128, 0, 128),
    'open'   : (0, 0, 255),
    'closed' : (255, 255, 0),
    'barrier': (0, 0, 0),
    'path'   : (0, 255, 0),
}

#print(type_map['start'])

class Cell:
    #width is the width of the cell here
    def __init__(self, row, col, width, dim):
        # 
        self.row = row
        self.col = col
        
        # Proxy for GPS coords
        self.x = row * width
        self.y = col * width
        
        self.width = width
        self.dim = dim

        #State of cell info (isClosed())
        self.type = 'none'

        #Graph info
        self.neighbors = []
    
    def get_coords(self):
        return self.row, self.col
    
    def reset(self):
        self.type = 'none'
    
    def set_open(self):
        self.type = 'open'
    
    def set_closed(self):
        self.type = 'closed'
    
    def set_barrier(self):
        self.type = 'barrier'
    
    def set_start(self):    
        self.type = 'start'
    
    def set_end(self):
        self.type = 'end'
    
    def draw(self, window):
        pygame.draw.rect(window, type_map[self.type], (self.x, self.y, self.width, self.width))
    
    def update_neighbors(self, map):
        pass

    def comp(self, other):
        return False

# Manhattan distance between two points
def h(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def init_map(dim, width):
    ## WILL BE 0.1 IN OUR EXAMPLE
    cell_width = width // dim
    map = np.empty((dim, dim), dtype=object)
    for i in range(dim):
        for j in range(dim):
            cell = Cell(i , j, cell_width, dim)
            map[i, j] = cell
    #INITIALIZE FROM ZOMBIE AND WALL LIST
    return map

# #Print test map
# test_map = init_map(100, 10)
# get_type = np.vectorize(lambda x: x.type)
# print(get_type(test_map))

def draw_map(window, dim, width):
    cell_width = width // dim
    for i in range(dim):
        pygame.draw.line(window, GRAY, (0, i * cell_width), (width, i * cell_width))
        for j in range(dim):
            pygame.draw.line(window, GRAY, (j * cell_width, 0), (j * cell_width, width))

def draw(window, grid, dim, width):
    window.fill(type_map['none'])

    for row in grid:
        for spot in row:
            spot.draw(window)
    draw_map(window, dim, width)
    pygame.display.update()


grid = init_map(10, 1)
draw(INIT_WINDOW, grid, 10, 1)

def get_clicked_pos(pos, dim, width):
    cell_width = width // dim
    y, x = pos
    row = y // cell_width
    col = x // cell_width
    return row, col

def main(window, width, dim):
    start = None
    end = None
    
    run = True
    started = False
    
    while run:
        draw(INIT_WINDOW, grid, dim, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if started:
                continue
            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, dim, width)
                spot = grid[row, col]
                
                if not start:
                    start = spot
                    start.set_start()

                elif not end:
                    end = spot
                    end.set_end()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    #started = True
                    pass
                
    pygame.quit()

# %% Simualate:
dim  = 100
#Randomly generate zombies and positions
zombie_colors = [
    'green',
    'blue',
    'aqua',
    'purple'
]
zombie_arr = []

class Zombie:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color

num_zombies = 10
for zombie in range(num_zombies):
    zombie = Zombie(random.randint(0, dim - 1), random.randint(0, dim - 1), random.choice(zombie_colors))
    zombie_arr.append(zombie)

# Generate zombie list working!
#print(zombie_arr[0].color)

#Randomly generate walls.
class Wall:
    def __init__(self, x, y):
        self.x = x
        self.y = y
wall_arr = []
num_walls = 100
for wall in range(num_walls):
    wall = Wall(random.randint(0, dim - 1), random.randint(0, dim - 1))
    wall_arr.append(wall)

#Print Wall Workign
#print(wall_arr)


grid = init_map(dim, WIDTH)
print(grid)

#Parse zombie/ wall into barrier cells
for zombie in zombie_arr:
    x = zombie.x
    y = zombie.y
    # Make this range dependent on zombie type!
    for i in range(x - 5, x + 5):
        for j in range (y -5, y + 5):
            if i < dim - 1 and j < dim - 1:
                grid[i, j].set_barrier()

for wall in wall_arr:
    x = wall.x
    y = wall.y
    grid[x, y].set_barrier()

main(INIT_WINDOW, WIDTH, dim)

    

    

        
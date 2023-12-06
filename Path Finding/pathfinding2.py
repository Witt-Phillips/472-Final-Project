import math
import random
import numpy as np

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
#Dim 
dim = 50


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

#print(zombie_arr)
occupancy_matrix = [[1 for j in range(dim)] for i in range(dim)]

for wall in wall_arr:
    x = wall.x
    y = wall.y
    occupancy_matrix[x][y] = 0

for zombie in zombie_arr:
    x = zombie.x
    y = zombie.y
    # Make this range dependent on zombie type & have # cells scale with distance!
    for i in range(x - 5, x + 5):
        for j in range (y - 5, y + 5):
            if i < dim - 1 and j < dim - 1:
                occupancy_matrix[i][j] = 0

#Display occupancy
for row in occupancy_matrix:
    print(row)

grid = Grid(matrix=occupancy_matrix)
# Make this intake youbot position
start = grid.node(0, 0)
#Get position of target berry
end = grid.node(40, 20)

finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)

print('operations:', runs, 'path length:', len(path))
print(grid.grid_str(path=path, start=start, end=end))
#print(path)

#print(vars(path[1]))

#if dist is optimal (say we try top three)
path_to_take = [(obj.x, obj.x) for obj in path]
print(path_to_take)
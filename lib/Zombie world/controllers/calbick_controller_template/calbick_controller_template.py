"""youbot_controller controller."""

import sys, os

webots_home = '/Applications/WeBots.app'  # Replace with your actual path
root_dir = os.path.join(webots_home, 'lib', 'controller')

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor
from youbot_zombie import *

# %%
# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# Addditional packages/ constants---------------------------------------------------------------
import numpy as np
import pandas as pd
import cv2
import math
import matplotlib.pyplot as plt
from math import sin, cos, atan
import random
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# Define Global Variables
_2pi = 2 * math.pi
grid_width_default = 0.1
plotMap = True
CELL_WIDTH = 0.1

#Old Class Definitions
# %%  Define Classes

#New Classes
rows = ["red", "pink", "orange", "yellow"]
cols = ["p20E", "m20E", "p40H", "arm"]
weight_names = ["health", "energy", "dist"]
initial_posterior = np.array([
        [.75, 0, .25, 0],
        [0, .25, .75, 0],
        [0, .75, 0, .25],
        [.25, 0, 0, .75]
    ])
weights = np.array(([[0.5, 0.5, 0.5],
                     [0.5, 0.5, 0.5],
                     [0.5, 0.5, 0.5],
                     [0.5, 0.5, 0.5]]))
zombie_colors = [
    'green',
    'blue',
    'aqua',
    'purple'
]
base_zombie_ranges = {
    # vision radius for blue zombies?
    "green": 2,
    "blue": 2,
    "aqua": 3,
    "purple": 2,
}

class worldMapObject:
    def __init__(self, youbot=None, grid_cell_width=grid_width_default):
        self.cell_width = grid_cell_width  # in meters
        self.youbot = youbot  # keep info on youtbot
        self.timestep = None  # initialize placeholder for world timestep in simulation
        self.cell_object_table = {}  # hash table for "map_cell":object_list_in_cell (e.g. '[1,0]':[berry1 berry2 zombie1] )
        self.world_zombie_list = []
        self.world_berry_list = []
        self.world_solid_list = []
        self.world_object_list = self.world_zombie_list + self.world_berry_list + self.world_solid_list
        self.cell_properties_list = []

        # Init GPS (to base map on)
        self.init_gps = ()

        # Berry Probability
        self.prior = to_df(np.ones((4, 4)) / 4)
        self.count = to_df(np.zeros((4, 4)))
        self.draws = np.empty((0, 2))
        self.weights = pd.DataFrame(weights, index=cols, columns=weight_names)


class baseObject():
    def __init__(self, map, gps_xy=None, typeid=None, origin_xy=None):
        if gps_xy is None:
            gps_xy = [None, None]
        self.map = map
        self.typeid = typeid
        self.object_id = id(self)
        self.cell_idx = None
        self.cell_hash = None
        self.gps_xy = gps_xy
        self.map_rc = self.hash_gps_to_map()
        self.origin_xy = origin_xy
        # WP Added dist to youbout - check!
        if self.map.youbot is not None:
            if self.map.youbot.gps_xy is not None and self.gps_xy is not None:
                dist = distance(self.map.youbot.gps_xy, self.gps_xy)
            else:
                dist = None
        else:
            dist = None
        self.dist2youbot = dist
        self.angle2youbot = None

    def hash_gps_to_map(self):
        if self.gps_xy[1] is not None:
            map_rc = convert_gps_to_map(self.gps_xy, self.map)
            # Update object worldMap if necessary - ERROR - cannot be part of initialization
            # if self.map_rc != map_rc:
            #     self.update_cell_table(map_rc)
            return map_rc
        else:
            return None

    def update_cell_table(self, cell_rc):
        # This function is called only when new_map_rc and old_map_rc don't match
        hash_str = cell_rc.__str__()
        new_rc = cell_rc
        old_rc = self.map_rc

        self.cell_hash = hash_str

        # Assign object to cell dictionary with hash string
        if self.map.cell_object_table.get(hash_str) is None:
            # If the hash does not exist this is the first object in that cell
            # Initialize the list at this hash with this object
            self.map.cell_object_table[hash_str] = [self]
        else:  # Append the list at this hash with this object
            self.map.cell_object_table[hash_str].append(self)

        if old_rc is None:
            return

        # Remove this object from cell dictionary with old hash string
        self.map.cell_object_table[old_rc.__str__()].remove(self)

# Changed structure, added observed effect
class berryObject(baseObject):
    def __init__(self, map, dist=None, berry_color=None, effect_type=None, gps_xy=None):
        super().__init__(map, gps_xy, dist, 'berry')
        self.color = berry_color
        self.effect = effect_type
        self.dist2youbot = dist
        self.priority = self.priority_score()
        self.gps_xy = gps_xy
        self.map_rc = self.hash_gps_to_map() if gps_xy is not None else None
        map.world_berry_list.append(self)

    def observe_effect(self, obs_effect):
        self.effect = obs_effect
        prior = self.map.prior.values
        count = self.map.count.values

        draw = np.array([rows.index(self.color), cols.index(self.effect)])
        self.map.draws = np.vstack((self.map.draws, draw))
        count[draw[0], draw[1]] += 1
        prior = update_prior(draw, prior)

        self.map.prior = to_df(prior)
        self.map.count = to_df(count)
        self.map.world_berry_list = sorted(self.map.world_berry_list, key=lambda x: x.priority, reverse=True)

    def priority_score(self):
        # Get effect1 (likely primary) and effect 2 (likely secondary), and their probabilities
        seen_array = self.map.count.loc[self.color, :] != 0

        effect_1 = self.map.prior.iloc[rows.index(self.color)].idxmax()
        prob_effect_1 = self.map.prior.at[self.color, effect_1]
        seen_array[effect_1] = False

        effect_2 = seen_array.idxmax()
        if seen_array.max() == 0:
            effect_2 = second_largest_column(self.map.prior.iloc[rows.index(self.color)])
            # print("effect 2 defaulted:", effect_2)
        prob_effect_2 = self.map.prior.at[self.color, effect_2]

        health = self.map.youbot.robot_info[0]
        energy = self.map.youbot.robot_info[1]
        dist = distance(self.gps_xy, self.map.youbot.gps_xy)

        # priority score by effect
        score_1 = priority_score_formula(self.map.weights.at[effect_1, "health"],
                                         self.map.weights.at[effect_1, "energy"],
                                         self.map.weights.at[effect_1, "dist"],
                                         health, energy, dist)
        score_2 = priority_score_formula(self.map.weights.at[effect_2, "health"],
                                         self.map.weights.at[effect_2, "energy"],
                                         self.map.weights.at[effect_2, "dist"],
                                         health, energy, dist)

        # replace pro_effect_2 with 1 - prob_effect_1 (should really factor in all possible effects,
        # but will be small and don't really have time...)
        weighted_average = (prob_effect_1 * score_1) + (prob_effect_2 * score_2)
        return weighted_average


class youbotObject(baseObject):
    def __init__(self, map, wb_robot=None, sensors=None, wheels=None, gps_xy=None, init_gps_xy=None):
        super().__init__(map, gps_xy, 'youbot')
        self.wb_robot = wb_robot
        #WP Added Robot Info parameter
        self.robot_info = None
        self.sensors = sensors
        self.wheels = wheels
        self.init_gps = init_gps_xy
        self.bearing = None
        self.map_rc = self.hash_gps_to_map() if gps_xy is not None else None
        map.youbot = self

class zombieObject(baseObject):
    def __init__(self, map, zombie_color=None, gps_xy=None, typeid='zombie'):
        super().__init__(map, gps_xy, typeid)
        self.color = zombie_color
        self.chasing = False
        self.map_rc = self.hash_gps_to_map() if gps_xy is not None else None
        if gps_xy is not None:
            self.bearing = distance(map.youbot.gps_xy, self.gps_xy)
            self.distance = map.youbot.gps_xy
        else:
            self.bearing = None
            self.distance = None

        map.world_zombie_list.append(self)

#Addded inherit from base object  - that OK?
class wallObject(baseObject):
    def __init__(self, map, wall_color='brown', gps_xy=None, typeid='wall', moveable=False):
        super().__init__(map, gps_xy, typeid)
        self.moveable = moveable
        self.color = "green" if self.moveable else "brown"
        self.map_rc = self.hash_gps_to_map() if gps_xy is not None else None
        map.world_solid_list.append(self)


class cellObject():
    def __init__(self, map, friction=None, danger_level=None):
        super().__init__(map, None, 'cell')
        self.friction = friction  # scaler value based on slippage of wheels and/or youbot speed for == acceleration
        self.danger_level = danger_level  # based on zombies we think are around it and our historical health score change when we are in this cell
        map.cell_properties_list.append(self)


# %%  Define Functions

def getObjectRGB(object):
    color_map = {
        "zombie": "purple", "berry": "red",
        "purple_zombie": "purple", "green_zombie": "green",
        "aqua_zombie": "aqua", "blue_zombie": "blue",
        "pink_berry": "pink", "orange_berry": "orange",
        "red_berry": "red", "yellow_berry": "yellow",
        "brown": "gray"
    }

    if object.color is None:
        color_id = object.typeid
    else:
        color_id = color_map['_'.join([object.typeid, object.color])]

    return color_id


def angle2object(youbot, object):
    xdif = youbot.gps_xy[0] - object.gps_xy[0]
    ydif = youbot.gps_xy[1] - object.gps_xy[1]
    # Add pi / 2 to angle?
    angle = math.atan2(ydif, xdif) - youbot.bearing + (math.pi / 2)
    factor = abs(angle // (2 * math.pi))
    if factor != 0:
        angle /= factor
    if angle < 0:
        angle = angle + (2 * math.pi)
    #print("Angle to obj:", angle * (180 / math.pi))
    return angle


def path_to_object(youbot, object):

    angle = object.angle2youbot

    if object.dist2youbot < .2:
        print("Berry reached!")
        youbot.wheels["front_right"].setVelocity(0.0)
        youbot.wheels["front_left"].setVelocity(0.0)
        youbot.wheels["back_right"].setVelocity(0.0)
        youbot.wheels["back_left"].setVelocity(0.0)
    elif angle < to_rad(10):
        youbot.wheels["front_right"].setVelocity(8.0)
        youbot.wheels["front_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(8.0)
        youbot.wheels["back_left"].setVelocity(8.0)
    elif angle > math.pi:
        youbot.wheels["front_right"].setVelocity(8.0)
        youbot.wheels["front_left"].setVelocity(-8.0)
        youbot.wheels["back_right"].setVelocity(8.0)
        youbot.wheels["back_left"].setVelocity(-8.0)
    else:
        youbot.wheels["front_right"].setVelocity(-8.0)
        youbot.wheels["front_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(-8.0)
        youbot.wheels["back_left"].setVelocity(8.0)

########## Utility functions

def multiply(list, multiplier):
    return [x * multiplier for x in list]


def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def convert_gps_to_map(gps_xy, map):
    # Transform from world coordinates to map coordinates
    # x, y are world coordinates
    # return x, y in map coordinates
    return [(a - b) // map.cell_width for a, b in zip(gps_xy, map.init_gps)]


# Check this
def convert_map_to_gps(idx, map):
    return [(a * map.cell_width) + b for a, b in zip(idx, map.init_gps)]


def get_comp_angle(compass_values):
    angle = math.atan2(compass_values[1], compass_values[0]) - (math.pi / 2)
    if angle < 0:
        angle = angle + (2 * math.pi)
    return angle


def map_lidar(map, beam_number, magnitude):
    # map_lidar(self, beamNum, dist, ori)
    # for i in range(len(lidar_values)):
    #         mainMap.map_lidar(i, lidar_values[i], orientation)

    # Normalize angle
    theta = ((_2pi / 512) * beam_number) + map.youbot.orientation
    theta = theta - (_2pi * (theta > _2pi))  # subtract by 2𝜋 if  𝜃 > 2𝜋

    # Find coords & map
    gps_xy = multiply([cos(theta), sin(theta)], magnitude)

    # Figure out if there is an object marked close to that position
    objects   = map.world_object_list
    positions = [obj.gps_xy for obj in objects]

    assign_object(map, gps_xy)

    return theta,gps_xy


def assign_object(map, gps_xy):
    pass


# -------------- Plotting Functions ------------
def plot_init(map):
    ubot = map.youbot
    zombies = map.world_zombie_list
    berries = map.world_berry_list
    solids = map.world_solid_list

    # Initialize plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.grid()

    tags = ['_'.join([obj.typeid, obj.color]) for obj in map.cell_object_list]
    unique_tags = np.unique(tags)

    # Lambda function to filter objects by typeid
    filter_by_typeid = lambda typeid: [obj for obj in map.cell_object_list if obj.typeid == typeid]
    filter_by_typeid = lambda typeid: [obj for obj in map.cell_object_list if obj.typeid == typeid]
    # Creating the dictionary
    groot_tags = {tag: filter_by_typeid(tag) for tag in unique_tags}

    # # Adding additional keys if needed
    # groot_tags["youbot"] = []
    # groot_tags["visit_map"] = some_visit_map_value
    #
    # object_list = map.cell_object_list
    #
    # max_cell  = max([max(obj.map_rc) for obj in object_list])
    # max_grid  = max_cell + 0.1
    # grid_range = np.arange(-max_grid, max_grid, 1)
    # ax.set_xticks(grid_range)
    # ax.set_yticks(grid_range)
    # ax.grid(True, which='both', color='black', linewidth=1,)
    #
    # ax.set_xlim(-max_grid - 0.5, max_grid + 0.5)
    # ax.set_ylim(-max_grid - 0.5, max_grid + 0.5)
    #
    # # Draw gridlines and squares
    # for x in np.arange(-max_cell, max_cell + 1):
    #     for y in np.arange(-max_cell, max_cell + 1):
    #         cell_key = f"({x}, {y})"
    #         if cell_key in map_instance.cellTable:
    #             cell = map_instance.cellTable[cell_key]
    #             color = 'green' if cell.visited else 'black'
    #         else:
    #             color = 'white'
    #         ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color=color, edgecolor='black'))
    #
    # # Plot cells
    # for cell in map_instance.cellTable.values():
    #     color = 'green' if (cell.xPos, cell.yPos) == current_pos else 'black' if cell.visited else 'white'
    #     ax.add_patch(plt.Rectangle((cell.xPos - 0.5, cell.yPos - 0.5), 1, 1, color=color))
    #
    # # Scatter different objects
    # object_colors = {
    #     "purple_zombie": "purple", "green_zombie": "green", "aqua_zombie": "aqua",
    #     "blue_zombie": "blue", "pink_berry": "pink", "orange_berry": "orange",
    #     "red_berry": "red", "yellow_berry": "yellow", "wall": "gray", "self": "black"
    # }
    # for cell in map_instance.cellTable.values():
    #     if cell.typeid:
    #         obj_type = cell.typeid
    #         ax.scatter(cell.xPos, cell.yPos, color=object_colors[obj_type], s=30)
    #
    # plt.draw()
    # plt.pause(0.1)

    return fig, ax


def update_plot(map, fig, ax):
    new_objects = map.world_object_list
    for obj in new_objects:
        x, y, c = obj.gps_xy[1], obj.gps_xy[2], getObjectRGB(obj)

    # robot position
    for obj in new_objects():
        if obj.typeid:
            obj_type = obj.typeid
            ax.scatter(obj.xPos, obj.yPos, color=obj[obj_type], s=30)

    ax.scatter(x, y, c='r', marker='o')
    fig.canvas.draw()
    fig.canvas.flush_events()

# Probability Utility Functions

def to_rad(deg):
    return deg * (math.pi / 180)


def second_largest_column(row):
    return sorted(row.index, key=row.get, reverse=True)[1]


# Priority Score

def priority_score_formula(w_h, w_e, w_d, h, e, d):
    health_score = score(w_h, h)
    energy_score = score(w_e, e)
    dist_score = score(w_d, 5 * d)
    return (health_score + energy_score + dist_score) / 30000


def score(weight, value):
    return weight * ((100 - value) ** 2)


def random_posterior():
    shuffled_matrix = initial_posterior.copy()
    np.random.shuffle(shuffled_matrix)  # Shuffle rows
    np.random.shuffle(shuffled_matrix.T)  # Shuffle columns
    return shuffled_matrix


def drawfromposterior(posterior):
    color = np.random.randint(4)
    effects = np.random.rand()
    c = posterior[color, :]
    e = (c == .75) if effects > .25 else (c == .25)
    return np.array([color, np.where(e)[0][0]])


def calculate_likelihoods(observed_color, observed_effect, prior):
    likelihoods = np.zeros_like(prior)
    for color in range(prior.shape[0]):
        for effect in range(prior.shape[1]):
            if color == observed_color:
                likelihoods[color, effect] = 0.75 if effect == observed_effect else 0.25
            else:
                likelihoods[color, effect] = 0.1 if effect == observed_effect else 0.5
    likelihoods /= np.sum(likelihoods)
    return likelihoods


def update_prior(obs, prior):
    likelihoods = calculate_likelihoods(obs[0], obs[1], prior)
    updated_prior = prior * likelihoods
    updated_prior /= np.sum(updated_prior, axis=1, keepdims=True)  # Normalize rows
    updated_prior /= np.sum(updated_prior, axis=0, keepdims=True)  # Normalize columns
    return updated_prior


def to_df(matrix):
    return pd.DataFrame(matrix, index=rows, columns=cols)


########### World Initiation Functions ###############
def init_youbot(map):
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # health, energy, armour in that order
    robot_info = [100, 100, 0]
    passive_wait(0.1, robot, timestep)

    # Sensor initialization
    gps = robot.getDevice("gps")
    gps.enable(timestep)

    compass = robot.getDevice("compass")
    compass.enable(timestep)

    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)

    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)

    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # Wheel initialization
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")

    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))

    fr.setVelocity(0.0)
    fl.setVelocity(0.0)
    br.setVelocity(0.0)
    bl.setVelocity(0.0)

    sensors = {
        "gps": gps,
        "lidar": lidar,
        "light": lightSensor,
        "camera": camera8,
        "compass": compass
    }

    wheels = {
        "front_right": fr,
        "front_left": fl,
        "back_right": br,
        "back_left": bl,
    }

    youbot = youbotObject(map)
    youbot.wb_robot = robot
    youbot.robot_info = robot_info
    youbot.sensors = sensors
    youbot.wheels = wheels

    map.timestep = timestep

    return youbot


########### Main Function ###############
def main(simparams=None):
    # Initialize main map & establish relative center from GPS
    world_map = worldMapObject()

    # Initialize youbot in world with sensors
    youbot = init_youbot(world_map)
    robot = youbot.wb_robot

    # Run get all berry positions from controllers/youbot_controllers/youbot_zombie.py
    get_all_berry_pos(robot)

    # Initialize plot
    if plotMap:
        fig, ax = plot_init(world_map)

    robot_not_dead = 1
    timestep = world_map.timestep
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")

    youbot.init_gps = round(youbot.sensors["gps"].getValues()[0, 2], 3)

    # Temp variable to track coord change across time steps
    temp_coords = ""

    # Sensor Control Loop
    while robot.step(TIME_STEP) != -1:

        # Get sensor data
        gps_values = youbot.sensors["gps"].getValues()
        lidar_values = youbot.sensors["lidar"].getRangeImage()
        compass_values = youbot.sensors["compass"].getValues()

        # Update youbot orientation and gps position
        youbot.orientation = get_comp_angle(compass_values)
        youbot.gps_xy = round(gps_values[0, 2], 3)

        # Lidar Mapping
        for i in range(len(lidar_values)):
            if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
                object_list = world_map.world_object_list

                # This could be made more efficient if we limit search to only hash coordinates within youbot (r, 𝜃)
                gpsvals, = [object.gps_xy for object in object_list]

                # mainMap.map_lidar(i, lidar_values[i], orientation)

        # if plotMap:
        #     new_objects = get_objects_to_update(world_map)
        #
        #     update_plot(map, new_objects, ax)


def simulate_main(nzombies=3, nberries=10, ntimesteps=100):
    # %%
    zombie_colors = ("purple", "green", "blue", "aqua")
    berry_colors = ("pink", "orange", "red", "yellow")

    map = worldMapObject()

    # Init youbot
    youbot = youbotObject(map, gps_xy=[0, 0], init_gps_xy=[0, 0])

    zombies = [zombieObject(map, zombie_color=zombie_colors[random.randint(0, 3)]) for x in range(nzombies)]
    berries = [berryObject(map, berry_color=berry_colors[random.randint(0, 3)]) for x in range(nberries)]

    # Init Plot and plot initial positions
    ax = plot_init(map)

    # Run simulation and plot
    for t in range(ntimesteps):
        # Update youbot position
        pass
        # Update zombie positions


# %% Start simple simulation

def simple_sim():
    # Initialize main map & establish relative center from GPS
    world_map = worldMapObject()

    # Initialize youbot in world with sensors
    youbot = init_youbot(world_map)
    robot = youbot.wb_robot

    # Run get all berry positions from controllers/youbot_controllers/youbot_zombie.py
    get_all_berry_pos(robot)

    # Initialize plot
    if plotMap:
        fig, ax = plot_init(world_map)

    robot_not_dead = 1
    timestep = world_map.timestep
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")

    youbot.init_gps = round(youbot.sensors["gps"].getValues()[0, 2], 3)

    # Temp variable to track coord change across time steps
    temp_coords = ""

    # Sensor Control Loop
    while robot.step(TIME_STEP) != -1:

        # Get sensor data
        gps_values = youbot.sensors["gps"].getValues()
        lidar_values = youbot.sensors["lidar"].getRangeImage()
        compass_values = youbot.sensors["compass"].getValues()

        # Update youbot orientation and gps position
        youbot.orientation = get_comp_angle(compass_values)
        youbot.gps_xy = round(gps_values[0, 2], 3)

        # Lidar Mapping
        for i in range(len(lidar_values)):
            if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
                object_list = world_map.world_object_list

                # This could be made more efficient if we limit search to only hash coordinates within youbot (r, 𝜃)
                gpsvals, = [object.gps_xy for object in object_list]


def start_sim():
    # Put into helper function init_youbot

    # ------------------CHANGE CODE BELOW HERE ONLY--------------------------

    # Initialize main map & establish relative center from GPS
    world_map = worldMapObject()

    # Initialize youbot in world with sensors
    youbot = init_youbot(world_map)
    robot = youbot.wb_robot
    timestep = world_map.timestep

    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0

    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")

    get_all_berry_pos(robot)

    robot_not_dead = 1

    # Run get all berry positions from controllers/youbot_controllers/youbot_zombie.py
    get_all_berry_pos(robot)

    # Sensor Control Loop
    count = 0
    # while robot.step(TIME_STEP) != -1:
    for i in range(10):
        print(robot.step(TIME_STEP))
        image = youbot.sensors["camera"].getImage()
        width = youbot.sensors["camera"].getWidth()
        height = youbot.sensors["camera"].getHeight()

        # # Used ChatGPT to clarify syntax
        # np_u    = np.frombuffer(image, dtype=np.uint8)
        # np_img  = np_u.reshape(height, width, 4)
        # rgb_img = np_img[:, :, :3]
        # # hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
        #
        # cv2.imshow("Back Camera", rgb_img)
        # cv2.waitKey(100)

        youbot.wheels["font_right"].setVelocity(6.0)
        youbot.wheels["font_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(6.0)
        youbot.wheels["back_left"].setVelocity(8.0)

    # ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

    while (robot_not_dead == 1):

        if (robot_info[0] < 0):
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            # if(zombieTest):
            #    print("TEST PASSED")
            # else:
            #    print("TEST FAILED")
            # robot.simulationQuit(20)
            # exit()

        if (timer % 2 == 0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)

        if (timer % 16 == 0):
            robot_info = update_robot(robot_info)
            timer = 0

        if (robot.step(timestep) == -1):
            exit()

        timer += 1

    # ------------------CHANGE CODE BELOW HERE ONLY--------------------------

    # ------------------CHANGE CODE ABOVE HERE ONLY--------------------------

    return 0


# %% ----------- Sandbox -----------

# Initialize map
world_map = worldMapObject()

# Initialize youbot in world with sensors
youbot   = init_youbot(world_map)
robot    = youbot.wb_robot
timestep = world_map.timestep

# Stuff that they put in I believe
passive_wait(0.1, robot, timestep)
pc = 0
timer = 0

# They also use these to update information in the simulation
robot_node = robot.getFromDef("Youbot")
trans_field = robot_node.getField("translation")

# Loads the zombie and berry controllers for the world
get_all_berry_pos(robot)

# %% Run time step by time step
# NOTE: must run once after moving in Webots manually to get the current sensor readings!

tmp = robot.step(TIME_STEP)
world_map.init_gps = [youbot.sensors["gps"].getValues()[0],youbot.sensors["gps"].getValues()[2]]


#%%
# ------------- Individual Sandboxes ---------------
# This allows us to write longer codeblocks that won't be run upon syncing with webots

def pullFrame(youbot):
    camera = youbot.sensors["camera"]
    width = camera.getWidth()
    height = camera.getHeight()

    np_u = np.frombuffer(camera.getImage(), dtype=np.uint8)
    np_img = np_u.reshape(height, width, 4)
    np_img = np_img[:, :, ::-1]

    image = np_img[:, :, 1:]
    h_scale = 2
    sv_scale = 100 / 255

    return image, cv2.cvtColor(image, cv2.COLOR_RGB2HSV) * [h_scale, sv_scale, sv_scale]

def initplot(isolated_regions):
    # Assuming 'isolated_regions' is your dictionary of images
    num_colors = len(isolated_regions)

    # Create a figure and subplots
    fig, ax = plt.subplots(3, 4, figsize=(15, 8))

    # Initialize subplots for isolated regions
    mask_ax = {}
    for i, color in enumerate(isolated_regions, 1):
        row = 1 * (i > 4)
        col = (i - 1) % (num_colors // 2)
        mask_ax[color] = ax[row, col]
        ax[row, col].imshow(np.zeros_like(list(isolated_regions.values())[0]))  # Placeholder image
        ax[row, col].set_title(color)
        ax[row, col].axis('off')

    # Initialize subplots for RGB channels and original image
    rgb_ax = []
    for i in range(3):
        rgb_ax.append(ax[2, i].imshow(np.zeros_like(isolated_regions['blue'][:, :, 0]), cmap='gray'))
        ax[2, i].set_title(['Red', 'Green', 'Blue'][i])
        ax[2, i].axis('off')

    original_image = ax[2, -1]
    imag_ax = original_image.imshow(np.zeros_like(isolated_regions['blue']))
    original_image.set_title('Image')
    original_image.axis('off')

    plt.tight_layout()
    plt.show(block=False)
    return fig, mask_ax, rgb_ax, imag_ax

def processImageBerry(ims,lidar):
    pass
def processImageZombie(ims,lidar):
    pass
def processImageSoldid(ims,lidar):
    pass
def lidarDetect(map):
    pass
def analyzeColor(map,plts=None):

    # Explore Back Camera Image Processing
    rgb, hsv = pullFrame(map.youbot)

    # Define saturation and value ranges
    saturation_range = [30, 100]
    value_range = [0, 100]

    hue_ranges = {
        "red1": [0, 5],
        "red2": [340, 360],
        "orange": [10, 36],
        "yellow": [41, 69],
        "green": [69, 140],
        "aqua": [155, 195],
        "blue": [200, 250],
        "purple": [250, 290],
        "pink": [299, 333],
    }

    # Create color ranges in HSV
    color_ranges = {
        color: [np.array([hue_low] + [saturation_range[0]] + [value_range[0]]),
                np.array([hue_high] + [saturation_range[1]] + [value_range[1]])]
        for color, (hue_low, hue_high) in hue_ranges.items()
    }

    masks = {}
    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, lower, upper)
        masks[color] = mask

    masks["red"] = cv2.bitwise_or(masks["red1"], masks["red2"])
    del(masks["red1"])
    del (masks["red2"])

    isolated_regions = {}
    for color, mask in masks.items():
        isolated_region = cv2.bitwise_and(rgb, rgb, mask=mask)
        isolated_regions[color] = isolated_region

    # berry_colors  = ("red","yellow","pink","orange","purple")
    # zombie_colors = ("green","blue","aqua","purple")
    #
    # berries = processImageBerry(map,isolated_regions[berry_colors])
    # zombies = processImageZombie(map,isolated_regions[zombie_colors])

    plotStuff = True
    if plotStuff:
        if plts is None:
            plt.ion()
            plts = initplot(isolated_regions)
            return plts
        else:
            fig,mask_ax,rgb_ax,imag_ax = plts[0],plts[1] ,plts[2],plts[3]
            # Update each color region image
            for color, img in isolated_regions.items():
                mask_ax[color].images[0].set_data(img)

            # Update RGB channel images
            for i, channel_img in enumerate([rgb[:, :, i] for i in range(3)]):
                rgb_ax[i].set_data(channel_img)

            # Update original image
            imag_ax.set_data(rgb)
            # Redraw the updated plots
            fig.canvas.draw()
            fig.canvas.flush_events()


def isolateCameraRegions(map):
    # Explore Back Camera Image Processing
    rgb, hsv = pullFrame(map.youbot)

    # Define saturation and value ranges
    saturation_range = [30, 100]
    value_range = [0, 100]

    hue_ranges = {
        "red1": [0, 5],
        "red2": [340, 360],
        "orange": [10, 36],
        "yellow": [41, 69],
        "green": [69, 140],
        "aqua": [155, 195],
        "blue": [200, 250],
        "purple": [250, 290],
        "pink": [299, 333],
    }

    # Create color ranges in HSV
    color_ranges = {
        color: [np.array([hue_low] + [saturation_range[0]] + [value_range[0]]),
                np.array([hue_high] + [saturation_range[1]] + [value_range[1]])]
        for color, (hue_low, hue_high) in hue_ranges.items()
    }

    masks = {}
    for color, (lower, upper) in color_ranges.items():
        mask = cv2.inRange(hsv, lower, upper)
        masks[color] = mask

    masks["red"] = cv2.bitwise_or(masks["red1"], masks["red2"])
    del (masks["red1"])
    del (masks["red2"])

    isolated_regions = {}
    for color, mask in masks.items():
        isolated_region = cv2.bitwise_and(rgb, rgb, mask=mask)
        isolated_regions[color] = isolated_region

    return isolated_regions

def analyzeScene(map):

    camera_masks = isolateCameraRegions(map)

    berry_colors = ("red", "yellow", "pink", "orange", "purple")
    zombie_colors = ("green", "blue", "aqua", "purple")

    berries = processImageBerry(map, isolated_regions[berry_colors])
    zombies = processImageZombie(map, isolated_regions[zombie_colors])

def lidar2image(map):
#%%
    tmp = robot.step(TIME_STEP)
    lidar_values = map.youbot.sensors["lidar"].getRangeImage()

    for i in range(len(lidar_values)):
        if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
            print(lidar_values[i])
    pass

def sandbox_dc():
# %% Sandbox for Dan
# Create a figure with subplots
    plts = analyzeColor(world_map)
#%%
    tmp = robot.step(TIME_STEP)
    analyzeColor(world_map,plts)

#%% assess object
    lidar_objects = lidar2image(world_map)

    # If lidar is picking up objects in visible region of world map
    if lidar_objects is not None:
    # analy
        analyzeScene(world_map)


def gps_to_occupancy(gps_xy, min_x, min_y, cell_width):
    x, y = [int((a - b) // cell_width) for a, b in zip(gps_xy, (min_x, min_y))]
    return x, y


def occupancy_to_gps(idx, min_x, min_y, cell_width):
    return [(a * cell_width) + b for a, b in zip(idx, (min_x, min_y))]

def list_min_max_gps(lst):
    min_max = {
        'min_x': min(lst, key=lambda obj: obj.gps_xy[0]).gps_xy[0],
        'max_x': max(lst, key=lambda obj: obj.gps_xy[0]).gps_xy[0],
        'min_y': min(lst, key=lambda obj: obj.gps_xy[1]).gps_xy[1],
        'max_y': max(lst, key=lambda obj: obj.gps_xy[1]).gps_xy[1]
    }
    return min_max


def build_occupancy_grid(map):
    # Bounds - replace these with parameters.
    min_max_gps = list_min_max_gps(map.world_object_list)
    min_x = min_max_gps["min_x"]
    max_x = min_max_gps["max_x"]
    min_y = min_max_gps["min_y"]
    max_y = min_max_gps["max_y"]

    # Basic map info
    cell_width = map.cell_width
    map_width = int(((max_x - min_x) // cell_width) + 1)
    map_height = int(((max_y - min_y) // cell_width) + 1)

    occupancy_matrix = [[1 for i in range(map_height)] for j in range(map_width)]

    # POPULATE OCCUPANCY MATRIX
    for obj in world_map.world_object_list:
        x, y = gps_to_occupancy(obj.gps_xy, min_x, min_y, cell_width)

        if obj.typeid == 'zombie':
            zombie_range = int(base_zombie_ranges[obj.color] // cell_width)
            # create dist to nearest berry for purple zombies
            for i in range(x - zombie_range, x + zombie_range):
                for j in range(y - zombie_range, y + zombie_range):
                    if 0 <= i < (map_width - 1) and 0 <= j < (map_height - 1):
                        # print("Tried to add at idx", i, j)
                        occupancy_matrix[i][j] = 0
        occupancy_matrix[x][y] = 0
    grid = Grid(matrix=occupancy_matrix)
    return grid


def print_path(map, grid, path, target):
    cell_width = map.cell_width
    min_max_gps = list_min_max_gps(map.world_object_list)
    min_x = min_max_gps["min_x"]
    max_x = min_max_gps["max_x"]
    min_y = min_max_gps["min_y"]
    max_y = min_max_gps["max_y"]

    start = gps_to_occupancy(map.youbot.gps_xy, min_x, min_y, cell_width)
    end = gps_to_occupancy(target.gps_xy, min_x, min_y, cell_width)
    print('path length:', len(path))
    print(grid.grid_str(path=path, start=start, end=end))


def calculate_path(map, grid, target):
    cell_width = map.cell_width
    min_max_gps = list_min_max_gps(map.world_object_list)
    min_x = min_max_gps["min_x"]
    max_x = min_max_gps["max_x"]
    min_y = min_max_gps["min_y"]
    max_y = min_max_gps["max_y"]

    youbot_x, youbot_y = gps_to_occupancy(map.youbot.gps_xy, min_x, min_y, cell_width)
    start = grid.node(youbot_x, youbot_y)
    # Get position of target berry
    berry_x, berry_y = gps_to_occupancy(target.gps_xy, min_x, min_y, cell_width)
    end = grid.node(berry_x, berry_y)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)
    return path

def berry_seeking_target_coords(map, num_berries_considered, display_path=False):
    # Initialization functions
    min_max_gps = list_min_max_gps(world_map.world_object_list)
    grid = build_occupancy_grid(map)
    potential_paths = []

    # Calculate paths for n most promising berries
    for i in range(num_berries_considered):
        if i < len(world_map.world_berry_list):
            potential_paths.append(calculate_path(world_map, grid, world_map.world_berry_list[i]))
            grid.cleanup()

    # Find minimum length valid path
    if all(el == [] for el in potential_paths):
        print("No berry target found.")
        return None

    optimal_path = min(filter(lambda x: len(x) > 0, potential_paths), key=len, default=None)
    optimal_berry = world_map.world_berry_list[potential_paths.index(optimal_path)]

    # path_to_take = [(obj.x, obj.y) for obj in optimal_path]
    gps_target = occupancy_to_gps(optimal_path[1], min_max_gps["min_x"], min_max_gps["min_y"], CELL_WIDTH)

    # Optional print
    if display_path:
        print_path(world_map, grid, optimal_path, optimal_berry)
        print("Optimal Berry:", optimal_berry.color, "at", optimal_berry.gps_xy)
        print("Step to:", gps_target)

    return gps_target


def sandbox_wp():
    # %% Sandbox for Witt
    tmp = robot.step(TIME_STEP)

# SIMULATE using data structure
    posterior = random_posterior()
    world_map.youbot.gps_xy = [youbot.sensors["gps"].getValues()[0], youbot.sensors["gps"].getValues()[2]]
    world_map.world_berry_list = []
    world_map.youbot.bearing = get_comp_angle(world_map.youbot.sensors["compass"].getValues())

    # Probability
    toggle_prob = True
    if toggle_prob:
        nsamples = 5

        for i in range(nsamples):
            draw = drawfromposterior(posterior)
            color = rows[draw[0]]
            effect = cols[draw[1]]

            # Observe base object
            random_coords = [random.uniform(-10, 10), random.uniform(-10, 10)]
            base_obj = baseObject(world_map, random_coords)
            berry_obj = berryObject(dist=base_obj.dist2youbot,
                                    berry_color=color,
                                    map=world_map,
                                    gps_xy=random_coords)

            #Print priority score before observation
            p_score = berry_obj.priority_score()
            print("Priority score for", berry_obj.color,
                  "at dist", round(berry_obj.dist2youbot, 2), "is", round(p_score, 2))

            # Observe Berry
            berry_obj.observe_effect(effect)

        # Sort priority list
        world_map.world_berry_list = sorted(world_map.world_berry_list, key=lambda x: x.priority, reverse=True)
        for berry in world_map.world_berry_list:
            print("Color:", berry.color, "Dist:", berry.dist2youbot, "Priority:", berry.priority)

        print("Count:\n", world_map.count)
        print("Posterior\n", world_map.prior)

        plot_toggle = False
        if plot_toggle:
            # Plotting
            count = world_map.count
            prior = world_map.prior

            # Create subplots
            fig, axes = plt.subplots(1, 3, figsize=(25, 8))

            # Display the final updated prior
            axes[0].imshow(posterior)
            axes[0].set_title("Posterior")

            axes[1].imshow(count)
            axes[1].set_title("Berries Seen")

            axes[2].imshow(prior)
            axes[2].set_title("Prior")

    # identify berry to seek
    berry2seek = world_map.world_berry_list[0]

    # %% Movement
    toggle_move = False
    if toggle_move:
        steps = 100



        for i in range(steps):
            #Update Sensors
            tmp = robot.step(TIME_STEP)
            youbot.gps_xy = [youbot.sensors["gps"].getValues()[0], youbot.sensors["gps"].getValues()[2]]
            youbot.bearing = get_comp_angle(world_map.youbot.sensors["compass"].getValues())
            berry2seek.angle2youbot = angle2object(youbot, berry2seek)
            berry2seek.dist2youbot = distance(youbot.gps_xy, berry2seek.gps_xy)

            # Print testing
            print("Angle to obj:", berry2seek.angle2youbot * (180 / math.pi))
            print("Dist to berry:", berry2seek.dist2youbot)
            print("Orientation:", youbot.bearing * (180 / math.pi))
            print("Youbot:", youbot.gps_xy)
            print("Object:", berry2seek.gps_xy)

            path_to_object(youbot, berry2seek)

    # %% Pathing Sim

    # SIMULATE WORLD STATE
    world_map.world_object_list = []

    # zombies
    num_zombies = 5
    for i in range(num_zombies):
        zombie = zombieObject(world_map,
                              random.choice(zombie_colors),
                              (random.uniform(-10, 10), random.uniform(-10, 10)),
                              )
        world_map.world_zombie_list.append(zombie)
        # Add this behavior to zombies, walls, and berries!
        world_map.world_object_list.append(zombie)

    # walls
    num_walls = 30
    for i in range(num_walls):
        wall = baseObject(world_map,
                          (random.uniform(-10, 10), random.uniform(-10, 10)),
                          )
        world_map.world_solid_list.append(wall)
        world_map.world_object_list.append(wall)

    # berries
    for berry in world_map.world_berry_list:
        berry.gps_xy = (random.uniform(-10, 10), random.uniform(-10, 10))
        world_map.world_object_list.append(berry)

    # append youbot to world object list
    world_map.world_object_list.append(world_map.youbot)

    # BUILD OCCUPANCY MATRIX & CHART PATH
    grid = build_occupancy_grid(world_map)
    path = calculate_path(world_map, grid, berry2seek)
    #print_path(world_map, grid, path, berry2seek)
    grid.cleanup()

    # CHART FOR TOP FIVE BERRIES
    # use display_path=True arg to display
    berry_seeking_target_coords(world_map, 3, display_path=True)


def sandbox_ma():
    # %% Sandbox for Mohammad
    tmp = robot.step(TIME_STEP)
    youbot.sensors["gps"].getValues()
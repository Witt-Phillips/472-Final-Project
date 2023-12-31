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
    def __init__(self, grid_cell_width=grid_width_default):
        self.cell_width = grid_cell_width  # in meters
        self.timestep = None  # initialize placeholder for world timestep in simulation
        self.cell_object_table = {}  # hash table for "map_cell":object_list_in_cell (e.g. '[1,0]':[berry1 berry2 zombie1] )
        self.world_zombie_list = []
        self.world_berry_list = []
        self.world_berry_dict = []
        self.world_solid_list = []
        self.visible_objecs = []
        self.hidden_objects = []
        self.world_object_list = self.world_zombie_list + self.world_berry_list + self.world_solid_list
        self.cell_properties_list = []
        self._init_gps = []
        self.youbot = youbotObject(self)  # keep info on youtbot

        # Berry Probability
        self.prior = to_df(np.ones((4, 4)) / 4)
        self.count = to_df(np.zeros((4, 4)))
        self.draws = np.empty((0, 2))
        self.weights = pd.DataFrame(weights, index=cols, columns=weight_names)

    @property
    def init_gps(self):
        return self._init_gps
    @init_gps.setter
    def init_gps(self,value):
        self._init_gps = value

class baseObject():
    def __init__(self, map, object_id=None,typeid=None,cell_idx=None,cell_hash= None,velocity=None,gps_0=None):
        self.map = map
        self.typeid = typeid
        if object_id is None:
            self.object_id = id(self)
        self.cell_idx  = cell_idx
        self.cell_hash = cell_hash
        self.velocity  = velocity
        self.gps_0     = gps_0
        self._gps_xy   = [0, 0]  # Initialize with a default value

        self.origin_xy = map.init_gps
        self.angle2youbot = None


    def toZombie(self,color):
        return zombieObject(self.map,zombie_color=color,
                            self.object_id,self.cell_idx,self.cell_hash,self.velocity,self.gps_0)
    @property
    def gps_xy(self):
        if self.object_id != map.youbot.object_id:
            # Getter method - called when you access the property
            return self._gps_xy
        else: # if youbot is calling
            self.gps_0   = self._gps_xy
            self._gps_xy = map.youbot.getgps
            return map.youbot.getgps

    @gps_xy.setter
    def gps_xy(self, value):
        # Setter method - called when you assign a value to the property
        # You can add validation or additional logic here
        if isinstance(value, list) and len(value) == 2:
            self.gps_0   = self._gps_xy
            self._gps_xy = value
        else:
            raise ValueError("gps_xy must be a list with two elements")

    @property
    def dist2youbot(self):
        if self.object_id == self.map.youbot.object_id:
            return 0
        else:
            dist = distance(self.map.youbot.gps_xy, self.gps_xy)
            return dist

    @property
    def map_rc(self):
        if self.gps_xy[0] is not None:
            return

    def hash_gps_to_map(self):
        if self.gps_xy[1] is not None:
            map_rc = convert_gps_to_map(self.gps_xy, self.map)
            # Update object worldMap if necessary - ERROR - cannot be part of initialization
            # if self.map_rc != map_rc:
            #     self.update_cell_table(map_rc)
            return map_rc
        else:
            return None

    @property
    def orientation(self):
        x0,y0 = [i for i in self.gps_0]
        x1,y1 = [i for i in self.gps_xy]

        # Calculate the vector components
        dx = x1 - x0
        dy = y1 - y0

        # Calculate the angle from the x-axis
        angle_from_x_axis = math.atan2(dy, dx)

        # Calculate the angle from the y-axis
        angle_from_y_axis = math.pi / 2 - angle_from_x_axis

        # Convert the angle to degrees
        angle_from_y_axis_degrees = math.degrees(angle_from_y_axis)
        return angle_from_y_axis

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


    def __init__(self, map, color=None, object_id=None, typeid=None, cell_idx=None, cell_hash=None, velocity=None,
             gps_0=None,effect_type=None):

    super().__init__(map, object_id, 'zombie', cell_idx, cell_hash, velocity, gps_0)
        self.color = berry_color
        self.effect = effect_type
        self.priority  = self.priority_score()
        self.reachable = None
        self.priority = self.priority_score()

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
        super().__init__(map, 'youbot')
        self.wb_robot = wb_robot
        #WP Added Robot Info parameter
        self.robot_info = None
        self.sensors = sensors
        self.wheels = wheels

    @property
    def wheel_velocity(self):
        return [self.wheels[key].getVelocity()  for key in self.wheels.keys()]


    @wheel_velocity.setter
    def wheel_velocity(self,vel):
        for key in youbot.wheels.keys():
            youbot.wheels[key].setVelocity(vel)

    @property
    def getgps(self):
        gps = self.sensors["gps"].getValues()
        return [gps[0], gps[2]]

    @property
    def update_orientation(self):

        return get_comp_angle(self.sensors["compass"].getValues())


class zombieObject(baseObject):
    def __init__(self, map, color=None, object_id=None,typeid=None,cell_idx=None,cell_hash= None,velocity=None,gps_0=None):
        super().__init__(map,object_id,'zombie',cell_idx,cell_hash,velocity,gps_0)
        self.color = color
        self.chasing = False
        # self.map_rc = self.hash_gps_to_map() if gps_xy is not None else None
        # if gps_xy is not None:
        #     self.bearing = distance(map.youbot.gps_xy, self.gps_xy)
        #     self.distance = map.youbot.gps_xy
        # else:
        #     self.bearing = None
        #     self.distance = None

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


def angle2object(*, youbot=None, obj=None, gps_xy=None):
    if gps_xy is not None:
        x, y = gps_xy
    elif obj is not None:
        x, y = obj.gps_xy
    else:
        return "Tried to path without valid target!"

    xdif = youbot.gps_xy[0] - x
    ydif = youbot.gps_xy[1] - y
    # Add pi / 2 to angle?
    angle = math.atan2(ydif, xdif) - youbot.bearing + (math.pi / 2)
    factor = abs(angle // (2 * math.pi))
    if factor != 0:
        angle /= factor
    if angle < 0:
        angle = angle + (2 * math.pi)
    #print("Angle to obj:", angle * (180 / math.pi))
    return angle


def path_to_object(*, youbot=None, obj=None, gps_xy=None):
    if youbot is None:
        return "Tried to path without valid youbot!"
    if gps_xy is not None:
        angle = angle2object(youbot=youbot, gps_xy=gps_xy)
        dist = distance(youbot.gps_xy, gps_xy)
    elif obj is not None:
        angle = angle2object(youbot=youbot, obj=obj)
        dist = distance(youbot.gps_xy, obj.gps_xy)
    else:
        return "Tried to path without valid target!"

    if dist < 0.1:
        print("Waypoint reached!")
        youbot.wheels["front_right"].setVelocity(8.0)
        youbot.wheels["front_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(8.0)
        youbot.wheels["back_left"].setVelocity(8.0)
    # straight
    elif angle < to_rad(5) or angle > to_rad(355):
        youbot.wheels["front_right"].setVelocity(8.0)
        youbot.wheels["front_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(8.0)
        youbot.wheels["back_left"].setVelocity(8.0)
    # soft left
    elif angle < to_rad(45):
        youbot.wheels["front_right"].setVelocity(4.0)
        youbot.wheels["front_left"].setVelocity(8.0)
        youbot.wheels["back_right"].setVelocity(4.0)
        youbot.wheels["back_left"].setVelocity(8.0)
    # soft right
    elif angle > to_rad(315):
        youbot.wheels["front_right"].setVelocity(8.0)
        youbot.wheels["front_left"].setVelocity(4.0)
        youbot.wheels["back_right"].setVelocity(8.0)
        youbot.wheels["back_left"].setVelocity(4.0)
    # hard right
    elif angle > math.pi:
        youbot.wheels["front_right"].setVelocity(5.0)
        youbot.wheels["front_left"].setVelocity(-5.0)
        youbot.wheels["back_right"].setVelocity(5.0)
        youbot.wheels["back_left"].setVelocity(-5.0)
    # hard left
    else:
        youbot.wheels["front_right"].setVelocity(-5.0)
        youbot.wheels["front_left"].setVelocity(5.0)
        youbot.wheels["back_right"].setVelocity(-5.0)
        youbot.wheels["back_left"].setVelocity(5.0)

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

    # Map egocenteric orientation
    theta = ((_2pi / 512) * (beam_number+1)) - (math.pi/2) # (3*math.pi/2)

    # if theta > _2pi:
    #     theta = theta - (_2pi)  # subtract by 2𝜋 if  𝜃 > 2𝜋

    # Find coords & map to robot
    gps_xy = multiply([cos(theta), sin(theta)], magnitude)


    return theta, gps_xy


def assign_object(map, gps_xy):
    pass

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

    youbot = map.youbot

    youbot.wb_robot = robot
    youbot.robot_info = robot_info
    youbot.sensors = sensors
    youbot.wheels = wheels

    init_gps = youbot.sensors["gps"].getValues()
    youbot.gps_0 = [init_gps[0], init_gps[2]]

    map.timestep = timestep

    return youbot


########### Main Function ###############
def main(simparams=None):
    # Initialize main map & establish relative center from GPS
    map = worldMapObject()

    # Initialize youbot in world with sensors
    youbot = init_youbot(map)
    robot = youbot.wb_robot

    # Run get all berry positions from controllers/youbot_controllers/youbot_zombie.py
    get_all_berry_pos(robot)

    # Initialize plot
    if plotMap:
        fig, ax = plot_init(map)

    robot_not_dead = 1
    timestep = map.timestep
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
                object_list = map.world_object_list

                # This could be made more efficient if we limit search to only hash coordinates within youbot (r, 𝜃)
                gpsvals, = [object.gps_xy for object in object_list]

                # mainMap.map_lidar(i, lidar_values[i], orientation)

        # if plotMap:
        #     new_objects = get_objects_to_update(map)
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



# %% ----------- Sandbox -----------

# Initialize map
map = worldMapObject()

# Initialize youbot in world with sensors
youbot   = init_youbot(map)
robot    = youbot.wb_robot
timestep = map.timestep

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
map.init_gps = [youbot.sensors["gps"].getValues()[0],youbot.sensors["gps"].getValues()[2]]


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

#%%
def analyzeColor(map,plts=None,plotStuff=False):

    # Explore Back Camera Image Processing
    rgb, hsv = pullFrame(map.youbot)

    # Define saturation and value ranges
    saturation_range = [40, 100]
    value_range = [0, 100]

    hue_ranges = {
        "red1": [0, 9],
        "red2": [340, 360],
        "orange": [12, 27],
        "yellow": [53, 60],
        "green": [69, 140],
        "aqua": [155, 190],
        "blue": [190, 215],
        "purple": [250, 280],
        "pink": [286, 333],
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

    plotStuff = plotStuff
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
    else:
        return isolated_regions


#%%
def isolateCameraRegions(map):
    # Explore Back Camera Image Processing
    rgb, hsv = pullFrame(map.youbot)

    # Define saturation and value ranges
    saturation_range = [30, 100]
    value_range = [0, 100]

    hue_ranges = {
        "red1": [0, 5],
        "red2": [340, 360],
        "orange": [21, 30],
        "yellow": [39, 60],
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



def runstep(value=None , func=None):
    tmp = robot.step(TIME_STEP)
    if isinstance(value,int):
        for i in range(value - 1):
            if isinstance(func, str):
                print(eval(func))
                tmp = robot.step(TIME_STEP)
            else:
                tmp = robot.step(TIME_STEP)
    elif isinstance(value,str):
                print(eval(value))


def lidarDetect(map):
    #%%
    youbot = map.youbot

    # tmp = robot.step(TIME_STEP)
    lidar_values = map.youbot.sensors["lidar"].getRangeImage()

    x = youbot.gps_xy[0]
    y = youbot.gps_xy[1]

    # Create a figure and axis
    fig, ax = plt.subplots()
    plt.ion()
    # Set up plot limits, labels, title, etc.
    ax.set_xlim(-10, 10)  # Adjust these limits based on your expected data range
    ax.set_ylim(-10, 10)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Lidar Data Visualization')


    ## Scan Image -----------------
    cmaps = analyzeColor(map)
    sum0 = [np.sum(x, (0, 2)) for x in cmaps.values()]
    sumb = np.sum(cmaps["blue"][:,:,2], (0, 2))


    berry_idxs = {"red":[] , "yellow":[], "orange": [] ,"pink": []}
    berry_threshold = 200
    for color in berry_idxs.keys():
        vec = np.sum(cmaps[color],(0,2))
        berry_idxs[color] = np.where(vec > berry_threshold)

    zombie_idxs = {"aqua": [], "blue": [], "purple": [], "green": []}
    zombie_threshold = 500
    for color in zombie_idxs.keys():
        vec = np.sum(cmaps[color], (0, 2))
        zombie_idxs[color] = np.where(vec > zombie_threshold)

    # Scan Lidar -----------

    deg30 = round(30 // (360 / 512))

    rightidx = 512 - deg30
    leftidx  = deg30
    # Build Scene Object list
    objs_in_view = []
    count1,count2 = 0,0
    continuity_buffer = 4
    for i in range(lidar_values):
        mag = lidar_values[i]

        lidar_in_range = (mag != float('inf')) & (mag != 0.0)
        if lidar_in_range:
            if count1 == 0:
                obj = baseObject()
            # Check if we are within image range
            if rightidx <= i <= leftidx:
                # Check if continuity has been broken
                if (count2 < continuity_buffer):

                    count1,count2 = 0,0

                # Check # beams that are continuously in same range
                # If we
                    theta, gps_xy = map_lidar(map, i, lidar_values[i])

            count1 += 1
        else:
            count1  = 0
            count2 += 1



    # Go through Objects in Scene and compare with hidden/visible objects
    objs_vis = map.v
    while len(objs_in_view) > 0:
        obj = objs_in_view.pop()


    ax.plot(xsv, ysv, color='blue')  # Add color if needed

    plt.show()

def sandbox_dc():
# %% Sandbox for Dan
# Create a figure with subplots
    plts = analyzeColor(map,plotStuff=True)
#%%
    tmp = robot.step(TIME_STEP)
    analyzeColor(map,plts,plotStuff=True)
    cmaps = analyzeColor(map)
    a = np.sum(cmaps["blue"][:,:,2],0)
    x = [np.sum(x,(0,2)) for x in cmaps.values()]

#%% assess object
    lidar_objects = lidar2image(map)

    # If lidar is picking up objects in visible region of world map
    if lidar_objects is not None:
    # analy
        analyzeScene(map)


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
    for obj in map.world_object_list:
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


def berry_seeking_target_coords(map, num_berries_considered, steps_ahead, display_path=False):
    # Initialization functions
    min_max_gps = list_min_max_gps(map.world_object_list)
    grid = build_occupancy_grid(map)
    potential_paths = []

    # Calculate paths for n most promising berries
    for i in range(num_berries_considered):
        if i < len(map.world_berry_list):
            potential_paths.append(calculate_path(map, grid, map.world_berry_list[i]))
            grid.cleanup()

    # Find minimum length valid path
    if all(el == [] for el in potential_paths):
        print("No berry target found.")
        return None

    optimal_path = min(filter(lambda x: len(x) > 0, potential_paths), key=len, default=None)
    optimal_berry = map.world_berry_list[potential_paths.index(optimal_path)]
    if len(optimal_path) <= steps_ahead:
        steps_ahead = len(optimal_path) - 1

    # path_to_take = [(obj.x, obj.y) for obj in optimal_path]
    gps_target = occupancy_to_gps(optimal_path[steps_ahead], min_max_gps["min_x"], min_max_gps["min_y"], CELL_WIDTH)

    # Optional print
    if display_path:
        print_path(map, grid, optimal_path, optimal_berry)
        print("Optimal Berry:", optimal_berry.color, "at", optimal_berry.gps_xy)
        print("Youbot:", youbot.gps_xy)
        print("Step to:", gps_target)


    return gps_target, optimal_berry


def sandbox_wp():
# %% Sandbox for Witt
    tmp = robot.step(TIME_STEP)

# SIMULATE using data structure
    posterior = random_posterior()
    map.youbot.gps_xy = [youbot.sensors["gps"].getValues()[0], youbot.sensors["gps"].getValues()[2]]
    map.world_berry_list = []
    map.youbot.bearing = get_comp_angle(map.youbot.sensors["compass"].getValues())

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
            base_obj = baseObject(map, random_coords)
            berry_obj = berryObject(dist=base_obj.dist2youbot,
                                    berry_color=color,
                                    map=map,
                                    gps_xy=random_coords)

            #Print priority score before observation
            p_score = berry_obj.priority_score()
            print("Priority score for", berry_obj.color,
                  "at dist", round(berry_obj.dist2youbot, 2), "is", round(p_score, 2))

            # Observe Berry
            berry_obj.observe_effect(effect)

        # Sort priority list
        map.world_berry_list = sorted(map.world_berry_list, key=lambda x: x.priority, reverse=True)
        for berry in map.world_berry_list:
            print("Color:", berry.color, "Dist:", berry.dist2youbot, "Priority:", berry.priority)

        print("Count:\n", map.count)
        print("Posterior\n", map.prior)

        plot_toggle = False
        if plot_toggle:
            # Plotting
            count = map.count
            prior = map.prior

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
    berry2seek = map.world_berry_list[0]
# %% Pathing Sim
    youbot.gps_xy = [youbot.sensors["gps"].getValues()[0], youbot.sensors["gps"].getValues()[2]]
    map.world_object_list = []

    # zombies
    num_zombies = 5
    for i in range(num_zombies):
        zombie = zombieObject(map,
                              random.choice(zombie_colors),
                              (random.uniform(-10, 10), random.uniform(-10, 10)),
                              )
        map.world_zombie_list.append(zombie)
        # Add this behavior to zombies, walls, and berries!
        map.world_object_list.append(zombie)

    # walls
    num_walls = 30
    for i in range(num_walls):
        wall = baseObject(map,
                          (random.uniform(-10, 10), random.uniform(-10, 10)),
                          )
        map.world_solid_list.append(wall)
        map.world_object_list.append(wall)

    # berries
    for berry in map.world_berry_list:
        berry.gps_xy = (random.uniform(-10, 10), random.uniform(-10, 10))
        map.world_object_list.append(berry)

    # append youbot to world object list
    map.world_object_list.append(map.youbot)

    # CHART FOR TOP N BERRIES
    target_coords, optimal_berry = berry_seeking_target_coords(map, 2, 3, display_path=True)

    # %% Movement based on gps_xy
    move_from_pathing = True
    if move_from_pathing:
        steps = 75

        for i in range(steps):
            # Update Sensors
            tmp = robot.step(TIME_STEP)
            youbot.gps_xy = [youbot.sensors["gps"].getValues()[0], youbot.sensors["gps"].getValues()[2]]
            youbot.bearing = get_comp_angle(map.youbot.sensors["compass"].getValues())
            if i % 10 == 1:
                target_coords, optimal_berry = berry_seeking_target_coords(map, 2, 3, display_path=False)

            # Print testing
            print_on = False
            if print_on:
                if i % 10 == 1:
                    # print("Angle to obj:", angle2object(youbot=youbot, gps_xy=target_coords) * (180 / math.pi))
                    # print("Dist to berry:", distance(youbot.gps_xy, target_coords))
                    # print("Orientation:", youbot.bearing * (180 / math.pi))
                    print("Youbot:", youbot.gps_xy)
                    print("Next path coords:", target_coords)
                    print("Berry coords", optimal_berry.gps_xy)


            if i % 3 == 1:
                path_to_object(youbot=youbot, gps_xy=target_coords)



def sandbox_ma():
    # %% Sandbox for Mohammad
    tmp = robot.step(TIME_STEP)
    youbot.sensors["gps"].getValues()
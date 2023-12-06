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
from math import sin, cos
import random

# Define Global Variables
_2pi = 2 * math.pi
grid_width_default = 0.1
plotMap = True

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

class worldMapObject:
    def __init__(self, youbot=None, grid_cell_width=grid_width_default):
        self.cell_width = grid_cell_width  # in meters
        self.youbot = youbot  # keep info on youtbot
        self.timestep = None  # initialize placeholder for world timestep in simulation
        self.cell_object_table = {}  # hash table for "map_cell":object_list_in_cell (e.g. '[1,0]':[berry1 berry2 zombie1] )
        self.world_zombie_list = []
        self.world_berry_list = []
        self.world_solid_list = []
        self.visible_objects  = []
        self.hidden_objects   = []
        self.world_object_list = self.world_zombie_list + self.world_berry_list + self.world_solid_list
        self.cell_properties_list = []

        # Berry Probability
        self.prior = to_df(np.ones((4, 4)) / 4)
        self.count = to_df(np.zeros((4, 4)))
        self.draws = np.empty((0, 2))
        self.weights = pd.DataFrame(weights, index=cols, columns=weight_names)

    @property
    def init_gps(self):
        return self.youbot.init_gps

class baseObject():
    def __init__(self, map, gps_xy=None, typeid=None, origin_xy=None):
        if gps_xy is None:
            gps_xy = [None, None]
        self.map       = map
        self.typeid    = typeid
        self.object_id = id(self)
        self.cell_idx  = None
        self.cell_hash = None
        self.velocity  = None
        self.gps_0     = None
        self.gps_xy    = gps_xy


    @property
    def gps_xy(self, gps_xy=None):
        if gps_xy is None:
            return gps_1


    @property
    def init_gps(self):
        return self.map.init_gps

    @propery
    def dist2youbot(self):
        if self.object_id == self.map.youbot.object_id:
            return 0
        else:
            dist = distance(self.map.youbot.gps_xy, self.gps_0)
            return dist

    @property
    def map_rc(self):
        if self.gps_xy[0] is not None:
            map_rc = convert_gps_to_map(self.gps_xy, self.map)
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
    def __init__(self, map, dist=None, berry_color=None, effect_type=None, gps_xy=None):
        super().__init__(map, gps_xy, dist, 'berry')
        self.color = berry_color
        self.effect = effect_type
        self.dist2youbot = dist
        self.priority  = self.priority_score()
        self.reachable = None
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

        # self.orientation = get_comp_angle(self)
        map.youbot = self

    @property
    def getgps(self):
        gpsxy = self.sensors["gps"].getValues()[0,2]
        self.gps_xy = gpsxy
        return gpsxy



class zombieObject(baseObject):
    def __init__(self, map, zombie_color=None, gps_xy=None, typeid='zombie'):
        super().__init__(map, gps_xy, typeid)
        self.color = zombie_color
        self.chasing = False
        if gps_xy is not None:
            self.bearing = distance(map.youbot.gps_xy, self.gps_xy)
            self.distance = map.youbot.gps_xy
        else:
            self.bearing = None
            self.distance = None

        map.world_zombie_list.append(self)


class wallObject():
    def __init__(self, map, wall_color='brown', gps_xy=None, typeid='wall', moveable=False):
        super().__init__(map, gps_xy, typeid)
        self.moveable = moveable
        self.color = "green" if self.moveable else "brown"
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


def orient_to_object(youbot, object):
    pass


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


def get_comp_angle(compass_values):
    angle = math.atan2(compass_values[1], compass_values[0])
    return angle + (_2pi * (angle < 0.0))


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

# Probability Utility Functions


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

    youbot = youbotObject(map,sensors=sensors,wheels=wheels)
    youbot.wb_robot   = robot
    youbot.robot_info = robot_info

    init_gps = youbot.sensors["gps"].getValues()
    youbot.gps_0 = [init_gps[0], init_gps[2]]

    map.timestep = timestep

    return youbot


########### Main Function ###############
def main(simparams=None):
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # health, energy, armour in that order
    robot_info = [100, 100, 0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0

    # ------------------CHANGE CODE BELOW HERE ONLY--------------------------

    # COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    # accelerometer = robot.getDevice("accelerometer")
    # accelerometer.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    # compass = robot.getDevice("compass")
    # compass.enable(timestep)

    # camera1 = robot.getDevice("ForwardLowResBigFov")
    # camera1.enable(timestep)

    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)

    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)

    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)

    # camera5 = robot.getDevice("BackLowRes")
    # camera5.enable(timestep)

    # camera6 = robot.getDevice("RightLowRes")
    # camera6.enable(timestep)

    # camera7 = robot.getDevice("LeftLowRes")
    # camera7.enable(timestep)

    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)

    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)

    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)

    # receiver = robot.getDevice("receiver")
    # receiver.enable(timestep)

    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)

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

    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")

    get_all_berry_pos(robot)

    robot_not_dead = 1

# --------- Start of our code -----------------------
    # Initialize main map & establish relative center from GPS
    world_map = worldMapObject()

    # Initialize youbot in world with sensors
    youbot = init_youbot(world_map)
    robot  = youbot.wb_robot

    sensors = {
        "gps": gps,
        "lidar": lidar,
        "camera": camera8
    }

    wheels = {
        "front_right": fr,
        "front_left": fl,
        "back_right": br,
        "back_left": bl,
    }

    youbot = youbotObject(map, sensors=sensors, wheels=wheels)
    youbot.wb_robot   = robot
    youbot.robot_info = robot_info

    # initialize Values
    robot.step(TIME_STEP)
    init_gps = youbot.sensors["gps"].getValues()
    youbot.gps_0 = round(youbot.sensors["gps"].getValues()[0, 2], 3)


    # Initialize plot if we are plotting
    plotMap = false
    if plotMap:
        fig, ax = plot_init(world_map)


# ------------ Start Sensor Control Loop ------------------
    while robot.step(TIME_STEP) != -1:
        # Get sensor data
        gps_values = gps.getValues()
        lidar_values = lidar.getRangeImage()
        compass_values = compass.getValues()

        orientation = get_comp_angle(compass_values)
        gpsX = round(gps_values[0], 3)
        gpsY = round(gps_values[2], 3)

        # Cast gps readings to map coords
        mappedX = mainMap.gps_to_map(mainMap.initialReading[0], gpsX)
        mappedY = mainMap.gps_to_map(mainMap.initialReading[1], gpsY)
        coords = str_coords(mappedX, mappedY)

        # Create new dictionary entry if cell unencountered
        if mainMap.cellTable.get(coords) is None:
            mainMap.cellTable[coords] = MapCell(mappedX, mappedY, None, None, None, None)

            # Manipulate current cell
        mainMap.currCell = mainMap.cellTable.get(coords)
        mainMap.currCell.visited = True

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







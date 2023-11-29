"""youbot_controller controller."""

import sys,os

def add_subdirs(root_dir):
    for subdir, dirs, files in os.walk(root_dir):
            sys.path.append(subdir)

add_subdirs('./lib')

webots_home = os.environ['WEBOTS_HOME']

# dirs = ['./lib', os.path.join(webots_home, 'lib', 'controller')]
# from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
# from controller import Supervisor


from youbot_zombie import *

# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# define functions here for making decisions and using sensor inputs


# Addditional packages/ constants---------------------------------------------------------------
import numpy as np
import cv2
import math
import matplotlib as plt
from math import sin, cos

# Define Global Variables
_2pi = 2*math.pi
grid_width_default = 0.1
plotMap = True

#%%  Define Classes
class worldMapObject:
    def __init__(self, youbot=None, grid_cell_width=0.1):
        self.cell_width = grid_cell_width  # in meters
        self.youbot     = youbot # keep info on youtbot
        self.timestep   = None # initialize placeholder for world timestep in simulation
        self.cell_object_table    = {} # hash table for "map_cell":object_list_in_cell (e.g. '[1,0]':[berry1 berry2 zombie1] )
        self.world_object_list    = []
        self.cell_properties_list = []
        self.berryProbabilities   ={
            "red": None,
            "pink": None,
            "orange": None,
            "yellow": None
        }

    def update_berry_probability(self, berry_obj, effect_dif):
        # Implement logic to update probability
        color = berry_obj.berry_color

        pass

class baseObject():
    def __init__(self, map, gps_xy=None, typeid=None):
        if gps_xy is None:
            gps_xy = [None, None]
        self.map       = map
        self.typeid    = typeid
        self.object_id = id(self)
        self.cell_idx  = None
        self.cell_hash = None
        self.gps_xy    = gps_xy
        self.map_rc    = self.hash_gps_to_map()
        self.init_gps  = map.youbot.init_gps

    def hash_gps_to_map(self):
        if self.gps_xy is not None:
            map_rc = convert_gps_to_map(self.gps_xy, self.map)
            # Update object worldMap if necessary
            if (self.map_rc != map_rc):
                self.update_cell_table(map_rc)
            return map_rc
        else:
            return None

    def update_cell_table(self , cell_rc):

        # Assign object to cell_object_table
        if self.cell_hash is None:
            # If this is the first time asigning this object to a grid cell
            # Update the hash string to '[row , col]'
            self.cell_hash = cell_rc.__str__()
            if self.map.cell_object_table.get(self.cell_hash) is None:
                # If the hash does not exist this is the first object in that cell
                # Initialize the list at this hash with this object
                self.map.cell_object_table[self.map_rc.__str__()] = [self]

            else: # Append the list at this hash with this object
                self.map.cell_object_table[self.map_rc.__str__()].append(self.cell_hash)
        else:
            # This object exists in the world
            # are we in a new cell?
            get_object
            if self.map_rc = map_rc

        # list of objects in cell
        cell_list = self.map.cell_object_table[self.cell_hash]

        # update map_rc of object



class youbotObject(baseObject):
    def __init__(self, map, wb_robot=None,sensors=None, wheels=None,gps_xy=None,init_gps_xy=None):
        super().__init__(map, gps_xy, 'youbot')
        self.wb_robot = wb_robot
        self.sensors  = sensors
        self.wheels   = wheels
        self.init_gps = init_gps_xy
        map.youbot = self

class berryObject(baseObject):
    def __init__(self, map, berry_color=None, gps_xy=None):
        super().__init__(map, gps_xy,'berry')
        self.color     = berry_color
        self.primary   = None
        self.secondary = None
        map.world_object_list.append(self)

class zombieObject(baseObject):
    def __init__(self, map, zombie_color=None, gps_xy=None, typeid='zombie'):
        super().__init__(map,gps_xy,typeid)
        self.color     = zombie_color
        self.chasing   = False
        if gps_xy is not None:
            self.bearing   = distance( map.youbot.gps_xy,self.gps_xy)
            self.distance  = map.youbot.gps_xy
        else:
            self.bearing  = None
            self.distance = None

        map.world_object_list.append(self)


class wallObject():
    def __init__(self, map, wall_color='brown', gps_xy=None, typeid='wall', moveable=False):
        super().__init__(map,gps_xy,typeid)
        self.moveable = moveable
        self.color    = "green" if self.moveable else "brown"
        map.world_object_list.append(self)

class cellObject():
    def __init__(self, map, friction=None, danger_level=None):
        super().__init__(map,None,'cell')
        self.friction = friction # scaler value based on slippage of wheels and/or youbot speed for == acceleration
        self.danger_level = danger_level # based on zombies we think are around it and our historical health score change when we are in this cell
        map.cell_properties_list.append(self)

#%%  Define Functions

def getObjectRGB(object):
    color_map = {
        "purple_zombie":"purple" , "green_zombie":"green",
        "aqua_zombie": "aqua", "blue_zombie": "blue",
        "pink_berry": "pink", "orange_berry": "orange",
        "red_berry": "red", "yellow_berry": "yellow",
        "brown": "gray"
    }


def orient_to_object(youbot , object):
    pass

########## Utility functions

def multiply(list , multiplier):
    return [x * multiplier for x in list]

def distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def convert_gps_to_map(gps_xy, map):
    # Transform from world coordinates to map coordinates
    # x, y are world coordinates
    # return x, y in map coordinates
    return (gps_xy - map.init_gps) // map.cell_width

def get_comp_angle(compass_values):
    angle = math.atan2(compass_values[1], compass_values[0])
    return angle + ( _2pi * (angle < 0.0))

def map_lidar(map, beam_number, magnitude):

    # map_lidar(self, beamNum, dist, ori)
    # for i in range(len(lidar_values)):
    #         mainMap.map_lidar(i, lidar_values[i], orientation)

    # Normalize angle
    theta = ((_2pi / 512) * beam_number) + map.youbot.orientation
    theta = theta - (_2pi *(theta>_2pi)) # subtract by 2𝜋 if  𝜃 > 2𝜋

    # Find coords & map
    gps_xy = multiply([cos(theta),sin(theta)], magnitude)

    # Figure out if there is an object marked close to that position
    objects = map.world_object_list

    positions = [obj.gps_xy for obj in objects]

    assign_object(map , gps_xy)
    # Toggles solid field
    if self.cellTable.get(coords) is None:
        self.cellTable[coords] = MapCell(xMapped, yMapped, None, None, True)
        # print("New solid detected at", coords, "marked solid")
    else:
        cell = self.cellTable.get(coords)
        cell.solid = True
        # print("Existing cell at", coords, "marked solid")

def assign_object(map , gps_xy):

########### Plotting Functions ###############
def plot_init(map):
    # Initialize plot
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    ax.grid()

    tags = ['_'.join([obj.typeid,obj.color]) for obj in map.cell_object_list]
    unique_tags = np.unique(tags)

    # Lambda function to filter objects by typeid
    filter_by_typeid = lambda typeid: [obj for obj in map.cell_object_list if obj.typeid == typeid]
    filter_by_typeid = lambda typeid: [obj for obj in map.cell_object_list if obj.typeid == typeid]
    # Creating the dictionary
    groot_tags = {tag: filter_by_typeid(tag) for tag in unique_tags}

    # Adding additional keys if needed
    groot_tags["youbot"] =
    groot_tags["visit_map"] = some_visit_map_value

    object_list = map.cell_object_list

    max_cell  = max([max(obj.map_rc) for obj in object_list])
    max_grid  = max_cell + 0.1
    grid_range = np.arange(-max_grid, max_grid, 1)
    ax.set_xticks(grid_range)
    ax.set_yticks(grid_range)
    ax.grid(True, which='both', color='black', linewidth=1,)

    ax.set_xlim(-max_grid - 0.5, max_grid + 0.5)
    ax.set_ylim(-max_grid - 0.5, max_grid + 0.5)

    # Draw gridlines and squares
    for x in np.arange(-max_cell, max_cell + 1):
        for y in np.arange(-max_cell, max_cell + 1):
            cell_key = f"({x}, {y})"
            if cell_key in map_instance.cellTable:
                cell = map_instance.cellTable[cell_key]
                color = 'green' if cell.visited else 'black'
            else:
                color = 'white'
            ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color=color, edgecolor='black'))

    # Plot cells
    for cell in map_instance.cellTable.values():
        color = 'green' if (cell.xPos, cell.yPos) == current_pos else 'black' if cell.visited else 'white'
        ax.add_patch(plt.Rectangle((cell.xPos - 0.5, cell.yPos - 0.5), 1, 1, color=color))

    # Scatter different objects
    object_colors = {
        "purple_zombie": "purple", "green_zombie": "green", "aqua_zombie": "aqua",
        "blue_zombie": "blue", "pink_berry": "pink", "orange_berry": "orange",
        "red_berry": "red", "yellow_berry": "yellow", "wall": "gray", "self": "black"
    }
    for cell in map_instance.cellTable.values():
        if cell.typeid:
            obj_type = cell.typeid
            ax.scatter(cell.xPos, cell.yPos, color=object_colors[obj_type], s=30)

    plt.draw()
    plt.pause(0.1)

    return fig, ax

def update_plot(map, new_objects, ax):
    # robot position
    for obj in new_objects():
        if cell.typeid:
            obj_type = cell.typeid
            ax.scatter(cell.xPos, cell.yPos, color=object_colors[obj_type], s=30)

    x_positions = [cell.xPos for cell in cells]
    ax.scatter(x, y, c='r', marker='o')
    fig.canvas.draw()
    fig.canvas.flush_events()

########### World Initiation Functions ###############
def init_youbot(map):

    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # health, energy, armour in that order
    robot_info = [100, 100, 0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0


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
        "gps":gps,
        "lidar":lidar,
        "light":lightSensor,
        "camera":camera8,
        "compass":compass
    }

    wheels = {
        "font_right":fr,
        "font_left": fl,
        "back_right":br,
        "back_left": bl,
    }

    youbot = youbotObject(map)
    youbot.wb_robot   = robot
    youbot.robot_info = robot_info
    youbot.sensors    = sensors

    map.timestep      = timestep

    return youbot

def sim_random_world(map,nzombies=3,nberries=10,ntimesteps=100):

    zombie_colors = ("purple", "green", "blue", "aqua")
    berry_colors  = ("pink", "orange", "red" , "yellow")

    for zombie in range(nzombies):
        zombieObject(map,zombie_color='rand',gps_xy='rand')

    for berry in range(nberries):
        berryObject(map,berry_color='rand',gps_xy='rand')

    youbot_trajectories = [0,0]

    # Update Trajectories
    for t in range(ntimesteps):
        pass

########### Main Function ###############
def main(simparams=None):

    #Initialize main map & establish relative center from GPS
    world_map = worldMapObject()

    #Initialize youbot in world with sensors
    youbot = init_youbot(world_map)
    robot  = youbot.wb_robot

    # Run get all berry positions from controllers/youbot_controllers/youbot_zombie.py
    get_all_berry_pos(robot)

    # Initialize plot
    if plotMap:
        fig, ax = plot_init(world_map)

    robot_not_dead = 1
    timestep       = world_map.timestep
    robot_node     = robot.getFromDef("Youbot")
    trans_field    = robot_node.getField("translation")

    youbot.init_gps = round(youbot.sensors["gps"].getValues()[0,2], 3)

    # Temp variable to track coord change across time steps
    temp_coords = ""

    if simparams is not None:
        sim_random_world(nzombies=simparams["nzombies"],
                         nberries=simparams["nberries"])

    # Sensor Control Loop
    while robot.step(TIME_STEP) != -1:

        # Get sensor data
        gps_values     = youbot.sensors["gps"].getValues()
        lidar_values   = youbot.sensors["lidar"].getRangeImage()
        compass_values = youbot.sensors["compass"].getValues()

        # Update youbot orientation and gps position
        youbot.orientation  = get_comp_angle(compass_values)
        youbot.gps_xy       = round(gps_values[0,2], 3)

        # Lidar Mapping
        for i in range(len(lidar_values)):
            if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
                object_list = world_map.world_object_list

                # This could be made more efficient if we limit search to only hash coordinates within youbot (r, 𝜃)
                gpsvals, = [object.gps_xy for object in object_list]

                mainMap.map_lidar(i, lidar_values[i], orientation)


        if plotMap:
            new_objects = get_objects_to_update(world_map)

            update_plot(map, new_objects, ax)


def sandbox():
    simparams = {
        "nzombies":3,
        "nberries":10
    }
    main(simparams)


def old_main():

    robot = []
    mainMap = []
    # Temp variable to track coord change across time steps
    temp_coords = ""

    # Sensor Control Loop
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

        # Lidar Mapping
        for i in range(len(lidar_values)):
            if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
                mainMap.map_lidar(i, lidar_values[i], orientation)
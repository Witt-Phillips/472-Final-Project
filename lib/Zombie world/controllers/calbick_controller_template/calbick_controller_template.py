"""youbot_controller controller."""

import sys,os
webots_home = '/Applications/WeBots.app'  # Replace with your actual path
root_dir = os.path.join(webots_home, 'lib', 'controller')

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor
from youbot_zombie import *


#%%
# ------------------CHANGE CODE BELOW HERE ONLY--------------------------
# Addditional packages/ constants---------------------------------------------------------------
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
from math import sin, cos
import random

# Define Global Variables
_2pi = 2*math.pi
grid_width_default = 0.1
plotMap = True

#%%  Define Classes
class worldMapObject:
    def __init__(self, youbot=None, grid_cell_width=grid_width_default):
        self.cell_width = grid_cell_width  # in meters
        self.youbot     = youbot # keep info on youtbot
        self.timestep   = None # initialize placeholder for world timestep in simulation
        self.cell_object_table    = {} # hash table for "map_cell":object_list_in_cell (e.g. '[1,0]':[berry1 berry2 zombie1] )
        self.world_zombie_list    = []
        self.world_berry_list     = []
        self.world_solid_list     = []
        self.world_object_list    = self.world_zombie_list + self.world_berry_list + self.world_solid_list
        self.cell_properties_list = []

        props   = {"id": None, "probability": None}
        effects = {"effect1":props, "effect2":props}
        self.berryProbabilities   = {
            "red": effects,
            "pink": effects,
            "orange": effects,
            "yellow": effects
        }

    def update_berry_probability(self, berry_obj, effect_dif):
        # Implement logic to update probability
        color = berry_obj.berry_color

        pass

class baseObject():
    def __init__(self, map, gps_xy=None, typeid=None , origin_xy=None):
        if gps_xy is None:
            gps_xy = [None, None]
        self.map        = map
        self.typeid     = typeid
        self.object_id  = id(self)
        self.cell_idx   = None
        self.cell_hash  = None
        self.gps_xy     = gps_xy
        self.map_rc     = self.hash_gps_to_map()
        self.origin_xy  = origin_xy

    def hash_gps_to_map(self):
        if self.gps_xy[1] is not None:
            map_rc = convert_gps_to_map(self.gps_xy, self.map)
            # Update object worldMap if necessary
            if self.map_rc != map_rc:
                self.update_cell_table(map_rc)
            return map_rc
        else:
            return None

    def update_cell_table(self , cell_rc):
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
        map.world_berry_list.append(self)

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

        map.world_zombie_list.append(self)

class wallObject():
    def __init__(self, map, wall_color='brown', gps_xy=None, typeid='wall', moveable=False):
        super().__init__(map,gps_xy,typeid)
        self.moveable = moveable
        self.color    = "green" if self.moveable else "brown"
        map.world_solid_list.append(self)

class cellObject():
    def __init__(self, map, friction=None, danger_level=None):
        super().__init__(map,None,'cell')
        self.friction = friction # scaler value based on slippage of wheels and/or youbot speed for == acceleration
        self.danger_level = danger_level # based on zombies we think are around it and our historical health score change when we are in this cell
        map.cell_properties_list.append(self)

#%%  Define Functions

def getObjectRGB(object):

    color_map = {
        "zombie": "purple" , "berry": "red",
        "purple_zombie":"purple" , "green_zombie":"green",
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
    objects   = map.world_object_list
    positions = [obj.gps_xy for obj in objects]

    assign_object(map , gps_xy)

def assign_object(map , gps_xy):
    pass

#-------------- Plotting Functions ------------
def plot_init(map):

    ubot    = map.youbot
    zombies = map.world_zombie_list
    berries = map.world_berry_list
    solids  = map.world_solid_list

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
    youbot.wheels     = wheels

    map.timestep      = timestep

    return youbot

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

                # mainMap.map_lidar(i, lidar_values[i], orientation)


        # if plotMap:
        #     new_objects = get_objects_to_update(world_map)
        #
        #     update_plot(map, new_objects, ax)

def simulate_main(nzombies=3,nberries=10,ntimesteps=100):
#%%
    zombie_colors = ("purple", "green", "blue", "aqua")
    berry_colors  = ("pink", "orange", "red" , "yellow")

    map = worldMapObject()

    # Init youbot
    ubot = youbotObject(map,gps_xy=[0,0],init_gps_xy=[0,0])

    zombies = [zombieObject(map,zombie_color=zombie_colors[random.randint(0,3)]) for x in range(nzombies)]
    berries = [berryObject(map, berry_color=berry_colors[random.randint(0, 3)]) for x in range(nberries)]

    # Init Plot and plot initial positions
    ax = plot_init(map)

    # Run simulation and plot
    for t in range(ntimesteps):
        # Update youbot position
        pass
        # Update zombie positions

#%% Start simple simulation

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
    youbot   = init_youbot(world_map)
    robot    = youbot.wb_robot
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
        image  = youbot.sensors["camera"].getImage()
        width  = youbot.sensors["camera"].getWidth()
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

#%% ----------- Sandbox -----------

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

#%% Run time step by time step
# NOTE: must run once after moving in Webots manually to get the current sensor readings!

tmp = robot.step(TIME_STEP)
youbot.sensors["gps"].getValues()

# ------------- Individual Sandboxes ---------------
# This allows us to write longer codeblocks that won't be run upon syncing with webots

def sandbox_dc():
#%% Sandbox for Dan
    # tmp = robot.step(TIME_STEP)
    youbot.sensors["gps"].getValues()

    image  = youbot.sensors["camera"].getImage()
    width  = youbot.sensors["camera"].getWidth()
    height = youbot.sensors["camera"].getHeight()

    np_u    = np.frombuffer(image, dtype=np.uint8)
    np_img  = np_u.reshape(height, width, 4)
    rgb_img = np_img[:, :, :3]
    # hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

    plt.ion()
    ax = plt.subplots()

    plt.imshow(rgb_img)



def sandbox_wp():
#%% Sandbox for Witt
    tmp = robot.step(TIME_STEP)
    youbot.sensors["gps"].getValues()

def sandbox_ma():
#%% Sandbox for Mohammad
    tmp = robot.step(TIME_STEP)
    youbot.sensors["gps"].getValues()

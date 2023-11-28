"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs


#Addditional packages/ constants---------------------------------------------------------------
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import numpy as npy
from controller import Display
import math
_2PI = 2 * math.pi
#--------------------------------------------------------------------------------


# Map data structure
    # Uses a hash map with GPS-derived coords as keys.
    # Values are objects in MapCell, which hold relevant cell info.
class Map:
    def __init__(self, cellWidth, currCell):
         self.cellWidth = cellWidth #in meters
         self.cellTable = {}
         self.currCell = currCell
         self.initialReading = []
        
    def gps_to_map(self, initial, gpsVal):
        dist_from_origin = gpsVal - initial
        index = dist_from_origin // self.cellWidth
        return int(index)
        
    def map_lidar(self, beamNum, dist, ori):
        #Normalize angle
        angle = ((_2PI / 512) * beamNum) + ori
        if (angle > _2PI):
            angle = angle - _2PI
        #print("Beam", beamNum , "read distance", round(dist, 3), "at angle", math.degrees(angle))
        
        #Find coords & map
        xPos = math.cos(angle) * dist
        yPos = math.sin(angle) * dist
        xMapped = self.gps_to_map(self.initialReading[0], xPos)
        yMapped = self.gps_to_map(self.initialReading[1], yPos)
        coords = str_coords(xMapped, yMapped)
        #print("Coords are", coords)

        #print("Object detected at (", xPos, ",", yPos, ")")
        
        if self.cellTable.get(coords) is None:
            #Toggle 'solid' field
            self.cellTable[coords] = MapCell(xMapped, yMapped, None, None, None, True)
            print("New solid detected at", coords, "marked solid")
        else:
            cell = self.cellTable.get(coords)
            cell.solid = True
            print("Existing cell at", coords, "marked solid")

class MapCell:
    def __init__(self, xPos, yPos, berryType, zombieType, visited, solid):
        self.xPos = xPos
        self.yPos = yPos
        self.coordStr = "(" + str(xPos) + ", " + str(yPos) + ")"
        self.berryType = berryType
        self.zombieType = zombieType
        self.visited = visited
        self.solid = solid

#Note - this code is translated from the C version available in the documentation.  
def get_comp_angle(compassVals):
    angle = math.atan2(compassVals[1], compassVals[0]) # got rid of angle_rad - 1.5708
    if (angle < 0.0):
        angle = angle + _2PI
    return angle

def str_coords(x, y):
    return "(" + str(x) + ", " + str(y) + ")"
    
# #Testing curr_pos visualization
# plt.ion()
# all_curr_pos_x = []
# all_curr_pos_y = []

#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)

    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    camera2 = robot.getDevice("ForwardHighResSmallFov")
    camera2.enable(timestep)
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    lidar.enablePointCloud()
    
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
    
    
    #Initialize main map
    mainMap = Map(.1, None)
    
    #Establish relative center from GPS
    initialX = round(gps.getValues()[0], 3)
    initialY = round(gps.getValues()[2], 3)
    mainMap.initialReading = [initialX, initialY]
    print("Initial unmapped coords:", mainMap.initialReading)

    init_cell = MapCell(0, 0, None, None, None, None)
    print("Init Cell has coords:", init_cell.coordStr)
    
   
    mainMap.cellTable[init_cell.coordStr] = init_cell
    mainMap.currCell = init_cell.coordStr
    
    #Temp variable to track coord change.
    temp_coords = ""

    while robot.step(TIME_STEP) != -1:
        #Get sensor data
        gps_values = gps.getValues()
        lidar_values = lidar.getRangeImage()
        compass_values = compass.getValues()
        
        orientation = get_comp_angle(compass_values)
        gpsX = round(gps_values[0], 3)
        gpsY = round(gps_values[2], 3)
        
        #Cast gps readings to map coords
        mappedX = mainMap.gps_to_map(mainMap.initialReading[0], gpsX)
        mappedY = mainMap.gps_to_map(mainMap.initialReading[1], gpsY)
        coords = str_coords(mappedX, mappedY)
        
        #Create new dictionary entry if cell unencountered
        if mainMap.cellTable.get(coords) is None:
            mainMap.cellTable[coords] = MapCell(mappedX, mappedY, None, None, None, None)    
        
        #Manipulate current cell
        mainMap.currCell = mainMap.cellTable.get(coords)
        curr_cell = mainMap.cellTable.get(coords)
        curr_cell.visited = True
        
        # all_curr_pos_x.append(curr_cell.xPos)
        # all_curr_pos_y.append(curr_cell.xPos)
        
        #Lidar Mapping
        for i in range(len(lidar_values)):
            if lidar_values[i] != float('inf') and lidar_values[i] != 0.0:
                mainMap.map_lidar(i, lidar_values[i], orientation)
                #print(lidar_values)
                #print("Orientation is", orientation)
                #print("i =", i, ",lidar_values[i] =", lidar_values[i], ", ori =", orientation)

       
        #Motor Control     
        fr.setVelocity(2.0)
        fl.setVelocity(4.0)
        br.setVelocity(2.0)
        bl.setVelocity(4.0)
        
        #Testing
        # testingCoords = mainMap.cellTable.get(coords).coordStr
        # #print("Current cell visited?:", mainMap.cellTable.get(coords).visited)        
        # #print("GPS yields", gpsX, ",", gpsY)
        # if testingCoords != temp_coords:
            # print("Map Position:", testingCoords)
        # temp_coords = testingCoords
        
        # #Plotting
        # plt.xlim(min(all_curr_pos_x) - 5, max(all_curr_pos_x) + 5)
        # plt.ylim(min(all_curr_pos_y) - 5, max(all_curr_pos_y) + 5)
        # plt.scatter(all_curr_pos_x, all_curr_pos_y, color='black')
        # plt.pause(timestep)
        # #plt.show()
        
    
    i=0
         

    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   
         #called every timestep
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        
        #make decisions using inputs if you choose to do so
        
                 
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
    
    return 0   


main()



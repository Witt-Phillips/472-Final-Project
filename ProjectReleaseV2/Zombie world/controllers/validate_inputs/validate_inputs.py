"""youbot_controller controller."""

from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
    
# Map data structure
    # Uses a hash map with GPS-derived coords as keys.
    # Values are objects in MapCell, which hold relevant cell info.
class Map:
    def __init__(self, cellWidth, currCell):
         self.cellWidth = cellWidth #in meters
         self.cellTable = {}
         self.currCell = currCell
        
    def gps_to_map(self, initialReading, gpsVal):
        dist_from_origin = gpsVal - initialReading
        index = dist_from_origin // self.cellWidth
        return int(index)

class MapCell:
    def __init__(self, xPos, yPos, berryType, zombieType, visited, obstacle):
        self.xPos = xPos
        self.yPos = yPos
        self.coordStr = "(" + str(xPos) + ", " + str(yPos) + ")"
        self.berryType = berryType
        self.zombieType = zombieType
        self.visited = visited
        self.obstacle = obstacle

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
   
    mappedX = mainMap.gps_to_map(initialX, initialX)
    mappedY = mainMap.gps_to_map(initialY, initialY)
    
    init_cell = MapCell(mappedX, mappedY, None, None, None, None)
    #Verify we are centered at (0, 0)
    print("Init Cell has coords:", init_cell.coordStr)
    
   
    mainMap.cellTable[init_cell.coordStr] = init_cell
    mainMap.currCell = init_cell.coordStr
    
    #Temp variable to track coord change.
    temp_coords = ""

    while robot.step(TIME_STEP) != -1:
        #Get sensor data
        gps_values = gps.getValues()
        lidar_values = lidar.getRangeImage()
        orientation = compass.getValues()
        gpsX = round(gps_values[0], 3)
        gpsY = round(gps_values[2], 3)
        
        #Cast gps readings to map coords
        mappedX = mainMap.gps_to_map(initialX, gpsX)
        mappedY = mainMap.gps_to_map(initialY, gpsY)
        coords = "(" + str(mappedX) + ", " + str(mappedY) + ")"
        
        #Create new dictionary entry if cell unencountered
        if mainMap.cellTable.get(coords) is None:
            mainMap.cellTable[coords] = MapCell(mappedX, mappedY, None, None, None, None)    
        
        #Manipulate current cell
        curr_cell = mainMap.cellTable.get(coords)
        curr_cell.visited = True
        
        
        #Testing
        testingCoords = mainMap.cellTable.get(coords).coordStr
        #print("Current cell visited?:", mainMap.cellTable.get(coords).visited)        
        #print("GPS yields", gpsX, ",", gpsY)
        if testingCoords != temp_coords:
            print("Map Position:", testingCoords)
        temp_coords = testingCoords
        
        #Motor Control     
        fr.setVelocity(2.0)
        fl.setVelocity(4.0)
        br.setVelocity(2.0)
        bl.setVelocity(4.0)
    
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

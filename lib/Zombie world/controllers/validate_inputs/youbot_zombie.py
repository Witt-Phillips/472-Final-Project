#zombie/berry stuff

import random

TIME_STEP = 32

berry_test = True
zombie_test = True
berry_pos = [[0.0]*3 for i in range(40)]

berry_found = 0

def get_all_berry_pos(supervisor):

    #print(berry_pos)
    num_berries = 0
    for i in range(0,40):
        berry_node = supervisor.getFromDef("Berry"+str(i))
        if(berry_node == None):
            break
            
        num_berries = i+1
        
    print(num_berries)
    
    for i in range(0,num_berries):
        berry_node = supervisor.getFromDef("Berry"+str(i))
        #print(berry_node)
        berry_trans_field = berry_node.getField("translation")
        #print(berry_trans_field)
        berry_trans = berry_trans_field.getSFVec3f()
        berry_pos[i][0] = berry_trans[0]
        berry_pos[i][1] = berry_trans[1]
        berry_pos[i][2] = berry_trans[2]
        #print(berry_trans)  

def print_health_energy(robot_info):
    print("HEALTH: " + str(robot_info[0]) + ", ENERGY: " + str(robot_info[1]) + ", ARMOUR: " + str(robot_info[2])) 
    
   
def berry_collision(berry_id, robot_info, supervisor):
        
        #robot_info is health, energy, armour
        
        berry_node = supervisor.getFromDef("Berry" + str(berry_id))
        berry_trans_field = berry_node.getField("translation")
        #print(berry_trans_field)
        berry_trans = berry_trans_field.setSFVec3f([20,20,20])
        
        secondary = False
        berry_type = 0
        
        if(berry_id>=0 and berry_id<=12):
            berry_type=1
        elif(berry_id>=13 and berry_id<=21):
            berry_type=2
        elif(berry_id>=22 and berry_id<=30):
            berry_type=3
        elif(berry_id>=31 and berry_id<=39):
            berry_type=4
                  
        randomNum = random.randint(1,4)
        
        if(randomNum == 4):
            secondary == True
        
        if(not secondary):
            if berry_type==1:
                robot_info[1] += 40
            elif berry_type==2:
                robot_info[0] += 20
            elif berry_type==3:
                robot_info[1] -= 20 
            elif berry_type==4:
                robot_info[2] += 15
        
        if(secondary):
            if berry_type==1:
                robot_info[0] += 20
            elif berry_type==2:
                robot_info[1] -= 20
            elif berry_type==3:
                robot_info[2] += 15 
            elif berry_type==4:
                robot_info[1] += 40 
                
        
        if(robot_info[1]>100):
            robot_info[1] = 100
        
        if(robot_info[0] > 100):
            robot_info[0]=100
            
        if(robot_info[1] <0):
            robot_info[1] = 0
        
        return robot_info
        
def check_zombie_collision(robot_info, robot_x, robot_z, supervisor):
    
    
    num_zombies = 0
    for i in range(0,40):
        zombie_node = supervisor.getFromDef("zombie"+str(i))
        if(zombie_node == None):
            break
        
        num_zombies = i+1
    
    
    for i in range(0,num_zombies):
        zombie_node = supervisor.getFromDef("zombie" + str(i))
        zombie_trans_field = zombie_node.getField("translation")
        #print(berry_trans_field)
        zombie_trans = zombie_trans_field.getSFVec3f()
        
        zombie_x = zombie_trans[0]
        zombie_z = zombie_trans[2]
        
        distance = abs(zombie_x-robot_x) + abs(zombie_z-robot_z)
        
        if(distance<0.5 and robot_info[2]<1):
            robot_info[0]-=1
        
    
    
    return robot_info


def check_berry_collision(robot_info, robot_x, robot_z, supervisor):
   
    global berry_found
    
    num_berries = 0
    for i in range(0,40):
        berry_node = supervisor.getFromDef("Berry"+str(i))
        if(berry_node == None):
            break
            
        num_berries = i+1
    
    for i in range(0,num_berries):
       
        berry_x = berry_pos[i][0]
        berry_y = berry_pos[i][1]
        berry_z = berry_pos[i][2]
        
        distance = abs(berry_x-robot_x) + abs(berry_z-robot_z)
        
        if(distance < 0.38 and berry_y < 0.07):
            berry_collision(i, robot_info, supervisor)
            berry_pos[i][0] = 40
            berry_pos[i][2] = 40
            #berry_found+=1
            #exit()
    #if(berry_found == num_berries):
        #print("TEST PASSED")
        #supervisor.simulationQuit(20)
        
    return robot_info
    
    
def update_robot(robot_info):
    
    if (robot_info[2] >0):
        robot_info[2] -= 1  
    
    if (robot_info[1] == 0):
        robot_info[0] -= 1
    
    if(robot_info[1] > 0): 
        robot_info[1] -= 1
        
    print_health_energy(robot_info)
    
    return robot_info
    

def passive_wait(sec, robot, timestep):
    start = robot.getTime()
    
    while(start+sec > robot.getTime()):
        if(robot.step(timestep) == -1):
            exit()
    
    
    
    
           
             
            
        
        
        
        
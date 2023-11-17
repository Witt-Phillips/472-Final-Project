# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Pedestrian class container."""
from controller import Supervisor
from controller import Emitter

import optparse
import math
import random
import struct 

#notjing

class Pedestrian (Supervisor):
    def __init__(self):
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.speed = 1.15
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = ["leftArmAngle", "leftLowerArmAngle", "leftHandAngle","rightArmAngle", "rightLowerArmAngle", "rightHandAngle","leftLegAngle", "leftLowerLegAngle", "leftFootAngle","rightLegAngle", "rightLowerLegAngle", "rightFootAngle","headAngle"]
        self.height_offsets = [-0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03]  # those coefficients are empirical coefficients which result in a realistic walking gait
        self.angles = [  # those coefficients are empirical coefficients which result in a realistic walking gait
                [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
                [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
                [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
                [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
                [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
                [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
                [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
                [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
                [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
                [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
                [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
                [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
                [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]
        Supervisor.__init__(self)

    def check_berry_close(self,zombiePos):
        for i in range(0,20):
            berry_id = "Berry" + str(i);
            berryNode = self.getFromDef(berry_id)
            
            if berryNode is not None:
                berryTranslationField = berryNode.getField('translation')
                berryPos = berryTranslationField.getSFVec3f()
    
                a_2 = (berryPos[0] - zombiePos[0])*(berryPos[0] - zombiePos[0])
                b_2 = (berryPos[2] - zombiePos[2])*(berryPos[2] - zombiePos[2])
                c = math.sqrt(a_2+b_2)
                if (c<2):
                    return berryPos
        return [30,30]

    

    def move_to_point(self,x,z):
        current_sequence = 0
        ratio = 0.1
        time = self.getTime()
        for i in range(0, self.BODY_PARTS_NUMBER):
            current_angle = self.angles[i][current_sequence] * (1 - ratio) + self.angles[i][(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio
            self.joints_position_field[i].setSFFloat(current_angle)
			
        # adjust height
        self.current_height_offset = self.height_offsets[current_sequence] * (1 - ratio) + self.height_offsets[(current_sequence + 1) % self.WALK_SEQUENCES_NUMBER] * ratio

        # move everything
        distance = time * self.speed
        relative_distance = distance - int(distance / self.waypoints_distance[self.number_of_waypoints - 1]) * self.waypoints_distance[self.number_of_waypoints - 1]

        for i in range(0, self.number_of_waypoints):
            if self.waypoints_distance[i] > relative_distance:
                break

        distance_ratio = 0
        if i == 0:
            distance_ratio = relative_distance / self.waypoints_distance[0]
        else:
            distance_ratio = (relative_distance - self.waypoints_distance[i - 1]) / (self.waypoints_distance[i] - self.waypoints_distance[i - 1])
            
        x = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][0] + (1 - distance_ratio) * self.waypoints[i][0]
        z = distance_ratio * self.waypoints[(i + 1) % self.number_of_waypoints][1] + (1 - distance_ratio) * self.waypoints[i][1]
        root_translation = [x, self.ROOT_HEIGHT + self.current_height_offset, z]
        angle = math.atan2(self.waypoints[(i + 1) % self.number_of_waypoints][0] - self.waypoints[i][0], self.waypoints[(i + 1) % self.number_of_waypoints][1] - self.waypoints[i][1])
        rotation = [0, 1, 0, angle]
                
        self.root_translation_field.setSFVec3f(root_translation)
        self.root_rotation_field.setSFRotation(rotation)
        
            
    def calculate_angle(self, zombie_x, zombie_z, target_x, target_z):                               
        ang = math.atan2(target_x - zombie_x, target_z - zombie_z)
        return ang
        
    def move_zombie(self, zombie_x, zombie_z, target_x, target_z):
        move_x = ((target_x - zombie_x)/ (abs(target_x - zombie_x) + abs(target_z - zombie_z))) *0.04 #0.01
        move_z = ((target_z - zombie_z) / (abs(target_x - zombie_x) + abs(target_z - zombie_z))) * 0.04 #0.01
        
        #print (abs(move_x) + abs(move_z))
        root_translation = [zombie_x + move_x, self.ROOT_HEIGHT + self.current_height_offset, zombie_z+move_z]
        
        ang = self.calculate_angle(zombie_x, zombie_z, target_x, target_z)
        #angle = math.radians(ang)
        rotation = [0, 1, 0, ang]
        self.translationField.setSFVec3f(root_translation)
        self.rotationField.setSFRotation(rotation)
        
    def youbotDistance(self, youbotPos, zombiePos):
        #distance = absolute(youbotPos[0] - zombiePos[0]) + absolute(youbotPos[2] - zombiePos[2]);
        a_2 = (youbotPos[0] - zombiePos[0])*(youbotPos[0] - zombiePos[0])
        b_2 = (youbotPos[2] - zombiePos[2])*(youbotPos[2] - zombiePos[2])
        c = math.sqrt(a_2+b_2)
        return c
		  
    def random_zombie(self):
        
        self.robotNode = self.getSelf()
        self.translationField = self.robotNode.getField('translation')
        self.rotationField = self.robotNode.getField("rotation")
        
        youbotNode = self.getFromDef('Youbot')
        youbotTranslationField = youbotNode.getField('translation')
        
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(self.robotNode.getField(self.joint_names[i]))
            
        timer = 0
        self.time_step = int(self.getBasicTimeStep())
        goal = [5,5]
        time_at_berry = 0
        emitter = Emitter("emitter")
        message = struct.pack("chd",b"p",100,120.08)
        emitter.setChannel(-1)
        emitter.setRange(4)
        while not self.step(self.time_step) == -1:
            self.translation = self.translationField.getSFVec3f()
            self.move_zombie(self.translation[0], self.translation[2], goal[0],goal[1])
            
            if (timer == 32): #only change movement once every second
                timer = 0
                emitter.send(message)
                youbotTranslation = youbotTranslationField.getSFVec3f()

                berryDistance = self.check_berry_close(self.translation)
                if (berryDistance[0]!=30) and (time_at_berry < 10):
                    #print (berryDistance)
                    goal =  [berryDistance[0],berryDistance[2]]
                    time_at_berry = time_at_berry + 1


                elif (self.youbotDistance(youbotTranslation, self.translation) < 3):#if robot close, chase it
                    x_goal = youbotTranslation[0]
                    z_goal = youbotTranslation[2]
                    #goal = [x_goal, z_goal]
                    goal = [youbotTranslation[0],youbotTranslation[2]]
                    #print ("close to robot")

                    
                else: #choose a random spot
                    if (random.randint(0,5) == 2): 
                        goal = [random.randint(int(self.translation[0]) -5, int(self.translation[0]) + 5), random.randint(int(self.translation[2]) -5, int(self.translation[2]) + 5)]
                        if (goal[0] < -12):
                            goal[0] = -12
                        if (goal[0] > 12):
                            goal[0] = 12
                        if (goal[1] < -12):
                            goal[1] = -12
                        if (goal[1] > 12):
                            goal[1] = 12
                if ((goal[0] <= -1) and (self.translation[0] > -1) and (self.translation[2] >0)):
                    goal[0] = -0.5
                if ((goal[0] >= -1) and (self.translation[0] < -1) and (self.translation[2] >0)):
                    goal[0] = -1.5
                if ((goal[1] <= -5) and (self.translation[2] > -5) and (self.translation[0] >-4)):
                    goal[1] = -4.5
                if ((goal[1] >= -5) and (self.translation[2] < -5) and (self.translation[0] >-4)):
                    goal[1] = -5.5
                if (time_at_berry < 30 and time_at_berry > 10):
                    #print ("waiting till next berry")
                    time_at_berry = time_at_berry+ 1
                if (time_at_berry > 29): #8 seconds of berry cooldown
                    time_at_berry = 0
                #print (time_at_berry)
            timer = timer +1
		


controller = Pedestrian()
controller.random_zombie()


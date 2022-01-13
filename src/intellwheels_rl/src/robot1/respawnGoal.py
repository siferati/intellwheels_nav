#!/usr/bin/env python

import rospy
import random
import time
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose


class Respawn():
    def __init__(self, random_goal_position = True):
        self.modelPath = os.path.dirname(os.path.realpath(__file__))
        self.modelPath = self.modelPath.replace('intellwheels_rl/src/robot1',
                                                'intellwheels_rl/meshes/goal_box/model.sdf')
        self.f = open(self.modelPath, 'r')
        self.model = self.f.read()
        self.goal_position = Pose()
        self.init_goal_x = 1.5
        self.init_goal_y = -4.2
        self.random_goal_position = random_goal_position
        self.goal_position.position.x = self.init_goal_x
        self.goal_position.position.y = self.init_goal_y
        self.modelName = 'goal'
        self.sub_model = rospy.Subscriber('gazebo/model_states', ModelStates, self.checkModel)
        self.check_model = False

    def checkModel(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def respawnModel(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(self.modelName, self.model, 'robotos_name_space', self.goal_position, "world")
                rospy.loginfo("Goal position : %.1f, %.1f", self.goal_position.position.x, self.goal_position.position.y)                
                break
            else:
                pass

    def deleteModel(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass

    def getPosition(self, delete):
        if delete:
            self.deleteModel()

        if(self.random_goal_position):
            self.goal_position.position.x = random.randrange(-20, 17) / 10.0
            self.goal_position.position.y = random.randrange(-42, -30) / 10.0

        rospy.loginfo("Goal at .... = %f , %f ", self.goal_position.position.x, self.goal_position.position.y)
        
        time.sleep(0.5)
        self.respawnModel()
        return self.goal_position.position.x, self.goal_position.position.y
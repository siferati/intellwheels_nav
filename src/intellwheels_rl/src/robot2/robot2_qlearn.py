#!/usr/bin/env python

import rospy
import os
import json
import numpy as np
import random
import time
import ast

import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from collections import deque
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64

from robot2.robot2_environment import Env
from algorithms.qlearn import QLearn
from tools.train_qlearn_log import TrainQlearnLog

def __numpy_to_string(A):
    return str(round(np.sum(A), 3))

if __name__ == '__main__':
    rospy.init_node('robot2_qlearn_2')

    #log file path 
    modelPath = os.path.dirname(os.path.realpath(__file__))
    modelPath = modelPath.replace('intellwheels_rl/src/robot2','intellwheels_rl/save_model')

    # log
    path_to_save_csv = modelPath + os.sep + "robot2_qlearn.csv"
    traing_log = TrainQlearnLog(path_to_save_csv)

    state_size = 12 # 
    action_size = 5
    
    chair2_speed =  rospy.get_param('/robot2_qlearn/chair2_speed')

    env = Env(action_size,chair2_speed, 'robot2_qlearn_close_to_chair.csv', 'robot2_qlearn_trajectory.csv')
    last_time_steps = np.ndarray(0)

    
    Alpha = 0.1
    Epsilon = 0.7
    Gamma = 0.9
    epsilon_discount = 0.95
    nepisodes = 100000
    nsteps = 5000

    Alpha = rospy.get_param("/robot2_qlearn/alpha")
    Epsilon = rospy.get_param("/robot2_qlearn/epsilon")
    Gamma = rospy.get_param("/robot2_qlearn/gamma")
    epsilon_discount = rospy.get_param("/robot2_qlearn/epsilon_discount")
    nepisodes = rospy.get_param("/robot2_qlearn/nepisodes")
    nsteps = rospy.get_param("/robot2_qlearn/nsteps")
    RELEASE = rospy.get_param("/robot2_qlearn/release")

    qlearn = QLearn(actions=range(action_size),alpha=Alpha, gamma=Gamma, epsilon=Epsilon)

    initial_epsilon = qlearn.epsilon
    start_time = time.time()
    highest_reward = 0

    for tepisode in range(nepisodes):
        cumulated_reward = 0
        collision = False
        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount
       
       
        state = env.reset()
        state = __numpy_to_string(state)
               
        for tstep in range(nsteps):
            
            action = qlearn.chooseAction(state)
            nextState, reward, collision, goal = env.step(action, tepisode, tstep)
            nextState = __numpy_to_string(nextState)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

           
            qlearn.learn(state, action, reward, nextState)

            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            f_time = str(h) + ":" + str(m)  + ":" + str(s)
            
            traing_log.save(tepisode, tstep ,str(qlearn.alpha),str(qlearn.gamma), str(initial_epsilon), epsilon_discount , str(cumulated_reward) , highest_reward , f_time )

            if RELEASE == False:
                if not(collision):
                    state = nextState
                else:
                    rospy.logdebug("DONE EPISODE!")
                    last_time_steps = np.append(last_time_steps, [int(tstep + 1)])
                    break
            else:
                state = nextState

        rospy.logwarn( ("EP: "+str(tepisode+1)+" - [alpha: "+str(round(qlearn.alpha,2))+" - gamma: "+str(round(qlearn.gamma,2))+" - epsilon: "+str(round(qlearn.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))


    l = last_time_steps.tolist()
    l.sort()


    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()

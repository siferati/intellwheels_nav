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

from robot1.robot1_environment import Env
from algorithms.qlearn import QLearn
from tools.train_qlearn_log import TrainQlearnLog


def __numpy_to_string(A):
    return str(round(np.sum(A), 3))

if __name__ == '__main__':
    rospy.init_node('robot1_qlearn_1')

    #log file path 
    modelPath = os.path.dirname(os.path.realpath(__file__))
    modelPath = modelPath.replace('intellwheels_rl/src/robot1','intellwheels_rl/save_model')

    # log
    path_to_save_csv = modelPath + os.sep + "robot1_qlearn.csv"
    traing_log = TrainQlearnLog(path_to_save_csv)

    state_size = 12 # 
    action_size = 5

    chair1_speed =  rospy.get_param('/robot1_qlearn/chair1_speed')
    
    env = Env(action_size,  chair1_speed, True,  "robot1_qlearn_goal.csv", "robot1_qlearn_trajectory.csv")
    last_time_steps = np.ndarray(0)

    
    Alpha = rospy.get_param("/robot1_qlearn/alpha")
    Epsilon = rospy.get_param("/robot1_qlearn/epsilon")
    Gamma = rospy.get_param("/robot1_qlearn/gamma")
    epsilon_discount = rospy.get_param("/robot1_qlearn/epsilon_discount")
    nepisodes = rospy.get_param("/robot1_qlearn/nepisodes")
    nsteps = rospy.get_param("/robot1_qlearn/nsteps")
    
    
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

        print("Episode: ", tepisode)            
               
        # for each episode, we test the robot for nsteps
        for tstep in range(nsteps):

            #print("Episode: ", tepisode, " Step: ", tstep)            

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)
            
            nextState, reward, collision, goal = env.step(action, tepisode, tstep)
            nextState = __numpy_to_string(nextState)

            cumulated_reward += reward
            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            rospy.logdebug("env.get_state...==>" + str(nextState))

            # Make the algorithm learn based on the results
            '''
            print("Types..................")
            print("Episode: ", tepisode, " Step: ", tstep)
            print("state: ",  type(state) , " ", state   )
            print("action: ", type(action), "  ", action)
            print("reward: ", type(reward), " " , reward )
            print("nextState: ", type(nextState), " ", nextState)
            print("")
            '''

            qlearn.learn(state, action, reward, nextState)

            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            f_time = str(h) + ":" + str(m)  + ":" + str(s)

            traing_log.save(tepisode, tstep ,str(qlearn.alpha),str(qlearn.gamma), str(initial_epsilon), epsilon_discount , str(cumulated_reward) , highest_reward , f_time )
       
            if not(collision):
                state = nextState
            else:
                rospy.logdebug("DONE EPISODE!")
                last_time_steps = np.append(last_time_steps, [int(tstep + 1)])
                break

            #rospy.logdebug("###################### END Step...["+str(i)+"]")

        
        rospy.logwarn( ("EP: "+str(tepisode+1)+" - [alpha: "+str(qlearn.alpha)+" - gamma: "+str(qlearn.gamma)+" - epsilon: "+str(qlearn.epsilon)+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s)))

    rospy.loginfo ( ("\n|"+str(nepisodes)+"|"+str(qlearn.alpha)+"|"+str(qlearn.gamma)+"|"+str(initial_epsilon)+"*"+str(epsilon_discount)+"|"+str(highest_reward)+"| PICTURE |"))

    l = last_time_steps.tolist()
    l.sort()

    #print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, l[-100:]) / len(l[-100:])))

    env.close()

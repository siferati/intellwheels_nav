#!/usr/bin/env python

import rospy
import os
import os.path
import json
import numpy as np
import random
import time
import pandas as pd

import tensorflow as tf
from datetime import datetime

import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from collections import deque
from std_msgs.msg import Float32MultiArray
from robot1.robot1_environment_stage_1 import Env

from keras.models import Sequential, load_model
from keras.optimizers import RMSprop
from keras.layers import Dense, Dropout, Activation
from tensorflow.keras.callbacks import TensorBoard


EPISODES = 100

class ReinforceAgent():
    def __init__(self, state_size, action_size, use_tensorboard=False):
        self.pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=50)
        self.dirPath = os.path.dirname(os.path.realpath(__file__))

        self.dirPath = self.dirPath.replace('intellwheels_rl/src/robot1', 'intellwheels_rl/save_model/robot1_stage_1_')
        
        self.result = Float32MultiArray()

#       # Load model from last EPISODE
        self.load_model = True # If 'False', start from scratch
        self.load_episode = 90 # If 'True' start from this episode number 'self.load_episode'
        # ----------------------------

        self.state_size = state_size
        self.action_size = action_size
        self.episode_step = 1000
        self.target_update = 200
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64
        self.memory = deque(maxlen=1000000)

        self.model = self.buildModel()
        self.target_model = self.buildModel()
        
        if use_tensorboard:

            tensor_board_model_path = os.getcwd() + os.sep + os.path.join('tb_model_chair1') + os.sep + datetime.now().strftime("%Y%m%d-%H%M%S")
            if not os.path.exists(tensor_board_model_path):
                os.makedirs(tensor_board_model_path)

            self.tf_callback = TensorBoard(log_dir=tensor_board_model_path,update_freq=1)
            self.tf_callback.set_model(self.model)
            print("Save Model (model) on : ", tensor_board_model_path)

            tensor_board_target_path = os.getcwd() + os.sep  + os.path.join('tb_target_mode_chair1') + os.sep + datetime.now().strftime("%Y%m%d-%H%M%S")

            if not os.path.exists(tensor_board_target_path):
                os.makedirs(tensor_board_target_path)

            self.tf_callback2 = TensorBoard(log_dir=tensor_board_target_path,update_freq=1)
            self.tf_callback2.set_model(self.target_model)
            print("Save Model (target) on : ", tensor_board_target_path)

        self.updateTargetModel()

        if self.load_model:
            self.model.set_weights(load_model(self.dirPath+str(self.load_episode)+".h5").get_weights())

            with open(self.dirPath+str(self.load_episode)+'.json') as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')
        
    def buildModel(self):
        model = Sequential()
        dropout = 0.2
        model.add(Dense(64, input_shape=(self.state_size,), activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(dropout))
        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()
        return model

    def getQvalue(self, reward, next_target, done):
        if done:
            return reward
        else:
            return reward + self.discount_factor * np.amax(next_target)

    def updateTargetModel(self):
        self.target_model.set_weights(self.model.get_weights())

    def getAction(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)
        else:
            q_value = self.model.predict(state.reshape(1, len(state)))
            self.q_value = q_value
            return np.argmax(q_value[0])

    def appendMemory(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))


    #### .trainModel() loops '.batch_size = 64' times
    def trainModel(self, target=False):
        mini_batch = random.sample(self.memory, self.batch_size)
        X_batch = np.empty((0, self.state_size), dtype=np.float64)
        Y_batch = np.empty((0, self.action_size), dtype=np.float64)

        for i in range(self.batch_size):
            states = mini_batch[i][0]
            actions = mini_batch[i][1]
            rewards = mini_batch[i][2]
            next_states = mini_batch[i][3]
            dones = mini_batch[i][4]

            q_value = self.model.predict(states.reshape(1, len(states)))
            self.q_value = q_value

            if target:
                next_target = self.target_model.predict(next_states.reshape(1, len(next_states)))

            else:
                next_target = self.model.predict(next_states.reshape(1, len(next_states)))

            next_q_value = self.getQvalue(rewards, next_target, dones)

            X_batch = np.append(X_batch, np.array([states.copy()]), axis=0)
            Y_sample = q_value.copy()

            Y_sample[0][actions] = next_q_value
            Y_batch = np.append(Y_batch, np.array([Y_sample[0]]), axis=0)

            if dones:
                X_batch = np.append(X_batch, np.array([next_states.copy()]), axis=0)
                Y_batch = np.append(Y_batch, np.array([[rewards] * self.action_size]), axis=0)

        self.model.fit(X_batch, Y_batch, batch_size=self.batch_size, epochs=1, verbose=0, callbacks=[self.tf_callback])

if __name__ == '__main__':
    rospy.init_node('robot1_dqn_stage_1')

    #log file path 
    modelPath = os.path.dirname(os.path.realpath(__file__))
    modelPath = modelPath.replace('intellwheels_rl/src/robot1','intellwheels_rl/save_model')
    path_to_save_csv = modelPath + os.sep + "robot1.csv"

    pub_result = rospy.Publisher('result', Float32MultiArray, queue_size=5)
    pub_get_action = rospy.Publisher('get_action', Float32MultiArray, queue_size=5)
    
    result = Float32MultiArray()
    get_action = Float32MultiArray()

    state_size = 722 # input of the network
    action_size = 5

    env = Env(action_size)

    agent = ReinforceAgent(state_size, action_size, True)
    scores, episodes = [], []
    global_step = 0
    start_time = time.time()

    rospy.loginfo("+++++++++++++++++++++++++++++++++++++++++++++++++++++")

    if agent.load_model:
        rospy.loginfo("START TRAINING MODEL FROM episode = %d", agent.load_episode)
    else:
        rospy.loginfo("START TRAINING MODEL FROM scratch") 
    
    rospy.loginfo("=====================================================")
    
    # Run number of 'EPISODES = 3000'
    # 'global_step' resets to 0 for every new EPISODE

    # Determine the number of the starting episode
    if agent.load_model == False:
        load_episode = 0
    else:
        load_episode = agent.load_episode
    
    
    # Run every new episode
    for e in range(agent.load_episode + 1, EPISODES):
        done = False
        state = env.reset()
        score = 0
        # Loop t over a maximum of '.episode_step = 6000'
        #    if t >= 500 => it causes a rospy.loginfo("Time out!!")
        for t in range(agent.episode_step):

            #print("Episode ", t ," of ",  agent.episode_step)

            action = agent.getAction(state)
            next_state, reward, done = env.step(action)
            agent.appendMemory(state, action, reward, next_state, done)

            if len(agent.memory) >= agent.train_start:
                # global_step  <=,else  (.target_update = 2000)
                if global_step <= agent.target_update:
                    # Prediction using '.target_model'
                    agent.trainModel()
                else:
                    # Prediction using  '.model'
                    agent.trainModel(True)

            score += reward
            state = next_state
            get_action.data = [action, score, reward]
            pub_get_action.publish(get_action)

            # save the model at 10 in 10 steps
            if e % 10 == 0:
                agent.model.save(agent.dirPath + str(e) + '.h5')
                with open(agent.dirPath + str(e) + '.json', 'w') as outfile:
                    json.dump(param_dictionary, outfile)

            if t >= 500:
                rospy.loginfo("Time out!!")
                done = True

            if done:
                result.data = [score, np.max(agent.q_value)]
                pub_result.publish(result)
                agent.updateTargetModel()
                scores.append(score)
                episodes.append(e)

                m, s = divmod(int(time.time() - start_time), 60)
                h, m = divmod(m, 60)
                rospy.loginfo('Ep: %d score: %.2f memory: %d epsilon: %.2f time: %d:%02d:%02d',
                              e, score, len(agent.memory), agent.epsilon, h, m, s)

                # save log              

                data_csv = [[ e, score, np.max(agent.q_value) ,len(agent.memory), agent.epsilon, str(h) + ":" + str(m)  + ":" + str(s) ]]
                df = pd.DataFrame(data_csv, columns = ['Episode', 'Score', 'q-value' , 'Memory', 'Epsilon', 'Time'])
                df.to_csv(path_to_save_csv, mode='a', header=(e==0))

                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))
                break

            global_step += 1
            
            if global_step % agent.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")
    
        if agent.epsilon > agent.epsilon_min:
            agent.epsilon *= agent.epsilon_decay
        
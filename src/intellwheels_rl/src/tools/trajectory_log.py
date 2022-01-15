import pandas as pd
import time

class TrajectoryLog():
    def __init__(self, path):
        self.path = path

    def save(self, episode, step, posx, posy, goalposx, goalposy):
        data_csv = [[ episode, step, posx, posy,  goalposx, goalposy ]]
        df = pd.DataFrame(data_csv, columns = ['Episode', 'Step', 'PosX','PosY', 'GoalPosX','GoalPosY'])
        df.to_csv(self.path, mode='a', header=False)
        
    
    
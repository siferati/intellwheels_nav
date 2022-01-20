import pandas as pd
import numpy as np

class TrainDQNLog():
    def __init__(self, path):
        self.path = path

    def save(self, episode, score, q_value, epsilon, time, collision, goal ):

        data_csv = [[ episode, score, np.max(q_value), epsilon, time, str(collision) , str(goal) ]]
        df = pd.DataFrame(data_csv, columns = ['Episode', 'Score', 'q-value','Epsilon', 'Time', 'collision', 'goal'])
        df.to_csv(self.path, mode='a', header=False)
    
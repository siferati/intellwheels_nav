import pandas as pd

class TrainQlearnLog():
    def __init__(self, path):
        self.path = path

    def save(self, episode, step, alpha, gamma, epsilon, reward, time):     
        data_csv = [[ episode, step, alpha, gamma, epsilon, reward, time ]]
        df = pd.DataFrame(data_csv, columns = ['episode', 'step', 'alpha', 'epsilon','gamma','reward', 'time'])
        df.to_csv(self.path, mode='a', header=False)
    
import pandas as pd

class TrainQlearnLog():
    def __init__(self, path):
        self.path = path

    def save(self, episode, step, alpha, gamma, epsilon, epsilon_discount, reward, max_reward, time):     
        data_csv = [[ episode, step, alpha, gamma, epsilon, epsilon_discount, reward, max_reward , time ]]
        df = pd.DataFrame(data_csv, columns = ['episode', 'step', 'alpha', 'epsilon', 'epsilon_discount' ,'gamma','reward', 'max_reward', 'time'])
        df.to_csv(self.path, mode='a', header=False)
    
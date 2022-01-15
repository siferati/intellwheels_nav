import pandas as pd

class TrainDQNLog():
    def __init__(self, path):
        self.path = path

    def save(self, episode, score, q_value, memory, epsilon, time, timeout, collision, goal ):

        data_csv = [[ episode, score, np.max(agent.q_value), len(agent.memory), agent.epsilon, 
                            time, str(timeout) , str(collision) , str(goal) ]]
        df = pd.DataFrame(data_csv, columns = ['Episode', 'Score', 'q-value',
                            'Memory', 'Epsilon', 'Time', 'timeout', 'collision', 'goal'])
        df.to_csv(self.path, mode='a', header=False)
    
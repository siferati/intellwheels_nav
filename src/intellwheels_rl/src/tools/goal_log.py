import pandas as pd
import time

class GoalLog():
    def __init__(self, path):
        self.path = path
        self.start_time = time.time()

    def save(self, episode, step, action):

        m, s = divmod(int(time.time() - self.start_time), 60)
        h, m = divmod(m, 60)

        format_time = str(h) + ":" + str(m)  + ":" + str(s)

        data_csv = [[ episode, step, action, format_time ]]

        df = pd.DataFrame(data_csv, columns = ['Episode', 'Step', 'Action','Time'])
        df.to_csv(self.path, mode='a', header=False)
    
    
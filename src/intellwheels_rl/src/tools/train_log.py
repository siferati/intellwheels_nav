import pandas as pd

class TrainLog():
    def __init__(self, path):
        self.path = path

    def save(self, dataFrame):
        df = pd.DataFrame(dataFrame, columns = ['Episode', 'Score', 'q-value',
                             'Memory', 'Epsilon', 'Time', 
                             'timeout', 'collision', 'goal'])
        df.to_csv(self.path, mode='a', header=False)
    
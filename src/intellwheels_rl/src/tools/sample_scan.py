import numpy as np


class SampleScan():
    def __init__(self, limit):
        self.limit = limit

    def clean_data(self, scan):
        scan_range = []
        for i in range(len(scan)):
            if scan[i] == float('Inf'):
                scan_range.append(15)
            elif np.isnan(scan[i]):
                scan_range.append(0)
            else:
                scan_range.append(scan[i])
        return scan_range

    def get_sample_from_laser_scan(self, ranges):
            laser_hit_dst = 10000
            index = 0
            total_items = len(ranges)
            sample = [] 
            for i in range(total_items):

                if(ranges[i] != float('Inf') and  np.isnan(ranges[i]) == False and ranges[i] < laser_hit_dst):
                    laser_hit_dst = ranges[i] 
                    index = i               
            
            # get the neighbour arround the min. hit
            if(index - self.limit < 0 ):
                sample = np.array (ranges[0:self.limit])
            elif (index + self.limit >  total_items ):
                sample = np.array (ranges[(total_items - self.limit):total_items])
            else:
                sample = np.array (ranges[(index - self.limit/2) : (index + self.limit/2)])

            return sample
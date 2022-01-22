import matplotlib.pyplot as plt
import csv
import ast


def get_data_from_csv(filename):
	chair_trial_x = []
	chair_trial_y = []
	
 	with open(filename,'r') as csvfile:
		lines = csv.reader(csvfile, delimiter=';')
		for row in lines:
			if(row[0] != ''):
				chair_trial_x.append(float(ast.literal_eval(row[0])))
			if(row[1] != ''):
				chair_trial_y.append(float(ast.literal_eval(row[1])))
			
 	return chair_trial_x, chair_trial_y

if __name__ == "__main__":

	chair_trial_1_x, chair_trial_1_y = get_data_from_csv('robot1_qlearn_all_01_trajectory.csv')
	chair_trial_2_x, chair_trial_2_y = get_data_from_csv('robot1_qlearn_all_02_trajectory.csv')
	chair_trial_3_x, chair_trial_3_y = get_data_from_csv('robot1_qlearn_all_03_trajectory.csv')
	chair_trial_4_x, chair_trial_4_y = get_data_from_csv('robot1_qlearn_all_04_trajectory.csv')
	chair_trial_5_x, chair_trial_5_y = get_data_from_csv('robot1_qlearn_all_05_trajectory.csv')

	plt.scatter(1.0, -5.0, color = '#FF0000',s = 200)
	plt.scatter(chair_trial_1_x, chair_trial_1_y, marker='s', color = '#800080',s = 10, label='Trial 1')
 	plt.scatter(chair_trial_2_x, chair_trial_2_y, marker='^', color = '#0040FF',s = 10, label='Trial 2')
 	plt.scatter(chair_trial_3_x, chair_trial_3_y, marker='*', color = '#FE2E2E',s = 10, label='Trial 3')
 	plt.scatter(chair_trial_4_x, chair_trial_4_y, marker='D', color = '#FFC0CB',s = 10, label='Trial 4')
 	plt.scatter(chair_trial_5_x, chair_trial_5_y, marker='X', color = '#000000',s = 10, label='Trial 5')
  	plt.legend()
	
 	plt.xticks(rotation = 25)
	plt.xlabel('X (m)')
	plt.ylabel('Y (m)')
	plt.title('Trajectory of the Chair 1 using Q-Learning', fontsize = 10)

	plt.xlim([-1, 3.5])
	plt.ylim([-6, 1])
	plt.show()

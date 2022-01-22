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

	chair_trial_c1_x_t1_r1, chair_trial_c1_y_t1_r1 = get_data_from_csv('robot_two_robots_qlearn_all_01_trajectory_robot1.csv')
	chair_trial_c2_x_t1_r2, chair_trial_c2_y_t1_r2 = get_data_from_csv('robot_two_robots_qlearn_all_01_trajectory_robot2.csv')
 
 	chair_trial_c1_x_t2_r1, chair_trial_c1_y_t2_r1 = get_data_from_csv('robot_two_robots_qlearn_all_03_trajectory_robot1.csv')
	chair_trial_c2_x_t2_r2, chair_trial_c2_y_t2_r2 = get_data_from_csv('robot_two_robots_qlearn_all_03_trajectory_robot2.csv')

	chair_trial_c1_x_t3_r1, chair_trial_c1_y_t3_r1 = get_data_from_csv('robot_two_robots_qlearn_all_05_trajectory_robot1.csv')
	chair_trial_c2_x_t3_r2, chair_trial_c2_y_t3_r2 = get_data_from_csv('robot_two_robots_qlearn_all_05_trajectory_robot2.csv')


	plt.scatter(1.0, -5.0, color = '#FF0000',s = 200)
	
 	plt.scatter(chair_trial_c1_x_t1_r1, chair_trial_c1_y_t1_r1, marker='s', color = '#800080',s = 5, label='Chair (leader): Trial 1')
 	plt.scatter(chair_trial_c2_x_t1_r2, chair_trial_c2_y_t1_r2, marker='^', color = '#0040FF',s = 5, label='Chair (follower): Trial 1')
  
  	plt.scatter(chair_trial_c1_x_t2_r1, chair_trial_c1_y_t2_r1, marker='*', color = '#FE2E2E',s = 5, label='Chair (leader): Trial 2')
 	plt.scatter(chair_trial_c2_x_t2_r2, chair_trial_c2_y_t2_r2, marker='D', color = '#FFC0CB',s = 5, label='Chair (follower): Trial 2')
  
  	plt.scatter(chair_trial_c1_x_t3_r1, chair_trial_c1_y_t3_r1, marker='X', color = '#000000',s = 5, label='Chair (leader): Trial 3')
 	plt.scatter(chair_trial_c2_x_t3_r2, chair_trial_c2_y_t3_r2, marker='8', color = '#088A08',s = 5, label='Chair (follower): Trial 3')
 
 	plt.legend()
	plt.xticks(rotation = 25)
	plt.xlabel('X (m)')
	plt.ylabel('Y (m)')
	plt.title('Trajectory of the chair (leader) and chair (follower) using Q-Learning', fontsize = 10)

	plt.xlim([0.5, 7.5])
	plt.ylim([-7, 0])
	plt.show()
